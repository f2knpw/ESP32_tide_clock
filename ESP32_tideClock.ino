#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>   // ArduinoJson v7 requis (JsonDocument)
#include <ESPmDNS.h>
#include <NetBIOS.h>       // Network request for Windows
#include <Preferences.h>
#include <time.h>

#include "web_server.h"

// library for stepper motor
#include <AccelStepper.h>  //https://www.airspayce.com/mikem/arduino/AccelStepper/

// -------------------------------------------------------------------------
// RAM RTC
// RTC_NOINIT_ATTR : survives software reset, watchdog, esp_restart() and deep sleep.
// (RTC_DATA_ATTR is re-initialised on every reset except deep-sleep wake-up.)
// Content is random at power-on -> the magic number is mandatory.
// -------------------------------------------------------------------------
#define RTC_MAGIC_KEY 0xCAFE1234
RTC_NOINIT_ATTR uint32_t rtcMagicNumber;
RTC_NOINIT_ATTR long     rtcStepper1Pos;
RTC_NOINIT_ATTR long     rtcStepper2Pos;
RTC_NOINIT_ATTR uint32_t rtcWasNormal;   // 1 = system was in NORMAL mode before reset

// --- Signalisation ---
#define LED_PIN 22
unsigned long lastLedToggle = 0;

// --- Wi-Fi Configuration ---
String wifiSsid = "";
String wifiPassword = "";
const unsigned long WIFI_CHECK_INTERVAL_MS = 30000UL;
unsigned long lastWifiCheck = 0;

// --- API Maree ---
const char* apiKey = "YOUR_API_KEY"; //https://api-maree.fr/ --> for French Atlantic tides
const char* siteId = "boucau-bayonne-biarritz";
//attibution : Données de marée fournies par api-maree.fr sous licence CC BY, calculées à partir de composantes harmoniques Ifremer / PREVIMER, elles-mêmes sous licence CC BY.
const unsigned long API_RETRY_INTERVAL_MS = 10UL * 60UL * 1000UL; // retry every 10 min on failure
unsigned long lastApiAttempt = 0;

// --- Steppers pins ---
#define DIR1_PIN 14
#define STEP1_PIN 12
#define DIR2_PIN 33
#define STEP2_PIN 25
#define ENABLE_PIN 13

//Leds pins
#define EBB_TIDE_PIN 27
#define FLOOD_TIDE_PIN 26
// Configuration PWM LEDC
const int FLOOD_LEDC_CHANNEL = 0;
const int EBB_LEDC_CHANNEL   = 1;
const int LEDC_FREQ = 5000;     // 5 kHz
const int LEDC_RES = 8;         // Résolution 8 bits (0 - 255)
const int LED_BRIGHTNESS = 64;  // Intensity  (~25% de 255)
AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);

// --- Speeds ---
const float MANUAL_MAX_SPEED       = 1000.0f;
const float DRAIN_SPEED            = -500.0f;
const float CALIB_SPEED            = 400.0f;
const float NORMAL_MOVE_DURATION_S = 30.0f;   // time to reach the H+1 level (3600 for a smooth move over the hour)

// --- state machine and Calibration ---
enum SystemState {
  STATE_IDLE,           // Waiting
  STATE_NORMAL,         // normal tide Mode
  STATE_DRAINING_ZERO,  // draining the water tank (homing physical zero)
  STATE_CALIBRATING     // calibration step by step (using ladder rungs visually)
};

SystemState currentState = STATE_IDLE;
bool isHomed = false;   // true only when the physical zero is known

// --- physical ladder on the 3d model ---
const int TOTAL_RUNGS = 24;
const int MEAN_LEVEL_RUNG_INDEX = 10;                     // mid tide fixed to rung 10
int maxTargetRungCoef120 = 2 * MEAN_LEVEL_RUNG_INDEX;     // high tide coef 120 = rung 20 (exact by construction)
float meanWaterLevelHeight = 0.0;                         // today's mid tide level (m)
float stepHeightPerRung     = 0.0;                        // height per rung (m)
bool  scaleValid            = false;

long rungRelativeSteps[TOTAL_RUNGS]; // recorded motor steps for each rung
int currentCalibrationRung = 0;

// Variables for non blocking emergency draining
long drainTargetSteps = 0;
long drainStartPos = 0;

// --- Clock & System ---
bool isTimeAvailable = false;
int lastFetchedDay = -1;
int lastFetchedHour = -1;

Preferences preferences;
WiFiClientSecure secureClient;

// Tide time table: index 0..23 = today 00h..23h, index 24 = tomorrow 00h
const int LEVEL_SLOTS = 25;
float hourlyWaterLevels[LEVEL_SLOTS];
bool hourlyLevelsValid = false;

// --- functions Declarations ---
void setDualPumpsSpeed(float speed);
void stopDualPumps();
void processCalibrationCommand(String command);
void startEmptyingProcess();
void stopEmptyingProcess();
void confirmPhysicalZero();
void finishCalibration();
void saveRelativeRung(int rungIndex, long steps);
void loadCalibrationFromFlash();
void savePositionsToRTC();
bool restorePositionsFromRTC();
bool waitForNtp(unsigned long timeoutMs);
bool refreshTideData();
bool getTideExtremaAndComputeScale(const struct tm& day);
bool getWaterLevels(const struct tm& day);
bool httpGetJson(const String& url, JsonDocument& doc);
void printLocalTime();
bool calculateScaleFromTides(float highTideHeight, float lowTideHeight, int coefficient);
float convertHeightToRungIndex(float height);
long convertFractionalRungToAbsoluteSteps(float fractionalRung);
long convertRungToAbsoluteSteps(int targetRung);
void updateCurrentTideLevel();
void handleTideSchedule();
void handleWifi();
void handleLedBlink();
int getCurrentRungFromPosition();

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
 // Attach LEDC with frequency and resolution
 ledcSetup(FLOOD_LEDC_CHANNEL, LEDC_FREQ, LEDC_RES);
ledcAttachPin(FLOOD_TIDE_PIN, FLOOD_LEDC_CHANNEL);
ledcSetup(EBB_LEDC_CHANNEL, LEDC_FREQ, LEDC_RES);
ledcAttachPin(EBB_TIDE_PIN, EBB_LEDC_CHANNEL);
  // dimmed at startup
  ledcWrite(FLOOD_LEDC_CHANNEL, 0);
ledcWrite(EBB_LEDC_CHANNEL, 0);
  // Steppers Initialisation
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Activate drivers

  stepper1.setMaxSpeed(MANUAL_MAX_SPEED);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(MANUAL_MAX_SPEED);
  stepper2.setAcceleration(500);

  preferences.begin("tideClock", false);
  wifiSsid = preferences.getString("ssid", "YOUR_SSID");
  wifiPassword = preferences.getString("password", "YOUR_PASSWORD");

  loadCalibrationFromFlash();

  // get steppers position from RAM RTC
  // if magic number is invalid, a full tank drain is launched
  bool restored = restorePositionsFromRTC();

  // 1. Configure host name for Box / DHCP
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("tide");
  WiFi.setAutoReconnect(true);

  // 2. Connection to Wi-Fi
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  unsigned long startTime = millis();
  while ((WiFi.status() != WL_CONNECTED) && (millis() - startTime < 15000UL)) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected ! IP : " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WIFI] Connection failed. Will retry in background.");
  }

  // 3. DNS local mDNS (http://tide.local)
  if (MDNS.begin("tide")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS server started : http://tide.local");
  } else {
    Serial.println("Error while starting mDNS");
  }

  // 4. NetBIOS for Windows (http://tide)
  NBNS.begin("tide");

  // web server start
  setupWebServer();

  // No certificate check (acceptable for a hobby project).
  // Timeouts are handled by HTTPClient::setTimeout().
  secureClient.setInsecure();

  // NTP time (including summer/winter time)
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
  if (waitForNtp(10000)) {
    refreshTideData();
  }

  // Resume NORMAL mode automatically if the system was running before the reset
  if (restored && rtcWasNormal == 1) {
    currentState = STATE_NORMAL;
    lastFetchedHour = -1; // force a tide update on the first loop
    Serial.println("[SYSTEM] Resuming NORMAL mode after reset.");
  }
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
  handleWebServer();
  handleWifi();

  // handle LED blinking during draining (homing lost / security draining)
  if (currentState == STATE_DRAINING_ZERO) {
    handleLedBlink();
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // handling machine state and steppers
  switch (currentState) {
    case STATE_DRAINING_ZERO:
      stepper1.runSpeed();
      stepper2.runSpeed();

      // non blocking draining end
      if (labs(stepper1.currentPosition() - drainStartPos) >= drainTargetSteps) {
        confirmPhysicalZero();
        currentState = STATE_NORMAL;
        lastFetchedHour = -1; // force a tide update
        Serial.println("[SYSTEM] Emergency draining finished. Homing/Zero calibrated -> Switch to NORMAL MODE.");
      }
      break;

    case STATE_CALIBRATING:
      stepper1.runSpeed();
      stepper2.runSpeed();
      break;

    case STATE_NORMAL:
      stepper1.runSpeedToPosition();
      stepper2.runSpeedToPosition();
      break;

    case STATE_IDLE:
    default:
      break;
  }

  // save position to RAM RTC (only when the zero is known)
  if (currentState != STATE_DRAINING_ZERO) {
    savePositionsToRTC();
  }

  // tide data refresh + hourly update (no blocking HTTP while pumps are in manual motion)
  if (currentState == STATE_NORMAL || currentState == STATE_IDLE) {
    handleTideSchedule();
  }
}

void handleLedBlink() {
  if (millis() - lastLedToggle >= 250) {
    lastLedToggle = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void handleWifi() {
  if (millis() - lastWifiCheck < WIFI_CHECK_INTERVAL_MS) return;
  lastWifiCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Disconnected -> reconnecting...");
    WiFi.reconnect();
  }
}

void handleTideSchedule() {
  struct tm timeInfo;
  if (!getLocalTime(&timeInfo, 0)) return;   // non blocking
  isTimeAvailable = true;

  // New day (or data never fetched): refresh, with retry interval on failure
  if (timeInfo.tm_mday != lastFetchedDay) {
    if (lastApiAttempt == 0 || millis() - lastApiAttempt >= API_RETRY_INTERVAL_MS) {
      Serial.println("\n[NTP] New day / missing data : Api maree --> update tides.");
      if (refreshTideData()) {
        lastFetchedHour = -1; // force an update with the new data
      }
    }
  }

  if (currentState == STATE_NORMAL && timeInfo.tm_hour != lastFetchedHour) {
    lastFetchedHour = timeInfo.tm_hour;
    updateCurrentTideLevel();
  }
}

// -------------------------------------------------------------------------
// RAM RTC
// -------------------------------------------------------------------------
void savePositionsToRTC() {
  if (!isHomed) return;

  long p1 = stepper1.currentPosition();
  long p2 = stepper2.currentPosition();

  // During calibration the motor position is relative to the last validated rung
  if (currentState == STATE_CALIBRATING) {
    long base = convertRungToAbsoluteSteps(currentCalibrationRung);
    p1 += base;
    p2 += base;
  }

  rtcStepper1Pos = p1;
  rtcStepper2Pos = p2;
  rtcWasNormal   = (currentState == STATE_NORMAL) ? 1 : 0;
  rtcMagicNumber = RTC_MAGIC_KEY;
}

bool restorePositionsFromRTC() {
  if (rtcMagicNumber == RTC_MAGIC_KEY) {
    stepper1.setCurrentPosition(rtcStepper1Pos);
    stepper2.setCurrentPosition(rtcStepper2Pos);
    isHomed = true;

    Serial.println("==========================================");
    Serial.println("[RTC] Positions restored after reset :");
    Serial.printf("[RTC] stepper 1 : %ld steps | stepper 2 : %ld steps\n", rtcStepper1Pos, rtcStepper2Pos);
    Serial.println("==========================================");
    return true;
  }

  Serial.println("==========================================");
  Serial.println("[RTC] Magic Number invalid / Homing lost !");
  Serial.println("[RTC] Launch emergency draining.");
  Serial.println("==========================================");

  rtcWasNormal = 0;
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  startEmptyingProcess();
  return false;
}

// -------------------------------------------------------------------------
// Motors & Conversions
// -------------------------------------------------------------------------
long convertRungToAbsoluteSteps(int targetRung) {
  if (targetRung <= 0) return 0;
  if (targetRung > TOTAL_RUNGS) targetRung = TOTAL_RUNGS;

  long totalSteps = 0;
  for (int i = 0; i < targetRung; i++) {
    totalSteps += rungRelativeSteps[i];
  }
  return totalSteps;
}

long convertFractionalRungToAbsoluteSteps(float fractionalRung) {
  if (isnan(fractionalRung) || fractionalRung <= 0.0f) return 0;
  if (fractionalRung >= (float)TOTAL_RUNGS) return convertRungToAbsoluteSteps(TOTAL_RUNGS);

  int baseRung = (int)floorf(fractionalRung);
  if (baseRung < 0) baseRung = 0;
  if (baseRung > TOTAL_RUNGS - 1) baseRung = TOTAL_RUNGS - 1;
  float fraction = fractionalRung - (float)baseRung;

  long baseSteps = convertRungToAbsoluteSteps(baseRung);
  long currentRungSteps = rungRelativeSteps[baseRung];

  return baseSteps + lroundf(fraction * (float)currentRungSteps);
}

void updateCurrentTideLevel() {
  if (!isHomed || !scaleValid || !hourlyLevelsValid) {
    Serial.println("[NORMAL MODE] Update skipped (zero unknown or tide data missing).");
    return;
  }

  struct tm timeInfo;
  if (!getLocalTime(&timeInfo, 0)) return;

  int currentHour = timeInfo.tm_hour;
  int nextIndex = currentHour + 1;               // 1..24 (24 = tomorrow 00h)
  int nextHourDisplay = nextIndex % 24;
  
   int nextHour = (currentHour + 1) % 24;

  float currentHeight = hourlyWaterLevels[currentHour];
  float nextHeight = hourlyWaterLevels[nextHour];

  // --- LEDs following tide ---
  if (nextHeight > currentHeight) {
    // Flood tide
    ledcWrite(FLOOD_LEDC_CHANNEL, LED_BRIGHTNESS);
    ledcWrite(EBB_LEDC_CHANNEL, 0);
  } else if (nextHeight < currentHeight) {
    // Ebb tide
    ledcWrite(FLOOD_LEDC_CHANNEL, 0);
    ledcWrite(EBB_LEDC_CHANNEL, LED_BRIGHTNESS);
  } else {
    // no tide
    ledcWrite(FLOOD_LEDC_CHANNEL, 0);
    ledcWrite(EBB_LEDC_CHANNEL, 0);
  }



  nextHeight = hourlyWaterLevels[nextIndex];
  float nextFractionalRung = convertHeightToRungIndex(nextHeight);
  long targetSteps = convertFractionalRungToAbsoluteSteps(nextFractionalRung);

  long currentPosition = stepper1.currentPosition();
  long deltaSteps = targetSteps - currentPosition;
  float moveSpeed = (float)deltaSteps / NORMAL_MOVE_DURATION_S;

  Serial.printf("\n[NORMAL MODE] %02dh00 -> %02dh00 | Target (H+1) : %.3f m (Rung %.2f)\n",
                currentHour, nextHourDisplay, nextHeight, nextFractionalRung);
  Serial.printf("Position : %ld steps | Target : %ld steps | Speed : %.4f steps/sec\n",
                currentPosition, targetSteps, moveSpeed);

  stepper1.moveTo(targetSteps);
  stepper2.moveTo(targetSteps);

  // setSpeed() must be called AFTER moveTo() (moveTo recomputes the speed)
  setDualPumpsSpeed(moveSpeed);
}

void setDualPumpsSpeed(float speed) {
  // AccelStepper::setSpeed() clamps to maxSpeed: always restore a sufficient maxSpeed
  float maxS = fabsf(speed);
  if (maxS < MANUAL_MAX_SPEED) maxS = MANUAL_MAX_SPEED;

  stepper1.setMaxSpeed(maxS);
  stepper2.setMaxSpeed(maxS);
  stepper1.setSpeed(speed);
  stepper2.setSpeed(speed);
}

void stopDualPumps() {
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  stepper1.stop();
  stepper2.stop();
}

// -------------------------------------------------------------------------
// Calibration & Commands handling
// -------------------------------------------------------------------------
void startEmptyingProcess() {
  // Position becomes unknown as soon as draining starts
  isHomed = false;
  rtcMagicNumber = 0;
  rtcWasNormal = 0;

  currentState = STATE_DRAINING_ZERO;
  drainStartPos = stepper1.currentPosition();

  // Total volume of the whole ladder (margin above coef 120 rung)
  drainTargetSteps = convertRungToAbsoluteSteps(TOTAL_RUNGS);
  if (drainTargetSteps <= 0) drainTargetSteps = 24000; // Security: calibration empty

  setDualPumpsSpeed(DRAIN_SPEED);
  Serial.printf("\n[HOMING] draining during %ld steps.\n", drainTargetSteps);
}

void stopEmptyingProcess() {
  stopDualPumps();
  digitalWrite(LED_PIN, LOW);
  currentState = STATE_IDLE;
  Serial.println("[HOMING] Draining interrupted via Web command. Switching to IDLE.");
  Serial.println("[HOMING] WARNING : zero unknown. Send EMPTY_OK if the tank is empty, or SYS:HOME to drain again.");
}

void confirmPhysicalZero() {
  stopDualPumps();
  digitalWrite(LED_PIN, LOW);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  isHomed = true;

  savePositionsToRTC();
}

void processCalibrationCommand(String command) {
  command.trim();

  // 1. security draining
  if (command == "CALIB:START" || command == "SYS:HOME") {
    startEmptyingProcess();
  }

  // explicit web stop of draining
  else if (command == "SYS:STOP_DRAIN") {
    if (currentState == STATE_DRAINING_ZERO) {
      stopEmptyingProcess();
    }
  }

  // 2. Homing Validation -> switch to IDLE (also allowed after an interrupted draining)
  else if (command == "NO_WATER" || command == "EMPTY_OK") {
    if (currentState == STATE_DRAINING_ZERO || (currentState == STATE_IDLE && !isHomed)) {
      confirmPhysicalZero();
      currentState = STATE_IDLE;
      Serial.println("[SYSTEM] Physical zero confirmed. System IDLE.");
    }
  }

  // 3. switch to Normal Mode (only from IDLE, with a known zero)
  else if (command == "SYS:START_NORMAL") {
    if (currentState == STATE_IDLE) {
      if (!isHomed) {
        Serial.println("[SYSTEM] Refused : zero unknown. Send SYS:HOME first.");
        return;
      }
      stopDualPumps();
      currentState = STATE_NORMAL;
      lastFetchedHour = -1; // force a tide update on the next loop
      Serial.println("[SYSTEM] Switch to Normal MODE.");
    }
  }

  // 4. launch step by step calibration (only from IDLE, tank empty)
  else if (command == "CALIB:BEGIN_STEPS") {
    if (currentState == STATE_IDLE) {
      if (!isHomed || stepper1.currentPosition() != 0) {
        Serial.println("[CALIB] Refused : tank must be empty (send SYS:HOME then EMPTY_OK).");
        return;
      }
      stopDualPumps();
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      currentCalibrationRung = 0;
      currentState = STATE_CALIBRATING;
      Serial.printf("[CALIB] Calibration started. Target high tide Coef 120 : rung %d/%d\n",
                    maxTargetRungCoef120, TOTAL_RUNGS);

      setDualPumpsSpeed(CALIB_SPEED);
    }
  }

  // 5. manual control of pumps
  else if (command == "PUMP:UP") {
    if (currentState == STATE_CALIBRATING) setDualPumpsSpeed(CALIB_SPEED);
  }
  else if (command == "PUMP:DOWN") {
    if (currentState == STATE_CALIBRATING) setDualPumpsSpeed(-CALIB_SPEED);
  }
  else if (command == "PUMP:STOP") {
    if (currentState == STATE_DRAINING_ZERO) {
      stopEmptyingProcess();
    } else if (currentState == STATE_CALIBRATING) {
      stopDualPumps();
    }
  }

  // 6. Validation of current rung index (water has reached the rung)
  else if (command == "CALIB:NEXT") {
    if (currentState != STATE_CALIBRATING) return;

    stopDualPumps();
    long stepsMade = stepper1.currentPosition();

    if (stepsMade <= 0) {
      Serial.printf("[CALIB] Refused : %ld steps since last rung (must be > 0). Use PUMP:UP.\n", stepsMade);
      return;
    }

    if (currentCalibrationRung < maxTargetRungCoef120) {
      currentCalibrationRung++;
      saveRelativeRung(currentCalibrationRung, stepsMade);

      long totalAccumulatedSteps = convertRungToAbsoluteSteps(currentCalibrationRung);

      Serial.println("==========================================");
      Serial.printf("[CALIB] Rung validated : %d / %d (Target Coef 120 : %d)\n",
                    currentCalibrationRung, TOTAL_RUNGS, maxTargetRungCoef120);
      Serial.printf("[CALIB] Steps measured for rung %d : %ld steps\n", currentCalibrationRung, stepsMade);
      Serial.printf("[CALIB] total steps recorded : %ld steps\n", totalAccumulatedSteps);
      Serial.println("==========================================");

      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);

      if (currentCalibrationRung >= maxTargetRungCoef120) {
        finishCalibration();
      } else {
        setDualPumpsSpeed(CALIB_SPEED);
      }
    }
  }

  // 7. Stop calibration
  else if (command == "CALIB:END") {
    if (currentState == STATE_CALIBRATING) {
      stopDualPumps();

      // Restore the ABSOLUTE position (motor position was relative to the last rung)
      long absolutePos = convertRungToAbsoluteSteps(currentCalibrationRung) + stepper1.currentPosition();
      stepper1.setCurrentPosition(absolutePos);
      stepper2.setCurrentPosition(absolutePos);

      currentState = STATE_IDLE;
      savePositionsToRTC();

      Serial.println("==========================================");
      Serial.printf("[CALIB] Calibration stopped at rung %d/%d (position %ld steps). Go back to IDLE.\n",
                    currentCalibrationRung, maxTargetRungCoef120, absolutePos);
      Serial.println("==========================================");
    }
  }
}

void finishCalibration() {
  long lastValidStep = (currentCalibrationRung > 0) ? rungRelativeSteps[currentCalibrationRung - 1] : 1000;

  // Extrapolate the rungs above the coef 120 target
  for (int i = currentCalibrationRung; i < TOTAL_RUNGS; i++) {
    saveRelativeRung(i + 1, lastValidStep);
  }

  long totalStepsCurrent = convertRungToAbsoluteSteps(currentCalibrationRung);
  stepper1.setCurrentPosition(totalStepsCurrent);
  stepper2.setCurrentPosition(totalStepsCurrent);

  currentState = STATE_IDLE;
  savePositionsToRTC();

  Serial.println("==========================================");
  Serial.printf("[CALIB] High tide Coef 120 reached at rung %d/%d.\n", currentCalibrationRung, maxTargetRungCoef120);
  Serial.printf("[CALIB] Total volume reached : %ld steps.\n", totalStepsCurrent);
  Serial.println("[CALIB] Saved to flash. Switch to IDLE.");
  Serial.println("==========================================");
}

void saveRelativeRung(int rungIndex, long steps) {
  if (rungIndex < 1 || rungIndex > TOTAL_RUNGS) return;
  rungRelativeSteps[rungIndex - 1] = steps;

  String key = "rung_" + String(rungIndex);
  preferences.putLong(key.c_str(), steps);
}

void loadCalibrationFromFlash() {
  for (int i = 1; i <= TOTAL_RUNGS; i++) {
    String key = "rung_" + String(i);
    long v = preferences.getLong(key.c_str(), 1000);
    rungRelativeSteps[i - 1] = (v > 0) ? v : 1000;
  }
  Serial.println("[SYSTEM] Calibration loaded from Flash.");
}

// -------------------------------------------------------------------------
// Scale
// -------------------------------------------------------------------------
bool calculateScaleFromTides(float highTideHeight, float lowTideHeight, int coefficient) {
  if (coefficient <= 0 || highTideHeight <= lowTideHeight) {
    Serial.println("[SCALE] Invalid data (coef or extrema), scale not updated.");
    return false;
  }

  // 1. today's mid tide (rung 10)
  float newMean = (highTideHeight + lowTideHeight) / 2.0f;

  // 2. half range extrapolated to coefficient 120
  float observedHalfRange = (highTideHeight - lowTideHeight) / 2.0f;
  float halfRange120      = observedHalfRange * (120.0f / (float)coefficient);

  // 3. half range coef 120 = exactly 10 rungs
  float newStep = halfRange120 / (float)MEAN_LEVEL_RUNG_INDEX;
  if (newStep <= 0.0f) return false;

  meanWaterLevelHeight = newMean;
  stepHeightPerRung    = newStep;
  scaleValid           = true;

  // 4. high tide coef 120 = rung 20 by construction (no float ceil() rounding issue)
  maxTargetRungCoef120 = 2 * MEAN_LEVEL_RUNG_INDEX;

  Serial.printf("[SCALE] Half tide (rung 10) : %.2f m | height/rung : %.3f m | Low tide Coef 120 (Rung 0) : %.2f m\n",
                meanWaterLevelHeight, stepHeightPerRung, meanWaterLevelHeight - halfRange120);
  return true;
}

float convertHeightToRungIndex(float height) {
  // Protection against division by zero if the scale is not known yet
  if (!scaleValid || stepHeightPerRung <= 0.0f) return (float)MEAN_LEVEL_RUNG_INDEX;

  float heightDifference = height - meanWaterLevelHeight;

  // linear projection around rung 10
  float fractionalRung = (float)MEAN_LEVEL_RUNG_INDEX + (heightDifference / stepHeightPerRung);

  if (isnan(fractionalRung) || fractionalRung < 0.0f) return 0.0f;
  if (fractionalRung > (float)TOTAL_RUNGS) return (float)TOTAL_RUNGS;

  return fractionalRung;
}

// -------------------------------------------------------------------------
// API & Network
// -------------------------------------------------------------------------
bool waitForNtp(unsigned long timeoutMs) {
  struct tm t;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (getLocalTime(&t, 0)) {
      isTimeAvailable = true;
      printLocalTime();
      return true;
    }
    delay(200);
  }
  Serial.println("[NTP] Sync failed, will retry in loop.");
  return false;
}

bool refreshTideData() {
  lastApiAttempt = millis();
  if (lastApiAttempt == 0) lastApiAttempt = 1; // 0 is reserved for "never tried"

  struct tm today;
  if (!getLocalTime(&today, 0)) return false;

  // Old data belongs to another day: do not use it any more
  if (today.tm_mday != lastFetchedDay) hourlyLevelsValid = false;

  bool okScale = getTideExtremaAndComputeScale(today);
  delay(1000);
  bool okLevels = okScale && getWaterLevels(today);

  if (okScale && okLevels) {
    lastFetchedDay = today.tm_mday;
    Serial.println("[API] Tide data updated successfully.");
    return true;
  }

  Serial.printf("[API] Tide data update FAILED, retry in %lu min.\n", API_RETRY_INTERVAL_MS / 60000UL);
  return false;
}

bool httpGetJson(const String& url, JsonDocument& doc) {
  if (WiFi.status() != WL_CONNECTED) return false;

  for (int attempt = 1; attempt <= 2; attempt++) {
    HTTPClient http;
    http.setTimeout(15000);
    http.useHTTP10(true);  // avoids chunked encoding, required for stream parsing
    bool ok = false;

    if (http.begin(secureClient, url)) {
      http.setUserAgent("ESP32-TideClock");
      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK) {
        DeserializationError error = deserializeJson(doc, http.getStream());
        if (!error) {
          ok = true;
        } else {
          Serial.printf("[API] JSON error : %s\n", error.c_str());
        }
      } else {
        Serial.printf("[API] HTTP error %d (attempt %d)\n", httpCode, attempt);
      }
      http.end();
    }
    secureClient.stop();

    if (ok) return true;
    delay(1000);
  }
  return false;
}

bool getTideExtremaAndComputeScale(const struct tm& day) {
  char dateStr[11];
  strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &day);

  String baseUrl = "https://api-maree.fr/tide-extrema?";
  baseUrl += "site=" + String(siteId);
  baseUrl += "&from=" + String(dateStr);
  baseUrl += "&to=" + String(dateStr);
  baseUrl += "&tz=Europe/Paris";
  Serial.println(baseUrl + "&key=***");

  JsonDocument doc;
  if (!httpGetJson(baseUrl + "&key=" + String(apiKey), doc)) return false;

  JsonArray dataArray = doc["data"].as<JsonArray>();
  if (dataArray.isNull() || dataArray.size() == 0) return false;

  JsonArray extremaArray = dataArray[0]["extrema"].as<JsonArray>();
  if (extremaArray.isNull() || extremaArray.size() == 0) return false;

  float lowestTideHeight = 99.0;
  float highestTideHeight = -99.0;
  int currentCoefficient = 0;

  for (JsonObject item : extremaArray) {
    if (!item["height"].is<float>()) continue;
    float h = item["height"].as<float>();
    if (h < lowestTideHeight) lowestTideHeight = h;
    if (h > highestTideHeight) highestTideHeight = h;

    JsonVariant c = item["coef"];
    if (!c.isNull()) {
      int v = c.as<int>();
      if (v > 0) currentCoefficient = v;
    }
  }

  return calculateScaleFromTides(highestTideHeight, lowestTideHeight, currentCoefficient);
}

bool getWaterLevels(const struct tm& day) {
  char todayStr[11];
  strftime(todayStr, sizeof(todayStr), "%Y-%m-%d", &day);

  char dateFrom[17];
  strftime(dateFrom, sizeof(dateFrom), "%Y-%m-%dT00:00", &day);

  struct tm tomorrowInfo = day;
  tomorrowInfo.tm_mday += 1;
  tomorrowInfo.tm_isdst = -1;
  mktime(&tomorrowInfo);

  char dateTo[17];
  strftime(dateTo, sizeof(dateTo), "%Y-%m-%dT00:00", &tomorrowInfo);

  String baseUrl = "https://api-maree.fr/water-levels?";
  baseUrl += "site=" + String(siteId);
  baseUrl += "&from=" + String(dateFrom);
  baseUrl += "&to=" + String(dateTo);
  baseUrl += "&step=60";
  baseUrl += "&tz=Europe/Paris";
  Serial.println(baseUrl + "&key=***");

  JsonDocument doc;
  if (!httpGetJson(baseUrl + "&key=" + String(apiKey), doc)) return false;

  JsonArray dataArray = doc["data"].as<JsonArray>();
  if (dataArray.isNull()) return false;

  float levels[LEVEL_SLOTS];
  bool filled[LEVEL_SLOTS];
  for (int i = 0; i < LEVEL_SLOTS; i++) { levels[i] = 0.0f; filled[i] = false; }

  for (JsonObject item : dataArray) {
    String dtStr = item["datetime"].is<const char*>() ? item["datetime"].as<String>()
                                                       : item["time"].as<String>();
    if (!item["height"].is<float>()) continue;

    int tIndex = dtStr.indexOf('T');
    if (tIndex < 10 || (int)dtStr.length() < tIndex + 3) continue;

    String datePart = dtStr.substring(0, tIndex);
    int hour = dtStr.substring(tIndex + 1, tIndex + 3).toInt();
    if (hour < 0 || hour > 23) continue;

    int idx;
    if (datePart == todayStr) idx = hour;          // today 00h..23h
    else if (hour == 0)       idx = 24;            // tomorrow 00h
    else continue;

    if (filled[idx]) continue;                     // DST day: duplicated hour, keep the first one

    levels[idx] = item["height"].as<float>();
    filled[idx] = true;
  }

  int count = 0;
  for (int i = 0; i < LEVEL_SLOTS; i++) if (filled[i]) count++;
  if (count < 20) {
    Serial.printf("[API] Not enough water levels received (%d/%d).\n", count, LEVEL_SLOTS);
    return false;
  }

  // Fill missing slots (DST day, missing values) by linear interpolation
  for (int i = 0; i < LEVEL_SLOTS; i++) {
    if (filled[i]) continue;
    int p = i - 1; while (p >= 0 && !filled[p]) p--;
    int n = i + 1; while (n < LEVEL_SLOTS && !filled[n]) n++;
    if (p >= 0 && n < LEVEL_SLOTS) levels[i] = levels[p] + (levels[n] - levels[p]) * (float)(i - p) / (float)(n - p);
    else if (p >= 0)               levels[i] = levels[p];
    else                           levels[i] = levels[n];
  }

  for (int i = 0; i < LEVEL_SLOTS; i++) {
    hourlyWaterLevels[i] = levels[i];
    Serial.printf("Slot %02d%s | Height : %.3f m --> Rung Index: %.2f\n",
                  i % 24, (i == 24) ? " (J+1)" : "", levels[i], convertHeightToRungIndex(levels[i]));
  }

  hourlyLevelsValid = true;
  Serial.println("[API] Today's tide time table recorded successfully");
  return true;
}

void printLocalTime() {
  struct tm timeInfo;
  if (!getLocalTime(&timeInfo, 0)) {
    Serial.println("Error Sync NTP");
    return;
  }
  Serial.println(&timeInfo, "%A %d %B %Y %H:%M:%S");
}

int getCurrentRungFromPosition() {
  // if in calibration
  if (currentState == STATE_CALIBRATING) {
    return currentCalibrationRung;
  }

  long currentPos = stepper1.currentPosition();
  if (currentPos <= 0) return 0;

  // read cumulated steps to determine current rung
  long accumulatedSteps = 0;
  for (int i = 0; i < TOTAL_RUNGS; i++) {
    accumulatedSteps += rungRelativeSteps[i];
    if (currentPos < accumulatedSteps) {
      return i; // will reach this rung
    }
  }

  return TOTAL_RUNGS;
}
