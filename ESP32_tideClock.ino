#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include <TimeLib.h>
#include <ESPmDNS.h>
#include <NetBIOS.h> // Network request for Windows

#include <WiFiUdp.h>
#include <Preferences.h>

#include "web_server.h"

// library for stepper motor
#include <AccelStepper.h> //https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <MultiStepper.h>

// --- RAM RTC (maintained variables  post-reset / deep sleep) ---
#define RTC_MAGIC_KEY 0xCAFE1234
RTC_DATA_ATTR uint32_t rtcMagicNumber = 0;
RTC_DATA_ATTR long rtcStepper1Pos = 0;
RTC_DATA_ATTR long rtcStepper2Pos = 0;

// --- Signalisation ---
#define LED_PIN 22
unsigned long lastLedToggle = 0;

// --- Wi-Fi Configuration ---
String wifiSsid = "";
String wifiPassword = "";

// --- API Maree ---
const char* apiKey = "YOUR APIKEY"; //https://api-maree.fr/ --> for French Atlantic tides
const char* siteId = "boucau-bayonne-biarritz";
//attibution : Données de marée fournies par api-maree.fr sous licence CC BY, calculées à partir de composantes harmoniques Ifremer / PREVIMER, elles-mêmes sous licence CC BY.


// --- Steppers pins ---
#define DIR1_PIN 14
#define STEP1_PIN 12
#define DIR2_PIN 33
#define STEP2_PIN 25
#define ENABLE_PIN 13

AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);

// --- state machine and Calibration ---
enum SystemState {
  STATE_IDLE,           // Waiting
  STATE_NORMAL,         // normal tide Mode
  STATE_DRAINING_ZERO,  // draining the water tank (homing physical zero)
  STATE_CALIBRATING     // calibration step by step (using ladder rungs visually)
};

SystemState currentState = STATE_IDLE;
const int TOTAL_RUNGS = 24;
long rungRelativeSteps[TOTAL_RUNGS]; // recorded motor steps for each rung
int currentCalibrationRung = 0;
int maxTargetRungCoef120 = TOTAL_RUNGS; // initialization Rung for Coef 120 (will be dynmically computed later)

// Variables for non blocking emergency draining
long drainTargetSteps = 0;
long drainStartPos = 0;

// --- Clock & System ---
bool isTimeAvailable = false;
int timeZoneOffsetHours = 1;
const int daylightSavingOffsetHours = 1;
int lastFetchedDay = -1;
int lastFetchedHour = -1;

Preferences preferences;
WiFiClientSecure secureClient;

// --- physical ladder on the 3d model ---
const int MEAN_LEVEL_RUNG_INDEX = 10; // mid tide fixed to rung 10
float meanWaterLevelHeight = 0.0;     // today's mid tide level(meters)
float stepHeightPerRung     = 0.0;    // height per rung (m))

// daily Tide time table (0h to 23h)
float hourlyWaterLevels[24];
bool hourlyLevelsValid = false;

// --- functions Declarations ---
void setDualPumpsSpeed(float speed);
void stopDualPumps();
void processCalibrationCommand(String command);
void startEmptyingProcess();
void stopEmptyingProcess();
void finishCalibration();
void saveRelativeRung(int rungIndex, long steps);
void loadCalibrationFromFlash();
void savePositionsToRTC();
void restorePositionsFromRTC();
void getTideExtremaAndComputeScale();
void getWaterLevels();
void printLocalTime();
void calculateScaleFromTides(float highTideHeight, float lowTideHeight, int coefficient);
float convertHeightToRungIndex(float height);
long convertFractionalRungToAbsoluteSteps(float fractionalRung);
long convertRungToAbsoluteSteps(int targetRung);
void moveToRung(int targetRung);
void updateCurrentTideLevel();
void handleLedBlink();

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Steppers Initialisation
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Activate drivers

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  preferences.begin("tideClock", false);
  wifiSsid = preferences.getString("ssid", "YOUR_SSID");
  wifiPassword = preferences.getString("password", "YOUR_PASSWORD");

  loadCalibrationFromFlash();

  // get steppers position from RAM RTC
  // if magic number is invalid, restorePositionsFromRTC will launch a full tank drain
  restorePositionsFromRTC();

  
// 1. Configure host name for Box / DHCP
  WiFi.setHostname("tide");

  // 2. Connection to Wi-Fi
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  long startTime = millis();
  while ((WiFi.status() != WL_CONNECTED) && (millis() - startTime < 15000)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected ! IP : " + WiFi.localIP().toString());

  // 3. DNS local mDNS (http://tide.local)
  if (MDNS.begin("tide")) {
    // broadcast web service on port port 80 (needed for discovery Bonjour / Windows)
    MDNS.addService("http", "tcp", 80); 
    Serial.println("mDNS server started : http://tide.local");
  } else {
    Serial.println("Error while starting mDNS");
  }

  // 4. NetBIOS for Windows (http://tide)
  NBNS.begin("tide");

  // web server start
  setupWebServer();




  secureClient.setInsecure();
  secureClient.setTimeout(10000);

  // NTP time (including summer/winter time)
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
  delay(2000);
  printLocalTime();

  getTideExtremaAndComputeScale(); // call Api maree to get today's extrema
  delay(2000);
  getWaterLevels();                //call Api maree to get today's tide time table

  struct tm timeInfo;
  if (getLocalTime(&timeInfo)) {
    lastFetchedDay = timeInfo.tm_mday;
    lastFetchedHour = timeInfo.tm_hour;
  }
}

void loop() {
  handleWebServer();

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
      if (abs(stepper1.currentPosition() - drainStartPos) >= drainTargetSteps) {
        stopDualPumps();
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        
        savePositionsToRTC();
        digitalWrite(LED_PIN, LOW);

        currentState = STATE_NORMAL;
        Serial.println("[SYSTEM] emergency draining finished. Homing/Zero calibrated -> Switch to NORMAL MODE.");
        updateCurrentTideLevel();
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
      stopDualPumps();
      break;
  }

  // save position to RAM RTC
  if (currentState == STATE_NORMAL || currentState == STATE_CALIBRATING) {
    savePositionsToRTC();
  }

  // update tide if in NORMAL MODE
  if (currentState == STATE_NORMAL) {
    struct tm timeInfo;
    if (getLocalTime(&timeInfo)) {
      if (timeInfo.tm_mday != lastFetchedDay && timeInfo.tm_hour == 0) {
        Serial.println("\n[NTP] Midnight : Api maree --> update tides.");
        getTideExtremaAndComputeScale();
        delay(2000);
        getWaterLevels();
        lastFetchedDay = timeInfo.tm_mday;
        lastFetchedHour = timeInfo.tm_hour;
        updateCurrentTideLevel();
      }
      else if (timeInfo.tm_hour != lastFetchedHour) {
        lastFetchedHour = timeInfo.tm_hour;
        updateCurrentTideLevel();
      }
    }
  }
}

void handleLedBlink() {
  if (millis() - lastLedToggle >= 250) {
    lastLedToggle = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// -------------------------------------------------------------------------
// RAM RTC
// -------------------------------------------------------------------------
void savePositionsToRTC() {
  rtcStepper1Pos = stepper1.currentPosition();
  rtcStepper2Pos = stepper2.currentPosition();
  rtcMagicNumber = RTC_MAGIC_KEY;
}

void restorePositionsFromRTC() {
  if (rtcMagicNumber == RTC_MAGIC_KEY) {
    stepper1.setCurrentPosition(rtcStepper1Pos);
    stepper2.setCurrentPosition(rtcStepper2Pos);

    Serial.println("==========================================");
    Serial.println("[RTC] Positions restored after reset :");
    Serial.printf("[RTC] stepper 1 : %ld steps | stepper 2 : %ld steps\n", rtcStepper1Pos, rtcStepper2Pos);
    Serial.println("==========================================");
  } else {
    Serial.println("==========================================");
    Serial.println("[RTC] Magic Number invalid / Homing lost !");
    Serial.println("[RTC] Launch emergency draining.");
    Serial.println("==========================================");
    
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    rtcMagicNumber = 0;

    startEmptyingProcess();
  }
}

// -------------------------------------------------------------------------
// Moteurs & Conversions
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
  if (fractionalRung <= 0.0f) return 0;
  if (fractionalRung >= (float)TOTAL_RUNGS) return convertRungToAbsoluteSteps(TOTAL_RUNGS);

  int baseRung = (int)floor(fractionalRung);
  float fraction = fractionalRung - (float)baseRung;

  long baseSteps = convertRungToAbsoluteSteps(baseRung);
  long currentRungSteps = rungRelativeSteps[baseRung];

  return baseSteps + round(fraction * (float)currentRungSteps);
}

void updateCurrentTideLevel() {
  struct tm timeInfo;
  if (!getLocalTime(&timeInfo) || !hourlyLevelsValid) return;

  int currentHour = timeInfo.tm_hour;
  int nextHour = (currentHour + 1) % 24;

  float nextHeight = hourlyWaterLevels[nextHour];
  float nextFractionalRung = convertHeightToRungIndex(nextHeight);
  long targetSteps = convertFractionalRungToAbsoluteSteps(nextFractionalRung);

  long currentPosition = stepper1.currentPosition();
  long deltaSteps = targetSteps - currentPosition;

  const float TARGET_DURATION_SECONDS = 30.0f;
  float hourlySpeed = (float)deltaSteps / TARGET_DURATION_SECONDS;

  Serial.printf("\n[NORMAL MODE] %02dh00 -> %02dh00 | Target (H+1) : %.3f m (Rung %.2f)\n", 
                currentHour, nextHour, nextHeight, nextFractionalRung);
  Serial.printf("Position : %ld steps | Target : %ld steps | Speed : %.4f steps/sec\n", 
                currentPosition, targetSteps, hourlySpeed);

  stepper1.moveTo(targetSteps);
  stepper2.moveTo(targetSteps);

  stepper1.setMaxSpeed(fabs(hourlySpeed));
  stepper2.setMaxSpeed(fabs(hourlySpeed));

  setDualPumpsSpeed(hourlySpeed);
}

void setDualPumpsSpeed(float speed) {
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
  currentState = STATE_DRAINING_ZERO;
  drainStartPos = stepper1.currentPosition();
  
  // Total steps for max volume (Coeff 120  Rung)
  drainTargetSteps = convertRungToAbsoluteSteps(maxTargetRungCoef120 );
  if (drainTargetSteps <= 0) drainTargetSteps = 24000; // Sécurity calibration empty

  setDualPumpsSpeed(-500);
  Serial.printf("\n[HOMING] draining during %ld steps.\n", drainTargetSteps);
}

void stopEmptyingProcess() {
  stopDualPumps();
  digitalWrite(LED_PIN, LOW);
  currentState = STATE_IDLE;
  Serial.println("[HOMING] Draining interrupted via Web command. Switching to IDLE..");
}

void processCalibrationCommand(String command) {
  command.trim();

  // 1. security draining
  if (command == "CALIB:START" || command == "SYS:HOME") {
    startEmptyingProcess();
  }

  // explicit web stop of calibration
  else if (command == "SYS:STOP_DRAIN") {
    if (currentState == STATE_DRAINING_ZERO) {
      stopEmptyingProcess();
    }
  }
  
  // 2. Homing Validation  -> switch to IDLE
  else if (command == "NO_WATER" || command == "EMPTY_OK") {
    if (currentState == STATE_DRAINING_ZERO) {
      stopDualPumps();
      digitalWrite(LED_PIN, LOW);

      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      
      savePositionsToRTC();

      currentState = STATE_IDLE;
      Serial.println("[SYSTEM] Physical zero confirmed. System IDLE..");
    }
  }

  // 3. switch to Normal Mode (only from IDLE)
  else if (command == "SYS:START_NORMAL") {
    if (currentState == STATE_IDLE) {
      stopDualPumps();
      currentState = STATE_NORMAL;
      Serial.println("[SYSTEM] Switch to Normal MODE.");
      updateCurrentTideLevel();
    }
  }

  // 4. launch step by step calibration  (only from IDLE)
  else if (command == "CALIB:BEGIN_STEPS") {
    if (currentState == STATE_IDLE) {
      stopDualPumps();
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      currentCalibrationRung = 0;
      currentState = STATE_CALIBRATING;
      Serial.printf("[CALIB] Calibration started. Target high tide Coef 120 : rung %d/%d\n", 
                    maxTargetRungCoef120, TOTAL_RUNGS);
      
      setDualPumpsSpeed(400);
    }
  }

  // 5. manual control of pumps 
  else if (command == "PUMP:UP") {
    if (currentState == STATE_CALIBRATING) setDualPumpsSpeed(400);
  }
  else if (command == "PUMP:DOWN") {
    if (currentState == STATE_CALIBRATING) setDualPumpsSpeed(-400);
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

    if (currentCalibrationRung < maxTargetRungCoef120) {
      currentCalibrationRung++;
      saveRelativeRung(currentCalibrationRung, stepsMade);

      long totalAccumulatedSteps = convertRungToAbsoluteSteps(currentCalibrationRung);

      Serial.println("==========================================");
      Serial.printf("[CALIB] Rung validated : %d / %d (Target Coef 120 : %d)\n", 
                    currentCalibrationRung, TOTAL_RUNGS, maxTargetRungCoef120);
      Serial.printf("[CALIB] Steps measured for rung %d : %ld steps\n", currentCalibrationRung, stepsMade);
      Serial.printf("[CALIB] total steps recorded : %ld step\n", totalAccumulatedSteps);
      Serial.println("==========================================");

      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);

      if (currentCalibrationRung >= maxTargetRungCoef120) {
        finishCalibration();
      } else {
        setDualPumpsSpeed(400);
      }
    }
  }

  // 7. Stop calibration
  else if (command == "CALIB:END") {
    if (currentState == STATE_CALIBRATING) {
      stopDualPumps();
      currentState = STATE_IDLE;
      Serial.println("==========================================");
      Serial.printf("[CALIB] Calibration stopped at rung %d/%d. Go back to IDLE.\n", 
                    currentCalibrationRung, maxTargetRungCoef120);
      Serial.println("==========================================");
    }
  }
}

void finishCalibration() {
  long lastValidStep = (currentCalibrationRung > 0) ? rungRelativeSteps[currentCalibrationRung - 1] : 1000;

  for (int i = currentCalibrationRung; i < TOTAL_RUNGS; i++) {
    saveRelativeRung(i + 1, lastValidStep);
  }

  long totalStepsCurrent = convertRungToAbsoluteSteps(currentCalibrationRung);
  stepper1.setCurrentPosition(totalStepsCurrent);
  stepper2.setCurrentPosition(totalStepsCurrent);

  savePositionsToRTC();

  currentState = STATE_IDLE;

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
    rungRelativeSteps[i - 1] = preferences.getLong(key.c_str(), 1000);
  }
  Serial.println("[SYSTEM] Calibration loaded from Flash.");
}

// -------------------------------------------------------------------------
// API & Network
// -------------------------------------------------------------------------
void calculateScaleFromTides(float highTideHeight, float lowTideHeight, int coefficient) {
  if (coefficient <= 0) return;

  // 1. compute today's mid tide (rung 10)
  meanWaterLevelHeight = (highTideHeight + lowTideHeight) / 2.0f;

  // 2. mid tide extrapolated to coefficient 120
  float observedHalfRange = (highTideHeight - lowTideHeight) / 2.0f;
  float halfRange120      = observedHalfRange * (120.0f / (float)coefficient);

  // 3. half tide for coeff 120 is now  reaching exactly 10 rungs (from zero (land level) to 10)
  stepHeightPerRung = halfRange120 / (float)MEAN_LEVEL_RUNG_INDEX;

  // 4. high tide Coef 120 theoritical (should occur at rung  10 + 10 = 20)
  float maxWaterHeight120 = meanWaterLevelHeight + halfRange120;
  maxTargetRungCoef120 = (int)ceil(convertHeightToRungIndex(maxWaterHeight120)); // rung 20

  Serial.printf("[SCALE] Half tide (rung 10) : %.2f m | steps/rung : %.3f m | Low tide Coef 120 (Rung 0) : %.2f m\n", 
                meanWaterLevelHeight, stepHeightPerRung, meanWaterLevelHeight - halfRange120);
}

float convertHeightToRungIndex(float height) {
  float heightDifference = height - meanWaterLevelHeight;
  
  // linear projectiopn around rung 10
  float fractionalRung = (float)MEAN_LEVEL_RUNG_INDEX + (heightDifference / stepHeightPerRung);

  if (fractionalRung < 0.0f) return 0.0f;
  if (fractionalRung > (float)TOTAL_RUNGS) return (float)TOTAL_RUNGS;

  return fractionalRung;
}

void getTideExtremaAndComputeScale() {
  if (WiFi.status() != WL_CONNECTED) return;

  struct tm timeInfo;
  if (!getLocalTime(&timeInfo)) return;

  char dateStr[11];
  strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &timeInfo);

  String requestUrl = "https://api-maree.fr/tide-extrema?";
  requestUrl += "site=" + String(siteId);
  requestUrl += "&from=" + String(dateStr);
  requestUrl += "&to=" + String(dateStr);
  requestUrl += "&tz=Europe/Paris";
  requestUrl += "&key=" + String(apiKey);

  Serial.println(requestUrl);
  //https://api-maree.fr/tide-extrema?site=boucau-bayonne-biarritz&from=2026-09-22&to=2026-09-22&tz=Europe/Paris&key=YOUR_API


  for (int attempt = 1; attempt <= 3; attempt++) {
    HTTPClient http;
    http.setTimeout(15000);

    if (http.begin(secureClient, requestUrl)) {
      http.setUserAgent("ESP32-TideClock");
      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK) {
        DynamicJsonDocument jsonDoc(4096);
        WiFiClient* stream = http.getStreamPtr();
        DeserializationError error = deserializeJson(jsonDoc, *stream);

        if (!error && jsonDoc.containsKey("data")) {
          JsonArray dataArray = jsonDoc["data"].as<JsonArray>();
          if (dataArray.size() > 0 && dataArray[0].containsKey("extrema")) {
            JsonArray extremaArray = dataArray[0]["extrema"].as<JsonArray>();

            float lowestTideHeight = 99.0;
            float highestTideHeight = -99.0;
            int currentCoefficient = 0;

            for (JsonObject item : extremaArray) {
              float h = item["height"].as<float>();
              if (h < lowestTideHeight) lowestTideHeight = h;
              if (h > highestTideHeight) highestTideHeight = h;

              if (item.containsKey("coef") && !item["coef"].isNull()) {
                int c = item["coef"].as<int>();
                if (c > 0) currentCoefficient = c;
              }
            }

            calculateScaleFromTides(highestTideHeight, lowestTideHeight, currentCoefficient);
            http.end();
            secureClient.stop();
            return;
          }
        }
      }
      http.end();
    }
    secureClient.stop();
    delay(1000);
  }
}

void getWaterLevels() {
  if (WiFi.status() != WL_CONNECTED) return;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;

  char dateFrom[17];
  strftime(dateFrom, sizeof(dateFrom), "%Y-%m-%dT00:00", &timeinfo);

  struct tm tomorrowInfo = timeinfo;
  tomorrowInfo.tm_mday += 1;
  mktime(&tomorrowInfo);

  char dateTo[17];
  strftime(dateTo, sizeof(dateTo), "%Y-%m-%dT00:00", &tomorrowInfo);

  String url = "https://api-maree.fr/water-levels?";
  url += "site=" + String(siteId);
  url += "&from=" + String(dateFrom);
  url += "&to=" + String(dateTo);
  url += "&step=60";
  url += "&tz=Europe/Paris";
  url += "&key=" + String(apiKey);
  Serial.println(url);
  //https://api-maree.fr/water-levels?site=boucau-bayonne-biarritz&from=2026-09-22T00:00&to=2026-09-23T00:00&step=60&tz=Europe/Paris&key=YOUR_API


  for (int attempt = 1; attempt <= 3; attempt++) {
    HTTPClient http;
    http.setTimeout(15000);

    if (http.begin(secureClient, url)) {
      http.setUserAgent("ESP32-TideClock");
      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK) {
        DynamicJsonDocument doc(8192);
        WiFiClient* stream = http.getStreamPtr();
        DeserializationError error = deserializeJson(doc, *stream);

        if (!error && doc.containsKey("data")) {
          JsonArray dataArray = doc["data"].as<JsonArray>();

          for (int i = 0; i < 24; i++) hourlyWaterLevels[i] = 0.0;

          for (JsonObject item : dataArray) {
            String dtStr = item.containsKey("datetime") ? item["datetime"].as<String>() : item["time"].as<String>();

            int tIndex = dtStr.indexOf('T');
            if (tIndex != -1 && dtStr.length() >= tIndex + 3) {
              int hour = dtStr.substring(tIndex + 1, tIndex + 3).toInt();
              if (hour >= 0 && hour < 24) {
                float height = item["height"].as<float>();
                hourlyWaterLevels[hour] = height;

                float rungIndex = convertHeightToRungIndex(height);
                Serial.printf("Time : %s | Height : %.3f m --> Rung Index: %.2f\n", 
                              dtStr.c_str(), height, rungIndex);
              }
            }
          }

          hourlyLevelsValid = true;
          Serial.println("[API] Today's tide time table recorded successfuly");

          http.end();
          secureClient.stop();
          return;
        }
      }
      http.end();
    }
    secureClient.stop();
    delay(1000);
  }
}

void printLocalTime() {
  struct tm timeInfo;
  if (!getLocalTime(&timeInfo)) {
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
