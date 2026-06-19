#include "secrets.h"

#define BLYNK_TEMPLATE_ID   "TMPL2nNr2zitr"
#define BLYNK_TEMPLATE_NAME "Automated Planter"
#define BLYNK_AUTH_TOKEN    BLYNK_AUTH_TOKEN_SECRET

#define BLYNK_PRINT Serial

// Libraries
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "Adafruit_seesaw.h"
#include <TimeLib.h>
#include <WidgetRTC.h>

//Wifi credentials
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = WIFI_SSID_SECRET;
char pass[] = WIFI_PASS_SECRET;

BlynkTimer timer;
Adafruit_seesaw ss; // Soil moisture sensor
WidgetRTC rtc; // For Blynk to run off the real time clock

const int LED1_PIN = 12; // Input 1 for PicoBuck LED driver
const int LED2_PIN = 14; // Input 2 for PicoBuck LED driver 
const int PUMP_PIN = 26; // --> 1k ohm resistor --> base of TIP41C to control pump

bool soilSensorOK = false; // Initial state for moisture sensor 

// Light schedule
int lightStartSec = 7 * 3600; // Default start at 7 am
int lightStopSec  = 21 * 3600; // Default end at 9 pm
const int RAMP_SECS = 2 * 3600; // Ramp up/down (sunrise/set) over 2 hours
int lightTimerID; // Stored so timer interval can be changed dynamically
int max_bright = 255; // Peak PWM value, so system starts with max brightness

// Moisture averaging - rolling averages of 30 samples at 1s each, 5 min avg
const int AVG_SAMPLES = 300;
float moistureSum = 0;
float tempSum = 0;
int sampleCount = 0;

// --- Pump auto-watering config ---
int MOISTURE_LOW  = 620;  // turn on pump below this
int MOISTURE_OK   = 680;  // stop watering above this
float avgMoisture = 660; //initialize to mid-range value, won't get recorded on blynk
const unsigned long PUMP_ON_MS   = 15000;  // 15 seconds for pump to be on
const unsigned long PUMP_WAIT_MS = 1200000; // 20 minutes before recheck for more water

// Pump state machine for pump automation
enum PumpState { IDLE, PUMPING, WAITING };
PumpState pumpState = IDLE;
unsigned long pumpStateStart = 0;
bool manualPump = false; // manual pump switch override, true when on

unsigned long lastWateredEpoch = 0;  // Unix timestamp of last pump activation

// Sync to real time for light schedule
BLYNK_CONNECTED() {
  rtc.begin();
}

// Light schedule datastream
BLYNK_WRITE(V0) {
  lightStartSec = param[0].asInt();
  lightStopSec  = param[1].asInt();
  // Serial feedback when updated 
  Serial.print("Light window updated: ");
  Serial.print(lightStartSec / 3600); Serial.print("h to ");
  Serial.print(lightStopSec / 3600);  Serial.println("h");
}

// Manual pump switch datastream
BLYNK_WRITE(V2) {
  manualPump = param.asInt();
  digitalWrite(PUMP_PIN, manualPump);
  if (manualPump) {
    char buf[20];
    sprintf(buf, "%02d:%02d %02d/%02d", hour(), minute(), month(), day());
    Blynk.virtualWrite(V8, buf);
  }
  Serial.print("Manual pump: "); Serial.println(manualPump ? "ON" : "OFF");
}
// Slider for maximum brightness
BLYNK_WRITE(V5) {
  max_bright = param.asInt();
}

// Slider for the low moisture reading
BLYNK_WRITE(V6) {
  MOISTURE_LOW = param.asInt();
  Serial.print("Low moisture updated: "); Serial.print(MOISTURE_LOW);
}

// Slider for the okay moisture reading
BLYNK_WRITE(V7) {
  MOISTURE_OK = param.asInt();
  Serial.print("Okay moisture updated: "); Serial.print(MOISTURE_OK);
}

// Function to update the lights
void updateLights() {
  int nowSec   = hour() * 3600 + minute() * 60 + second();
  int duration = lightStopSec - lightStartSec;
  int elapsed  = nowSec - lightStartSec;
  int brightness = 0;

  if (elapsed > 0 && elapsed < duration) {
    if (elapsed < RAMP_SECS) {
      brightness = (int)(max_bright * 1.0 * elapsed / RAMP_SECS); // brightness ramp up
    } else if (elapsed < duration - RAMP_SECS) {
      brightness = max_bright; // maximum brightness during "day"
    } else {
      int timeLeft = duration - elapsed;
      brightness = (int)(max_bright * 1.0 * timeLeft / RAMP_SECS); // brightness ramp down
    }
  }

  analogWrite(LED1_PIN, brightness);
  analogWrite(LED2_PIN, brightness);
  Blynk.virtualWrite(V1, (brightness / 255.0f) * 100.0f); // datastream v1 set to brightness as a %

 // Run timer more frequently during ramps, less often at steady state
  bool isRamping = (elapsed > 0) && 
                   (elapsed < RAMP_SECS || elapsed > duration - RAMP_SECS) && 
                   (elapsed < duration);

  timer.changeInterval(lightTimerID, isRamping ? 60000L : 600000L);
}

// Pump automated control
void handlePump(float currentMoisture) {
  if (manualPump) return; // manual switch overides

  unsigned long currentMillis = millis();

  switch (pumpState) {
    case IDLE:
      if (currentMoisture < MOISTURE_LOW) {
        Serial.println("Moisture low, starting pump");
        digitalWrite(PUMP_PIN, HIGH);
        pumpState      = PUMPING;
        pumpStateStart = currentMillis;
      }
      break;

    case PUMPING:
      if (currentMillis - pumpStateStart >= PUMP_ON_MS) { //pumps for 15s
        Serial.println("Pump cycle done, waiting to recheck");
        digitalWrite(PUMP_PIN, LOW);
        pumpState = WAITING; //set to wait state
        pumpStateStart = currentMillis;
        char buf[20];
        sprintf(buf, "%02d:%02d %02d/%02d", hour(), minute(), month(), day());
        Blynk.virtualWrite(V8, buf);
      }
      break;

    case WAITING:
      if (currentMillis - pumpStateStart >= PUMP_WAIT_MS) {
        if (currentMoisture < MOISTURE_OK) { // if soil is not saturated pump again
          Serial.println("Still dry, pumping again");
          digitalWrite(PUMP_PIN, HIGH);
          pumpState = PUMPING;
          pumpStateStart = currentMillis;
        } else { //otherwise set to idle and wait until the soil is dry again
          Serial.println("Moisture OK, returning to idle");
          pumpState = IDLE;
        }
      }
      break;
  }
}

// soil moisture sensor averaging
void sampleSoil() {
  if (!soilSensorOK) {
    soilSensorOK = ss.begin(0x36);
    return;
  }

  // Accumulate samples
  moistureSum += ss.touchRead(0);
  tempSum     += ss.getTemp();
  sampleCount++;
  if (sampleCount % 300 == 0) {
    Serial.print("Sample "); Serial.print(sampleCount);
    Serial.print("/"); Serial.println(AVG_SAMPLES);
  }
  // Once we have enough samples, push the average
  if (sampleCount >= AVG_SAMPLES) {
    avgMoisture = moistureSum / sampleCount;
    float avgTemp     = tempSum     / sampleCount;

    Blynk.virtualWrite(V3, avgMoisture); // datastream to display current average
    //Blynk.virtualWrite(V8, avgTemp); //temp datastream, sensor is not good enough to make it worth it

    Serial.print("Avg moisture: "); Serial.println(avgMoisture);
    //Serial.print("Avg temp: ");     Serial.print(avgTemp); Serial.println("*C");

    moistureSum = 0;
    tempSum     = 0;
    sampleCount = 0;
  }
}

//  CONNECTION WATCHDOG
unsigned long lastConnectedTime = 0;
const unsigned long RECONNECT_TIMEOUT_MS = 10UL * 60 * 1000;  // 10 minutes

void checkConnection() {
  if (Blynk.connected()) {
    lastConnectedTime = millis();
  } else if (millis() - lastConnectedTime > RECONNECT_TIMEOUT_MS) {
    Serial.println("Lost connection too long, restarting...");
    ESP.restart();
  }
}

// Setup
void setup() {
  Serial.begin(115200);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  analogWrite(LED1_PIN, 0);
  analogWrite(LED2_PIN, 0);
  digitalWrite(PUMP_PIN, LOW);

  if (!ss.begin(0x36)) {
    Serial.println("Soil sensor not found, will retry");
  } else {
    Serial.println("Soil sensor OK");
    soilSensorOK = true;
  }

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(1000L, sampleSoil);   // sample every .1s, pushes when # for AVG_SAMPLES is met
  lightTimerID = timer.setInterval(60000L, updateLights); // updates the lights every 10 minutes by default

  lastConnectedTime = millis();  // initialize so it doesn't immediately trigger
  timer.setInterval(300000L, checkConnection);  // check every 5 min
}

void loop() {
  Blynk.run(); // Handle Blynk communication
  timer.run(); // Fire scheduled callbacks
  handlePump(avgMoisture); // Makes sure that the pump doesn't get stuck on and all timers work properly
}