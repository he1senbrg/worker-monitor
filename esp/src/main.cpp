#include "FallDetection.h"
#include "MAX30105.h"
#include "RockFall.h"
#include "heartRate.h"
#include "secrets.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Arduino_JSON.h>
#include <DHT.h>
#include <DHT_U.h>
#include <HTTPClient.h>
#include <MPU6500_WE.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <Wire.h>

#define BUZZER_PIN 14

RockfallDetector rockfall;
unsigned long lastRockfallCheck = 0;
const unsigned long ROCKFALL_CHECK_INTERVAL = 1000;

FallDetector detector;

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *websocket_server = SERVER_IP;
const int websocket_port = SERVER_PORT;
const char *websocket_path = "/";

WebSocketsClient webSocket;

JSONVar readings;

#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

Adafruit_BMP280 bmp;

MAX30105 particleSensor;

unsigned long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 70;
const byte RATE_ARRAY_SIZE = 4;
long rateArray[RATE_ARRAY_SIZE] = {0};
long rateArrayIndex = 0;
bool fingerDetected = false;


#define DHTPIN 16
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);


unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 10;

const char *ntfy_topic_url = "https://ntfy.vishnu.studio/sector";

void sendNotification(String id, String name, String event, String message) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(ntfy_topic_url);
    http.addHeader("Content-Type", "text/plain");
    http.addHeader("Title", event);
    http.addHeader("Priority", "3");

    const String payload = "[" + id + "] " + name + ": " + message;

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      Serial.printf("Notification sent! Response code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Failed to send notification. Error code: %d\n", httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi not connected - Cannot send notification");
  }
}


void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    Serial.println("WebSocket Disconnected");
    break;

  case WStype_CONNECTED:
    Serial.printf("WebSocket Connected to: %s\n", payload);
    break;

  case WStype_TEXT:
    // Serial.printf("Received: %s\n", payload);
    break;

  case WStype_ERROR:
    Serial.println("WebSocket Error");
    break;

  default:
    break;
  }
}


void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());
}


void calculateHeartRate(long irValue) {
  if (checkForBeat(irValue)) {
    unsigned long currentTime = millis();
    long delta = currentTime - lastBeat;
    lastBeat = currentTime;

    
    if (delta > 300 && delta < 2000) {
      beatsPerMinute = 60000.0 / delta; 

      
      rateArray[rateArrayIndex++] = (long)beatsPerMinute;
      rateArrayIndex %= RATE_ARRAY_SIZE;

      
      long total = 0;
      for (byte i = 0; i < RATE_ARRAY_SIZE; i++) {
        total += rateArray[i];
      }
      beatAvg = total / RATE_ARRAY_SIZE;

      beatAvg = constrain(beatAvg, 40, 200);
    }
  }
}

String getAllSensorReadings() {
  readings = JSONVar();

  readings["timestamp"] = millis();

  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float mpuTemp = myMPU6500.getTemperature();

  float resultantG =
      sqrt(gValue.x * gValue.x + gValue.y * gValue.y + gValue.z * gValue.z);

  readings["accX"] = String(gValue.x, 3);
  readings["accY"] = String(gValue.y, 3);
  readings["accZ"] = String(gValue.z, 3);
  readings["resultantG"] = String(resultantG, 3);
  readings["gyroX"] = String(gyr.x, 2);
  readings["gyroY"] = String(gyr.y, 2);
  readings["gyroZ"] = String(gyr.z, 2);
  readings["mpuTemp"] = String(mpuTemp, 1);

  if (bmp.takeForcedMeasurement()) {
    float bmpTemperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude(1013.25);

    readings["bmpTemp"] = String(bmpTemperature, 1);
    readings["pressure"] = String(pressure / 100.0, 1);
    readings["altitude"] = String(altitude, 1);
  } else {
    readings["bmpTemp"] = "Error";
    readings["pressure"] = "Error";
    readings["altitude"] = "Error";
  }

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    readings["dhtTemp"] = String(event.temperature, 1);
  } else {
    readings["dhtTemp"] = "Error";
  }

  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    readings["humidity"] = String(event.relative_humidity, 1);

    sensors_event_t tempEvent;
    dht.temperature().getEvent(&tempEvent);
    if (!isnan(tempEvent.temperature)) {
      float tempF = tempEvent.temperature * 9.0 / 5.0 + 32.0;
      float humidity = event.relative_humidity;

      if (tempF >= 80.0 && humidity >= 40.0) {
        float heatIndexF =
            -42.379 + 2.04901523 * tempF + 10.14333127 * humidity -
            0.22475541 * tempF * humidity - 6.83783e-3 * tempF * tempF -
            5.481717e-2 * humidity * humidity +
            1.22874e-3 * tempF * tempF * humidity +
            8.5282e-4 * tempF * humidity * humidity -
            1.99e-6 * tempF * tempF * humidity * humidity;
        float heatIndexC = (heatIndexF - 32.0) * 5.0 / 9.0;
        readings["heatIndex"] = String(heatIndexC, 1);
      } else {
        readings["heatIndex"] = "N/A";
      }
    }
  } else {
    readings["humidity"] = "Error";
    readings["heatIndex"] = "Error";
  }

  long irValue = particleSensor.getIR();
  readings["irValue"] = String(irValue);

  fingerDetected = (irValue > 7000);

  if (fingerDetected) {
    readings["heartRate"] = String(beatsPerMinute, 1);
    readings["avgHeartRate"] = String(beatAvg);
    readings["fingerDetected"] = "true";
  } else {
    readings["heartRate"] = "0";
    readings["avgHeartRate"] = "0";
    readings["fingerDetected"] = "false";
  }

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  initWiFi();

  Serial.print("MPU6500 (IMU)...");
  if (!myMPU6500.init()) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    Serial.println("Calibrating MPU6500...");
    delay(1000);
    myMPU6500.autoOffsets();
    Serial.println("MPU6500 calibration complete!");

    myMPU6500.enableGyrDLPF();
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.setSampleRateDivider(5);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  }

  Serial.print("BMP280 (Pressure)...");
  if (!bmp.begin(BMP280_ADDRESS_ALT)) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    bmp.setSampling(
      Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_500
    );
  }

  Serial.print("MAX30105 (Heart Rate)...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x50);
    particleSensor.setPulseAmplitudeIR(0x50);
    particleSensor.setLEDMode(2);
  }

  Serial.print("DHT22 (Temp/Humidity)...");
  dht.begin();
  Serial.println("OK");

  Serial.print("WebSocket connection...");
  webSocket.begin(websocket_server, websocket_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  Serial.println("Connecting...");

  Serial.println("ALL SENSORS INITIALIZED!");

  for (int i = 0; i < RATE_ARRAY_SIZE; i++) {
    rateArray[i] = 70;
  }

  delay(500);
}

void loop() {
  webSocket.loop();

  long irValue = particleSensor.getIR();

  fingerDetected = (irValue > 7000);

  if (fingerDetected) {
    calculateHeartRate(irValue);
  } else {
    for (int i = 0; i < RATE_ARRAY_SIZE; i++) {
      rateArray[i] = 70;
    }
    beatsPerMinute = 0;
  }

  if (millis() - lastRockfallCheck >= ROCKFALL_CHECK_INTERVAL) {
    lastRockfallCheck = millis();
    float pressure = bmp.readPressure();
    xyzFloat acc = myMPU6500.getGValues();
    xyzFloat gyro = myMPU6500.getGyrValues();
    float resultantG = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

    rockfall.update(pressure, resultantG, gyro);

    if (rockfall.isRockfallDetected()) {
      Serial.println("ROCKFALL DETECTED!");
      sendNotification(WORKER_ID, WORKER_NAME, "ROCKFALL DETECTED", "Rockfall detected in cave!");

      for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(150);
      }
    }
  }

  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();

    xyzFloat acc = myMPU6500.getGValues();
    xyzFloat gyro = myMPU6500.getGyrValues();
    float resultantG = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

    detector.update(resultantG, gyro, beatAvg);

    if (detector.isFallDetected()) {
      Serial.println("FALL DETECTED!");
      if (detector.isHeartRateCritical()) {
        sendNotification(WORKER_ID, WORKER_NAME, "FALL DETECTED", "FALL detected, Heart Rate CRITICAL");
      } else {
        sendNotification(WORKER_ID, WORKER_NAME, "FALL DETECTED", "FALL detected, Heart Rate stable.");
      }

      for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(300);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
      }
    }

    String sensorData = getAllSensorReadings();
    webSocket.sendTXT(sensorData);

    if (webSocket.isConnected()) {
      // Serial.println("Data sent to server");
    } else {
      Serial.println("WebSocket disconnected - attempting reconnect");
    }
  }
}
