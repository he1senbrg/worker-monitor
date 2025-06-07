#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MPU6500_WE.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Arduino_JSON.h>

// ========== WIFI CREDENTIALS ==========
const char *ssid = "ssid";
const char *password = "password";

// ========== WEBSOCKET SERVER CONFIG ==========
const char* websocket_server = "ip"; // Replace with your PC's IP
const int websocket_port = 8765;
const char* websocket_path = "/";

// Create WebSocket client
WebSocketsClient webSocket;

// Json Variable to Hold Sensor Readings
JSONVar readings;

// ========== KALMAN FILTER STRUCTURE ==========
struct KalmanFilter {
  float Q;     // Process noise covariance
  float R;     // Measurement noise covariance
  float P;     // Estimation error covariance
  float K;     // Kalman gain
  float X;     // Estimated value
};

// Initialize Kalman filters for MPU6500 data
KalmanFilter kalman_accX = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_accY = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_accZ = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_gyroX = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_gyroY = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_gyroZ = {0.01, 0.1, 1, 0, 0};

// ========== SENSOR SETUP ==========
// MPU6500 setup
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

// BMP280 setup
Adafruit_BMP280 bmp; // I2C

// MAX30105 setup
MAX30105 particleSensor;

// Increase this for more averaging. 4 is good.
const byte RATE_SIZE = 4;

// Array of heart rates
byte rates[RATE_SIZE];
byte rateSpot = 0;

// Time at which the last beat occurred
long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

// DHT22 setup
#define DHTPIN 16
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);

// ========== TIMING VARIABLES ==========
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 10; // Send data every 100ms

// ========== KALMAN FILTER FUNCTIONS ==========
float kalmanUpdate(KalmanFilter* kf, float measurement) {
  // Prediction step
  kf->P = kf->P + kf->Q;
  
  // Update step
  kf->K = kf->P / (kf->P + kf->R);
  kf->X = kf->X + kf->K * (measurement - kf->X);
  kf->P = (1 - kf->K) * kf->P;
  
  return kf->X;
}

// ========== WEBSOCKET FUNCTIONS ==========
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      break;
      
    case WStype_CONNECTED:
      Serial.printf("WebSocket Connected to: %s\n", payload);
      break;
      
    case WStype_TEXT:
      Serial.printf("Received: %s\n", payload);
      break;
      
    case WStype_ERROR:
      Serial.println("WebSocket Error");
      break;
      
    default:
      break;
  }
}

// Initialize WiFi
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

String getAllSensorReadings() {
  // Clear previous readings
  readings = JSONVar();
  
  // Add timestamp
  readings["timestamp"] = millis();

  // ========== IMU DATA (MPU6500) WITH KALMAN FILTERING ==========
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float mpuTemp = myMPU6500.getTemperature();
  
  // Apply Kalman filtering
  float filteredAccX = kalmanUpdate(&kalman_accX, gValue.x);
  float filteredAccY = kalmanUpdate(&kalman_accY, gValue.y);
  float filteredAccZ = kalmanUpdate(&kalman_accZ, gValue.z);
  float filteredGyroX = kalmanUpdate(&kalman_gyroX, gyr.x);
  float filteredGyroY = kalmanUpdate(&kalman_gyroY, gyr.y);
  float filteredGyroZ = kalmanUpdate(&kalman_gyroZ, gyr.z);
  
  // Calculate filtered resultant G
  float resultantG = sqrt(filteredAccX*filteredAccX + filteredAccY*filteredAccY + filteredAccZ*filteredAccZ);

  readings["accX"] = String(filteredAccX, 3);
  readings["accY"] = String(filteredAccY, 3);
  readings["accZ"] = String(filteredAccZ, 3);
  readings["resultantG"] = String(resultantG, 3);
  readings["gyroX"] = String(filteredGyroX, 2);
  readings["gyroY"] = String(filteredGyroY, 2);
  readings["gyroZ"] = String(filteredGyroZ, 2);
  readings["mpuTemp"] = String(mpuTemp, 1);

  // ========== ENVIRONMENTAL DATA (BMP280) ==========
  if (bmp.takeForcedMeasurement()) {
    float bmpTemperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude(1013.25);

    readings["bmpTemp"] = String(bmpTemperature, 1);
    readings["pressure"] = String(pressure / 100.0, 1); // Convert Pa to hPa
    readings["altitude"] = String(altitude, 1);
  } else {
    readings["bmpTemp"] = "Error";
    readings["pressure"] = "Error";
    readings["altitude"] = "Error";
  }

  // ========== DHT22 ENVIRONMENTAL DATA ==========
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

    // Calculate heat index if both values are valid
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

  // ========== BIOMETRIC DATA (MAX30105) ==========
  long irValue = particleSensor.getIR();
  readings["irValue"] = String(irValue);

  if (irValue >= 50000) {
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

  Serial.println("====================================");
  Serial.println("  MULTI-SENSOR WEBSOCKET CLIENT    ");
  Serial.println("====================================");

  // Initialize WiFi
  initWiFi();

  // ========== INITIALIZE MPU6500 ==========
  Serial.print("MPU6500 (IMU)............ ");
  if (!myMPU6500.init()) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    Serial.println("Calibrating MPU6500...");
    delay(1000);
    myMPU6500.autoOffsets();
    Serial.println("MPU6500 calibration complete!");

    // Configure MPU6500
    myMPU6500.enableGyrDLPF();
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.setSampleRateDivider(5);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  }

  // ========== INITIALIZE BMP280 ==========
  Serial.print("BMP280 (Pressure)........ ");
  if (!bmp.begin(BMP280_ADDRESS_ALT)) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  // ========== INITIALIZE MAX30105 ==========
  Serial.print("MAX30105 (Heart Rate).... ");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    particleSensor.setup();
  }

  // ========== INITIALIZE DHT22 ==========
  Serial.print("DHT22 (Temp/Humidity)... ");
  dht.begin();
  Serial.println("OK");

  // ========== INITIALIZE WEBSOCKET ==========
  Serial.print("WebSocket connection..... ");
  webSocket.begin(websocket_server, websocket_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  Serial.println("Connecting...");

  Serial.println("====================================");
  Serial.println("   ALL SENSORS INITIALIZED!        ");
  Serial.println("====================================");

  delay(500);
}

void loop() {
  // Handle WebSocket events
  webSocket.loop();

  // ========== CONTINUOUS HEART RATE MONITORING ==========
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      // Calculate average
      long total = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        total += rates[x];
      beatAvg = total / RATE_SIZE;
    }
  }

  // ========== PERIODIC SENSOR READINGS AND WEBSOCKET TRANSMISSION ==========
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();

    // Get all sensor data and send via WebSocket
    String sensorData = getAllSensorReadings();
    webSocket.sendTXT(sensorData);
    
    // Optional: Print connection status
    if (webSocket.isConnected()) {
      Serial.println("Data sent to server");
    } else {
      Serial.println("WebSocket disconnected - attempting reconnect");
    }
  }
}