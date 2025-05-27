#include <Arduino.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <Adafruit_BMP280.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

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
const unsigned long SENSOR_INTERVAL = 500;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("====================================");
  Serial.println("    MULTI-SENSOR INITIALIZATION    ");
  Serial.println("====================================");

  // ========== INITIALIZE MPU6500 ==========
  Serial.print("MPU6500 (IMU)............ ");
  if(!myMPU6500.init()){
    Serial.println("FAILED");
    Serial.println("ERROR: MPU6500 does not respond!");
  }
  else{
    Serial.println("OK");

    Serial.println("Calibrating MPU6500 - Keep sensor flat and still...");
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
    Serial.println("ERROR: Could not find BMP280 sensor!");
  }
  else{
    Serial.println("OK");

    // Configure BMP280
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  // ========== INITIALIZE MAX30105 ==========
  Serial.print("MAX30105 (Heart Rate).... ");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("FAILED");
    Serial.println("ERROR: MAX30105 was not found!");
  }
  else{
    Serial.println("OK");

    particleSensor.setup();

    // particleSensor.setPulseAmplitudeRed(0x0A);
    // particleSensor.setPulseAmplitudeGreen(0);

    // particleSensor.setPulseAmplitudeRed(0x50);
    // particleSensor.setPulseAmplitudeIR(0x7F);
    // particleSensor.setPulseAmplitudeGreen(0);
  }

  // ========== INITIALIZE DHT22 ==========
  Serial.print("DHT22 (Temp/Humidity)... ");
  dht.begin();
  
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  if (sensor.name) {
    Serial.println("OK");
    Serial.print("DHT22 Temperature Range: ");
    Serial.print(sensor.min_value);
    Serial.print("°C to ");
    Serial.print(sensor.max_value);
    Serial.println("°C");
  } else {
    Serial.println("FAILED");
    Serial.println("ERROR: DHT22 sensor not responding!");
  }

  Serial.println("====================================");
  Serial.println("   ALL SENSORS INITIALIZED!        ");
  Serial.println("Place finger on MAX30105 sensor    ");
  Serial.println("====================================");
  Serial.println();

  delay(500);
}

void printSensorData(long irValue) {
  Serial.println("══════════════════════════════════════════════════════════════════════");
  Serial.println("                           SENSOR READINGS                           ");
  Serial.println("══════════════════════════════════════════════════════════════════════");

  // ========== IMU DATA (MPU6500) ==========
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float mpuTemp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);

  Serial.println(" 📐 ACCELEROMETER & GYROSCOPE (MPU6500)                              ");
  Serial.println("──────────────────────────────────────────────────────────────────────");
  Serial.print(" Acceleration (g): X=");
  Serial.print(gValue.x, 3);
  Serial.print(" | Y=");
  Serial.print(gValue.y, 3);
  Serial.print(" | Z=");
  Serial.print(gValue.z, 3);
  Serial.println("          ");
  Serial.print(" Resultant G-Force: ");
  Serial.print(resultantG, 3);
  Serial.println(" g                                    ");
  Serial.print(" Gyroscope (°/s):   X=");
  Serial.print(gyr.x, 2);
  Serial.print(" | Y=");
  Serial.print(gyr.y, 2);
  Serial.print(" | Z=");
  Serial.print(gyr.z, 2);
  Serial.println("        ");
  Serial.print(" IMU Temperature: ");
  Serial.print(mpuTemp, 1);
  Serial.println(" °C                                      ");

  // ========== ENVIRONMENTAL DATA (BMP280) ==========
  Serial.println("──────────────────────────────────────────────────────────────────────");
  Serial.println(" 🌡️  ENVIRONMENTAL SENSORS (BMP280)                                   ");
  Serial.println("──────────────────────────────────────────────────────────────────────");

  if (bmp.takeForcedMeasurement()) {
    float bmpTemperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude(1013.25);

    Serial.print(" BMP Temperature: ");
    Serial.print(bmpTemperature, 1);
    Serial.println(" °C                                      ");
    Serial.print(" Pressure: ");

    // Convert Pa to hPa
    Serial.print(pressure / 100.0, 1);
    Serial.println(" hPa                                        ");
    Serial.print(" Altitude: ");
    Serial.print(altitude, 1);
    Serial.println(" m                                            ");
  } else {
    Serial.println(" ⚠️  BMP280 measurement failed!                                        ");
  }

  // ========== DHT22 ENVIRONMENTAL DATA ==========
  Serial.println("──────────────────────────────────────────────────────────────────────");
  Serial.println(" 🌡️💧 TEMPERATURE & HUMIDITY (DHT22)                                  ");
  Serial.println("──────────────────────────────────────────────────────────────────────");

  // Get temperature event
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(" ⚠️  Error reading DHT22 temperature!                                 ");
  } else {
    Serial.print(" DHT Temperature: ");
    Serial.print(event.temperature, 1);
    Serial.println(" °C                                      ");
  }

  // Get humidity event
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(" ⚠️  Error reading DHT22 humidity!                                    ");
  } else {
    Serial.print(" Humidity: ");
    Serial.print(event.relative_humidity, 1);
    Serial.println(" %                                           ");
    
    // Calculate heat index if both temperature and humidity are valid
    if (!isnan(event.relative_humidity)) {
      sensors_event_t tempEvent;
      dht.temperature().getEvent(&tempEvent);
      if (!isnan(tempEvent.temperature)) {
        // Simple heat index calculation (Fahrenheit based, converted)
        float tempF = tempEvent.temperature * 9.0/5.0 + 32.0; // Convert to Fahrenheit
        float humidity = event.relative_humidity;
        
        if (tempF >= 80.0 && humidity >= 40.0) {
          float heatIndexF = -42.379 + 2.04901523*tempF + 10.14333127*humidity 
                           - 0.22475541*tempF*humidity - 6.83783e-3*tempF*tempF 
                           - 5.481717e-2*humidity*humidity + 1.22874e-3*tempF*tempF*humidity 
                           + 8.5282e-4*tempF*humidity*humidity - 1.99e-6*tempF*tempF*humidity*humidity;
          float heatIndexC = (heatIndexF - 32.0) * 5.0/9.0; // Convert back to Celsius
          Serial.print(" Heat Index: ");
          Serial.print(heatIndexC, 1);
          Serial.println(" °C                                        ");
        } else {
          Serial.println(" Heat Index: N/A (conditions not met)                            ");
        }
      }
    }
  }

  // ========== BIOMETRIC DATA (MAX30105) ==========
  Serial.println("──────────────────────────────────────────────────────────────────────");
  Serial.println(" ❤️  HEART RATE MONITOR (MAX30105)                                    ");
  Serial.println("──────────────────────────────────────────────────────────────────────");

  Serial.print(" IR Signal: ");
  Serial.print(irValue);

  if (irValue < 50000) {
    Serial.println("                                          ");
    Serial.println(" ⚠️  No finger detected - Place finger on sensor                      ");
  } else {
    Serial.println("                                    ");
    Serial.print(" Heart Rate: ");
    if (beatsPerMinute > 0) {
      Serial.print(beatsPerMinute, 1);
      Serial.println(" BPM                                       ");
      Serial.print(" Average BPM: ");
      Serial.print(beatAvg);
      Serial.println("                                              ");
      Serial.println(" ✅ Finger detected - Good signal quality                            ");
    } else {
      Serial.println("Calculating...                             ");
      Serial.println(" 🔄 Detecting heartbeat...                                           ");
    }
  }

  Serial.println("══════════════════════════════════════════════════════════════════════");
  Serial.println();
}

void loop() {
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

  // ========== PERIODIC SENSOR READINGS ==========
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();

    printSensorData(irValue);
  }
}