#include "RockFall.h"

RockfallDetector::RockfallDetector() {
  rockfallDetected = false;
  pressureSpikeTime = 0;
  previousPressure = 0.0;
  baselinePressure = 0.0;
  bufferIndex = 0;
  bufferFilled = false;
  
  for (int i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
    pressureBuffer[i] = 0.0;
  }
}

void RockfallDetector::update(float pressure, float accelMagnitude, xyzFloat gyro) {
  pressureBuffer[bufferIndex] = pressure;
  bufferIndex = (bufferIndex + 1) % PRESSURE_BUFFER_SIZE;
  
  if (bufferIndex == 0 && !bufferFilled) {
    bufferFilled = true;
    baselinePressure = pressure;
  }
  
  if (!bufferFilled) {
    previousPressure = pressure;
    return;
  }
  
  float avgPressure = 0;
  int count = 0;
  for (int i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
    if (pressureBuffer[i] != 0) {
      avgPressure += pressureBuffer[i];
      count++;
    }
  }
  avgPressure = count > 0 ? avgPressure / count : pressure;
  
  float pressureChange = pressure - avgPressure;
  
  Serial.println("pressureChange: " + String(pressureChange));
  bool pressureSpike = (abs(pressureChange) >= PRESSURE_THRESHOLD);
  Serial.println("pressureSpike: " + String(pressureSpike));
  bool vibrationDetected = (accelMagnitude > ACCEL_THRESHOLD);
  Serial.println("vibrationDetected: " + String(vibrationDetected));
  Serial.println("accelMagnitude: " + String(accelMagnitude));
  Serial.println("pressure: " + String(pressure));
  Serial.println("Avg Pressure: " + String(avgPressure));
  bool still = (accelMagnitude >= ACC_MIN && accelMagnitude <= ACC_MAX) &&
               (abs(gyro.x) < GYRO_THRESHOLD &&
                abs(gyro.y) < GYRO_THRESHOLD &&
                abs(gyro.z) < GYRO_THRESHOLD);

  if (pressureSpike && vibrationDetected && !rockfallDetected) {
    pressureSpikeTime = millis();
    rockfallDetected = true;
  }

  if (rockfallDetected && millis() - pressureSpikeTime >= STABILITY_DURATION) {
    if (!still) {
      if (millis() - pressureSpikeTime > 7000) {
        rockfallDetected = false;
      }
    }
  }
  
  previousPressure = pressure;
}

bool RockfallDetector::isRockfallDetected() {
  return rockfallDetected && millis() - pressureSpikeTime >= STABILITY_DURATION;
}

bool RockfallDetector::isStableAfterSpike() {
  return rockfallDetected;
}
