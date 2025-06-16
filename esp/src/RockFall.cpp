#include "RockFall.h"

RockfallDetector::RockfallDetector() {
  rockfallDetected = false;
  pressureSpikeTime = 0;
  previousPressure = 0.0;
  baselinePressure = 0.0;
  bufferIndex = 0;
  bufferFilled = false;
  
  // Initialize buffer with zeros
  for (int i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
    pressureBuffer[i] = 0.0;
  }
}

void RockfallDetector::update(float pressure, float accelMagnitude, xyzFloat gyro) {
  // Store the current pressure in the buffer
  pressureBuffer[bufferIndex] = pressure;
  bufferIndex = (bufferIndex + 1) % PRESSURE_BUFFER_SIZE;
  
  // Mark when buffer is filled first time
  if (bufferIndex == 0 && !bufferFilled) {
    bufferFilled = true;
    baselinePressure = pressure;
  }
  
  // Skip pressure spike detection until buffer has enough data
  if (!bufferFilled) {
    previousPressure = pressure;
    return;
  }
  
  // Calculate average pressure from buffer (excluding current reading)
  float avgPressure = 0;
  int count = 0;
  for (int i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
    if (pressureBuffer[i] != 0) {
      avgPressure += pressureBuffer[i];
      count++;
    }
  }
  avgPressure = count > 0 ? avgPressure / count : pressure;
  
  // Compare current pressure with average of previous readings
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

  // Step 1: Detect spike in pressure + movement
  if (pressureSpike && vibrationDetected && !rockfallDetected) {
    pressureSpikeTime = millis();
    rockfallDetected = true;
  }

  // Step 2: Check if stable (after 3s)
  if (rockfallDetected && millis() - pressureSpikeTime >= STABILITY_DURATION) {
    if (!still) {
      // If it remains unstable for too long, reset
      if (millis() - pressureSpikeTime > 7000) {
        rockfallDetected = false;
      }
    }
  }
  
  // Store current pressure for next iteration
  previousPressure = pressure;
}

bool RockfallDetector::isRockfallDetected() {
  return rockfallDetected && millis() - pressureSpikeTime >= STABILITY_DURATION;
}

bool RockfallDetector::isStableAfterSpike() {
  return rockfallDetected;
}
