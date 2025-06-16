#include "FallDetection.h"

FallDetector::FallDetector() {
  fallDetected = false;
  impactTime = 0;
  lastHeartRate = 0;
}

void FallDetector::update(float resultantG, xyzFloat gyro, int heartRate) {
  lastHeartRate = heartRate;

  bool impact = (resultantG > IMPACT_THRESHOLD);
  bool still = (resultantG >= ACC_MIN && resultantG <= ACC_MAX) &&
    (
      abs(gyro.x) < GYRO_THRESHOLD && 
      abs(gyro.y) < GYRO_THRESHOLD &&
      abs(gyro.z) < GYRO_THRESHOLD
    );
  
  if (impact && !fallDetected) {
    impactTime = millis();
    fallDetected = true;
  }

  
  if (fallDetected && millis() - impactTime >= STABILITY_DURATION) {
    if (!still) {
      if (millis() - impactTime > 7000) {
        fallDetected = false;
      }
    }
  }
}

bool FallDetector::isFallDetected() {
  return fallDetected && millis() - impactTime >= STABILITY_DURATION;
}

bool FallDetector::isHeartRateCritical() {
  return (lastHeartRate < 40 || lastHeartRate > 130);
}

bool FallDetector::isStableAfterFall() { return fallDetected; }
