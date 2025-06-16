#ifndef FALL_DETECTION_H
#define FALL_DETECTION_H

#include <Arduino.h>
#include <MPU6500_WE.h>

class FallDetector {
public:
  FallDetector();

  void update(float resultantG, xyzFloat gyro, int heartRate);
  bool isFallDetected();
  bool isHeartRateCritical();
  bool isStableAfterFall();

private:
  bool fallDetected;
  unsigned long impactTime;
  const float FREE_FALL_THRESHOLD = 0.5;
  const float IMPACT_THRESHOLD = 2.0;
  const unsigned long STABILITY_DURATION = 3000;
  const float GYRO_THRESHOLD = 1.0;
  const float ACC_MIN = 0.9;
  const float ACC_MAX = 1.1;
  int lastHeartRate;
};

#endif
