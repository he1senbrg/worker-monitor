#ifndef ROCKFALL_DETECTION_H
#define ROCKFALL_DETECTION_H

#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <MPU6500_WE.h>

class RockfallDetector {
public:
  RockfallDetector();

  void update(float pressure, float accelMagnitude, xyzFloat gyro);
  bool isRockfallDetected();
  bool isStableAfterSpike();

private:
  bool rockfallDetected;
  unsigned long pressureSpikeTime;

  const float PRESSURE_THRESHOLD = 2.0;
  const float ACCEL_THRESHOLD = 1.2;
  const float GYRO_THRESHOLD = 1.0;
  const float ACC_MIN = 0.9;
  const float ACC_MAX = 1.1;
  const unsigned long STABILITY_DURATION = 3000;

  static const int PRESSURE_BUFFER_SIZE = 5;
  float pressureBuffer[PRESSURE_BUFFER_SIZE];
  int bufferIndex;
  bool bufferFilled;

  float previousPressure;
  float baselinePressure;
};

#endif
