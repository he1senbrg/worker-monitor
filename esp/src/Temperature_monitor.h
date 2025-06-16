#ifndef TEMPERATURE_MONITOR_H
#define TEMPERATURE_MONITOR_H

#include <Arduino.h>
class TemperatureMonitor {
public:
  TemperatureMonitor();
  void update(float temperature);
  bool shouldTakeBreak();
private:
  float currentTemperature;
  unsigned long lastBreakTime;
  bool breakInProgress;
  const float HIGH_TEMP_THRESHOLD = 38.0;
  const float LOW_TEMP_THRESHOLD = 10.0;
  const unsigned long BREAK_INTERVAL = 3600000;
  const unsigned long BREAK_DURATION = 900000;
};
#endif
