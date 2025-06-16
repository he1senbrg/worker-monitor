#include "TemperatureMonitor.h"

TemperatureMonitor::TemperatureMonitor() {
  currentTemperature = 0.0;
  lastBreakTime = millis();
  breakInProgress = false;
}
void TemperatureMonitor::update(float temperature) {
  currentTemperature = temperature;
  unsigned long currentTime = millis();
  if ((currentTemperature >= HIGH_TEMP_THRESHOLD || currentTemperature <= LOW_TEMP_THRESHOLD) &&
      (currentTime - lastBreakTime >= BREAK_INTERVAL)) {
    breakInProgress = true;
    lastBreakTime = currentTime;
  }
  if (breakInProgress && currentTime - lastBreakTime >= BREAK_DURATION) {
    breakInProgress = false;
  }
}
bool TemperatureMonitor::shouldTakeBreak() {
  return breakInProgress;
}
