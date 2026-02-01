#include "ArduHAB/HAB.h"

namespace ardupilot {
namespace hab {

bool ValidateSensorCounts(const HabConfig& config) {
  if (config.gps_count > kMaxGpsModules) {
    return false;
  }
  if (config.pressure_sensor_count > kMaxPressureSensors) {
    return false;
  }
  if (config.temperature_sensor_count > kMaxTemperatureSensors) {
    return false;
  }
  return true;
}

}  // namespace hab
}  // namespace ardupilot
