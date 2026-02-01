#include "ArduHAB/HAB.h"

namespace ardupilot {
namespace hab {

namespace {

float Clamp01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

}  // namespace

ArduHab::ArduHab() : config_{}, status_{} {
  config_.balloon_type = BalloonType::kSuperPressure;
  config_.comms = {false, false, false, false, false};
  config_.gps_count = 0U;
  config_.pressure_sensor_count = 0U;
  config_.temperature_sensor_count = 0U;
  ResetActuators();
}

void ArduHab::Init(const HabConfig& config) {
  config_ = config;

  if (config_.gps_count > kMaxGpsModules) {
    config_.gps_count = kMaxGpsModules;
  }
  if (config_.pressure_sensor_count > kMaxPressureSensors) {
    config_.pressure_sensor_count = kMaxPressureSensors;
  }
  if (config_.temperature_sensor_count > kMaxTemperatureSensors) {
    config_.temperature_sensor_count = kMaxTemperatureSensors;
  }

  ResetActuators();
  ApplySafetyLimits();
}

void ArduHab::UpdateSensors(const SensorReadings& readings) {
  status_.sensors = readings;
  ApplySafetyLimits();
}

void ArduHab::UpdateGps(const GpsFix (&gps)[kMaxGpsModules]) {
  for (size_t index = 0; index < kMaxGpsModules; ++index) {
    status_.gps[index] = gps[index];
  }
}

void ArduHab::SetCutawayState(uint8_t index, bool enabled) {
  if (index >= kMaxCutawayCircuits) {
    return;
  }
  status_.actuators.cutaway_enabled[index] = enabled;
}

void ArduHab::SetHeaterState(uint8_t index, bool enabled) {
  if (index >= kMaxHeaters) {
    return;
  }
  status_.actuators.heater_enabled[index] = enabled;
}

void ArduHab::SetVentPosition(float position) {
  status_.actuators.vent_position = Clamp01(position);
}

const HabStatus& ArduHab::status() const {
  return status_;
}

const HabConfig& ArduHab::config() const {
  return config_;
}

void ArduHab::ResetActuators() {
  for (size_t index = 0; index < kMaxCutawayCircuits; ++index) {
    status_.actuators.cutaway_enabled[index] = false;
  }
  for (size_t index = 0; index < kMaxHeaters; ++index) {
    status_.actuators.heater_enabled[index] = false;
  }
  status_.actuators.vent_position = 0.0f;
}

void ArduHab::ApplySafetyLimits() {
  for (size_t index = 0; index < kMaxPressureSensors; ++index) {
    if (status_.sensors.pressure_mbar[index] < kMinPressureMbar) {
      status_.sensors.pressure_mbar[index] = kMinPressureMbar;
    }
  }
  for (size_t index = 0; index < kMaxTemperatureSensors; ++index) {
    if (status_.sensors.temperature_c[index] < kMinTemperatureC) {
      status_.sensors.temperature_c[index] = kMinTemperatureC;
    }
  }
}

}  // namespace hab
}  // namespace ardupilot
