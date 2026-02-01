#include "ArduHAB/HAB.h"

#include <cmath>

namespace ardupilot {
namespace hab {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusMeters = 6371000.0;

float Clamp01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

double DegreesToRadians(double degrees) {
  return degrees * (kPi / 180.0);
}

double HaversineDistanceMeters(double lat1_deg, double lon1_deg,
                               double lat2_deg, double lon2_deg) {
  const double lat1_rad = DegreesToRadians(lat1_deg);
  const double lat2_rad = DegreesToRadians(lat2_deg);
  const double dlat = lat2_rad - lat1_rad;
  const double dlon = DegreesToRadians(lon2_deg - lon1_deg);
  const double sin_dlat = std::sin(dlat * 0.5);
  const double sin_dlon = std::sin(dlon * 0.5);
  const double a = sin_dlat * sin_dlat +
                   std::cos(lat1_rad) * std::cos(lat2_rad) * sin_dlon *
                       sin_dlon;
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return kEarthRadiusMeters * c;
}

bool FindValidFix(const GpsFix (&gps)[kMaxGpsModules], GpsFix* fix) {
  if (fix == nullptr) {
    return false;
  }
  for (size_t index = 0; index < kMaxGpsModules; ++index) {
    if (gps[index].valid) {
      *fix = gps[index];
      return true;
    }
  }
  return false;
}

}  // namespace

ArduHab::ArduHab() : config_{}, status_{} {
  config_.balloon_type = BalloonType::kSuperPressure;
  config_.comms = {false, false, false, false, false};
  config_.cutaway.allow_manual_command = true;
  config_.cutaway.timer_enabled = false;
  config_.cutaway.timer_seconds = 0U;
  config_.cutaway.geofence.enabled = false;
  config_.cutaway.geofence.center_latitude_deg = 0.0;
  config_.cutaway.geofence.center_longitude_deg = 0.0;
  config_.cutaway.geofence.radius_m = 0.0f;
  config_.cutaway.max_altitude_enabled = false;
  config_.cutaway.max_altitude_m = 0.0f;
  for (size_t index = 0; index < kMaxCutawayCircuits; ++index) {
    config_.actuators.cutaway_present[index] = true;
  }
  for (size_t index = 0; index < kMaxHeaters; ++index) {
    config_.actuators.heater_present[index] = true;
  }
  config_.actuators.vent_actuator_present = false;
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
  if (!config_.actuators.cutaway_present[index]) {
    status_.actuators.cutaway_enabled[index] = false;
    return;
  }
  status_.actuators.cutaway_enabled[index] = enabled;
}

void ArduHab::EvaluateCutaway(const CutawayInputs& inputs) {
  const bool manual_allowed = config_.cutaway.allow_manual_command;
  status_.cutaway.manual_commanded = manual_allowed && inputs.manual_commanded;

  status_.cutaway.timer_elapsed = config_.cutaway.timer_enabled &&
                                  (inputs.elapsed_seconds >=
                                   config_.cutaway.timer_seconds);

  status_.cutaway.geofence_violation = false;
  status_.cutaway.altitude_violation = false;

  GpsFix fix{};
  const bool has_fix = FindValidFix(status_.gps, &fix);
  if (has_fix) {
    if (config_.cutaway.geofence.enabled &&
        config_.cutaway.geofence.radius_m > 0.0f) {
      const double distance = HaversineDistanceMeters(
          fix.latitude_deg, fix.longitude_deg,
          config_.cutaway.geofence.center_latitude_deg,
          config_.cutaway.geofence.center_longitude_deg);
      status_.cutaway.geofence_violation =
          distance > static_cast<double>(config_.cutaway.geofence.radius_m);
    }

    if (config_.cutaway.max_altitude_enabled) {
      status_.cutaway.altitude_violation =
          fix.altitude_m > config_.cutaway.max_altitude_m;
    }
  }

  status_.cutaway.cutaway_active =
      status_.cutaway.manual_commanded || status_.cutaway.timer_elapsed ||
      status_.cutaway.geofence_violation || status_.cutaway.altitude_violation;

  if (status_.cutaway.cutaway_active) {
    for (size_t index = 0; index < kMaxCutawayCircuits; ++index) {
      status_.actuators.cutaway_enabled[index] = true;
    }
  }
}

void ArduHab::SetHeaterState(uint8_t index, bool enabled) {
  if (index >= kMaxHeaters) {
    return;
  }
  if (!config_.actuators.heater_present[index]) {
    status_.actuators.heater_enabled[index] = false;
    return;
  }
  status_.actuators.heater_enabled[index] = enabled;
}

void ArduHab::SetVentPosition(float position) {
  if (!config_.actuators.vent_actuator_present ||
      config_.balloon_type != BalloonType::kVentedLatexMeteorological) {
    status_.actuators.vent_position = 0.0f;
    return;
  }
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
  status_.cutaway.manual_commanded = false;
  status_.cutaway.timer_elapsed = false;
  status_.cutaway.geofence_violation = false;
  status_.cutaway.altitude_violation = false;
  status_.cutaway.cutaway_active = false;
}

void ArduHab::ApplySafetyLimits() {
  for (size_t index = 0; index < kMaxPressureSensors; ++index) {
    if (index >= config_.pressure_sensor_count) {
      status_.sensors.pressure_mbar[index] = kMinPressureMbar;
      continue;
    }
    if (!std::isfinite(status_.sensors.pressure_mbar[index]) ||
        status_.sensors.pressure_mbar[index] < kMinPressureMbar) {
      status_.sensors.pressure_mbar[index] = kMinPressureMbar;
    }
  }
  for (size_t index = 0; index < kMaxTemperatureSensors; ++index) {
    if (index >= config_.temperature_sensor_count) {
      status_.sensors.temperature_c[index] = kMinTemperatureC;
      continue;
    }
    if (!std::isfinite(status_.sensors.temperature_c[index]) ||
        status_.sensors.temperature_c[index] < kMinTemperatureC) {
      status_.sensors.temperature_c[index] = kMinTemperatureC;
    }
  }
  for (size_t index = 0; index < kMaxGpsModules; ++index) {
    if (index >= config_.gps_count) {
      status_.gps[index].valid = false;
    }
  }
  for (size_t index = 0; index < kMaxCutawayCircuits; ++index) {
    if (!config_.actuators.cutaway_present[index]) {
      status_.actuators.cutaway_enabled[index] = false;
    }
  }
  for (size_t index = 0; index < kMaxHeaters; ++index) {
    if (!config_.actuators.heater_present[index]) {
      status_.actuators.heater_enabled[index] = false;
    }
  }
  if (!config_.actuators.vent_actuator_present ||
      config_.balloon_type != BalloonType::kVentedLatexMeteorological) {
    status_.actuators.vent_position = 0.0f;
  }
}

}  // namespace hab
}  // namespace ardupilot
