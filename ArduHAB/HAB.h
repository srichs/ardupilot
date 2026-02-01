#ifndef ARDUHAB_HAB_H_
#define ARDUHAB_HAB_H_

#include <cstddef>
#include <cstdint>

#include "ArduHAB/config.h"

namespace ardupilot {
namespace hab {

enum class BalloonType : uint8_t {
  kSuperPressure = 0,
  kZeroPressure = 1,
  kLatexMeteorological = 2,
  kVentedLatexMeteorological = 3,
};

struct CommsConfig {
  bool lora_enabled;
  bool adsb_enabled;
  bool horus_enabled;
  bool iridium_enabled;
  bool starlink_enabled;
};

struct GeofenceConfig {
  bool enabled;
  double center_latitude_deg;
  double center_longitude_deg;
  float radius_m;
};

struct CutawayConfig {
  bool allow_manual_command;
  bool timer_enabled;
  uint32_t timer_seconds;
  GeofenceConfig geofence;
  bool max_altitude_enabled;
  float max_altitude_m;
};

struct GpsFix {
  bool valid;
  double latitude_deg;
  double longitude_deg;
  float altitude_m;
};

struct SensorReadings {
  float pressure_mbar[kMaxPressureSensors];
  float temperature_c[kMaxTemperatureSensors];
};

struct ActuatorConfig {
  bool cutaway_present[kMaxCutawayCircuits];
  bool heater_present[kMaxHeaters];
  bool vent_actuator_present;
};

struct ActuatorState {
  bool cutaway_enabled[kMaxCutawayCircuits];
  bool heater_enabled[kMaxHeaters];
  float vent_position;
};

struct CutawayStatus {
  bool manual_commanded;
  bool timer_elapsed;
  bool geofence_violation;
  bool altitude_violation;
  bool cutaway_active;
};

struct CutawayInputs {
  bool manual_commanded;
  uint32_t elapsed_seconds;
};

struct HabConfig {
  BalloonType balloon_type;
  CommsConfig comms;
  CutawayConfig cutaway;
  ActuatorConfig actuators;
  uint8_t gps_count;
  uint8_t pressure_sensor_count;
  uint8_t temperature_sensor_count;
};

struct HabStatus {
  GpsFix gps[kMaxGpsModules];
  SensorReadings sensors;
  ActuatorState actuators;
  CutawayStatus cutaway;
};

class ArduHab {
 public:
  ArduHab();

  void Init(const HabConfig& config);
  void UpdateSensors(const SensorReadings& readings);
  void UpdateGps(const GpsFix (&gps)[kMaxGpsModules]);
  void SetCutawayState(uint8_t index, bool enabled);
  void EvaluateCutaway(const CutawayInputs& inputs);
  void SetHeaterState(uint8_t index, bool enabled);
  void SetVentPosition(float position);
  const HabStatus& status() const;
  const HabConfig& config() const;

 private:
  void ResetActuators();
  void ApplySafetyLimits();

  HabConfig config_;
  HabStatus status_;
};

}  // namespace hab
}  // namespace ardupilot

#endif  // ARDUHAB_HAB_H_
