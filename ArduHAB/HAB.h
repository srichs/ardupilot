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

struct ActuatorState {
  bool cutaway_enabled[kMaxCutawayCircuits];
  bool heater_enabled[kMaxHeaters];
  float vent_position;
};

struct HabConfig {
  BalloonType balloon_type;
  CommsConfig comms;
  uint8_t gps_count;
  uint8_t pressure_sensor_count;
  uint8_t temperature_sensor_count;
};

struct HabStatus {
  GpsFix gps[kMaxGpsModules];
  SensorReadings sensors;
  ActuatorState actuators;
};

class ArduHab {
 public:
  ArduHab();

  void Init(const HabConfig& config);
  void UpdateSensors(const SensorReadings& readings);
  void UpdateGps(const GpsFix (&gps)[kMaxGpsModules]);
  void SetCutawayState(uint8_t index, bool enabled);
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
