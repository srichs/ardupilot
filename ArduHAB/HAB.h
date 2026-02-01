#ifndef ARDUHAB_HAB_H_
#define ARDUHAB_HAB_H_

#include <cstdint>

#include "ArduHAB/config.h"

namespace arduhab {

struct GpsState {
  bool valid = false;
  int32_t latitude_e7 = 0;
  int32_t longitude_e7 = 0;
  int32_t altitude_cm = 0;
  uint32_t time_ms = 0;
};

struct PressureState {
  bool valid = false;
  float pressure_mbar = 0.0f;
};

struct TemperatureState {
  bool valid = false;
  float temperature_c = 0.0f;
};

struct CutawayState {
  bool armed = false;
  bool fired = false;
  uint32_t last_fire_ms = 0;
};

struct HeaterState {
  bool enabled = false;
};

struct ActuatorState {
  bool enabled = false;
  uint8_t position_percent = 0;
};

enum class CommsLink : uint8_t {
  kLora = 0,
  kAdsb = 1,
  kHorus = 2,
  kIridium = 3,
  kStarlink = 4,
  kCount = 5,
};

class ArduHABController {
 public:
  ArduHABController();

  void Init();
  void Update();

  bool UpdateGps(uint8_t index, const GpsState& state);
  bool UpdatePressure(uint8_t index, const PressureState& state);
  bool UpdateTemperature(uint8_t index, const TemperatureState& state);

  bool ArmCutaway(uint8_t index, bool armed);
  bool FireCutaway(uint8_t index, uint32_t time_ms);
  bool SetHeater(uint8_t index, bool enabled);
  bool SetActuatorPosition(uint8_t percent);

  void SetCommsEnabled(CommsLink link, bool enabled);
  bool IsCommsEnabled(CommsLink link) const;

  bool GetGps(uint8_t index, GpsState* out) const;
  bool GetPressure(uint8_t index, PressureState* out) const;
  bool GetTemperature(uint8_t index, TemperatureState* out) const;
  bool GetCutaway(uint8_t index, CutawayState* out) const;
  bool GetHeater(uint8_t index, HeaterState* out) const;
  ActuatorState GetActuator() const;
  bool IsFailsafeActive() const;

 private:
  void ProcessCommands();
  void UpdateSensors();
  void UpdateComms();
  void UpdateFailsafe();
  void UpdateSystem();

  bool IsIndexValid(uint8_t index, uint8_t max) const;

  GpsState gps_states_[kMaxGpsModules];
  PressureState pressure_states_[kMaxPressureModules];
  TemperatureState temperature_states_[kMaxTemperatureSensors];
  CutawayState cutaway_states_[kMaxCutawayCircuits];
  HeaterState heater_states_[kMaxHeaters];
  ActuatorState actuator_state_;
  bool comms_enabled_[static_cast<uint8_t>(CommsLink::kCount)];
  bool failsafe_active_;
};

}  // namespace arduhab

#endif  // ARDUHAB_HAB_H_
