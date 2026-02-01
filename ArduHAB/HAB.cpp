#include "ArduHAB/HAB.h"

namespace arduhab {

ArduHABController::ArduHABController() : failsafe_active_(false) {
  for (uint8_t i = 0; i < static_cast<uint8_t>(CommsLink::kCount); ++i) {
    comms_enabled_[i] = false;
  }
}

void ArduHABController::Init() {
  failsafe_active_ = false;
  actuator_state_ = ActuatorState{};
  for (uint8_t i = 0; i < kMaxGpsModules; ++i) {
    gps_states_[i] = GpsState{};
  }
  for (uint8_t i = 0; i < kMaxPressureModules; ++i) {
    pressure_states_[i] = PressureState{};
  }
  for (uint8_t i = 0; i < kMaxTemperatureSensors; ++i) {
    temperature_states_[i] = TemperatureState{};
  }
  for (uint8_t i = 0; i < kMaxCutawayCircuits; ++i) {
    cutaway_states_[i] = CutawayState{};
  }
  for (uint8_t i = 0; i < kMaxHeaters; ++i) {
    heater_states_[i] = HeaterState{};
  }
}

void ArduHABController::Update() {
  ProcessCommands();
  UpdateSensors();
  UpdateComms();
  UpdateFailsafe();
  UpdateSystem();
}

bool ArduHABController::UpdateGps(uint8_t index, const GpsState& state) {
  if (!IsIndexValid(index, kMaxGpsModules)) {
    return false;
  }
  gps_states_[index] = state;
  return true;
}

bool ArduHABController::UpdatePressure(uint8_t index,
                                       const PressureState& state) {
  if (!IsIndexValid(index, kMaxPressureModules)) {
    return false;
  }
  if (state.pressure_mbar < kMinPressureMbar) {
    pressure_states_[index] = PressureState{};
    return false;
  }
  pressure_states_[index] = state;
  return true;
}

bool ArduHABController::UpdateTemperature(
    uint8_t index, const TemperatureState& state) {
  if (!IsIndexValid(index, kMaxTemperatureSensors)) {
    return false;
  }
  if (state.temperature_c < kMinTemperatureC) {
    temperature_states_[index] = TemperatureState{};
    return false;
  }
  temperature_states_[index] = state;
  return true;
}

bool ArduHABController::ArmCutaway(uint8_t index, bool armed) {
  if (!IsIndexValid(index, kMaxCutawayCircuits)) {
    return false;
  }
  cutaway_states_[index].armed = armed;
  if (!armed) {
    cutaway_states_[index].fired = false;
  }
  return true;
}

bool ArduHABController::FireCutaway(uint8_t index, uint32_t time_ms) {
  if (!IsIndexValid(index, kMaxCutawayCircuits)) {
    return false;
  }
  if (!cutaway_states_[index].armed) {
    return false;
  }
  cutaway_states_[index].fired = true;
  cutaway_states_[index].last_fire_ms = time_ms;
  return true;
}

bool ArduHABController::SetHeater(uint8_t index, bool enabled) {
  if (!IsIndexValid(index, kMaxHeaters)) {
    return false;
  }
  heater_states_[index].enabled = enabled;
  return true;
}

bool ArduHABController::SetActuatorPosition(uint8_t percent) {
  if (percent > kMaxActuatorPercent) {
    return false;
  }
  actuator_state_.enabled = true;
  actuator_state_.position_percent = percent;
  return true;
}

void ArduHABController::SetCommsEnabled(CommsLink link, bool enabled) {
  const uint8_t index = static_cast<uint8_t>(link);
  if (index >= static_cast<uint8_t>(CommsLink::kCount)) {
    return;
  }
  comms_enabled_[index] = enabled;
}

bool ArduHABController::IsCommsEnabled(CommsLink link) const {
  const uint8_t index = static_cast<uint8_t>(link);
  if (index >= static_cast<uint8_t>(CommsLink::kCount)) {
    return false;
  }
  return comms_enabled_[index];
}

bool ArduHABController::GetGps(uint8_t index, GpsState* out) const {
  if (out == nullptr) {
    return false;
  }
  if (!IsIndexValid(index, kMaxGpsModules)) {
    return false;
  }
  *out = gps_states_[index];
  return true;
}

bool ArduHABController::GetPressure(uint8_t index, PressureState* out) const {
  if (out == nullptr) {
    return false;
  }
  if (!IsIndexValid(index, kMaxPressureModules)) {
    return false;
  }
  *out = pressure_states_[index];
  return true;
}

bool ArduHABController::GetTemperature(uint8_t index,
                                       TemperatureState* out) const {
  if (out == nullptr) {
    return false;
  }
  if (!IsIndexValid(index, kMaxTemperatureSensors)) {
    return false;
  }
  *out = temperature_states_[index];
  return true;
}

bool ArduHABController::GetCutaway(uint8_t index, CutawayState* out) const {
  if (out == nullptr) {
    return false;
  }
  if (!IsIndexValid(index, kMaxCutawayCircuits)) {
    return false;
  }
  *out = cutaway_states_[index];
  return true;
}

bool ArduHABController::GetHeater(uint8_t index, HeaterState* out) const {
  if (out == nullptr) {
    return false;
  }
  if (!IsIndexValid(index, kMaxHeaters)) {
    return false;
  }
  *out = heater_states_[index];
  return true;
}

ActuatorState ArduHABController::GetActuator() const {
  return actuator_state_;
}

bool ArduHABController::IsFailsafeActive() const { return failsafe_active_; }

bool ArduHABController::IsIndexValid(uint8_t index, uint8_t max) const {
  return index < max;
}

}  // namespace arduhab
