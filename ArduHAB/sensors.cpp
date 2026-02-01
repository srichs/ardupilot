#include "ArduHAB/HAB.h"

namespace arduhab {

void ArduHABController::UpdateSensors() {
  for (uint8_t i = 0; i < kMaxPressureModules; ++i) {
    if (pressure_states_[i].valid &&
        pressure_states_[i].pressure_mbar < kMinPressureMbar) {
      pressure_states_[i] = PressureState{};
    }
  }

  for (uint8_t i = 0; i < kMaxTemperatureSensors; ++i) {
    if (temperature_states_[i].valid &&
        temperature_states_[i].temperature_c < kMinTemperatureC) {
      temperature_states_[i] = TemperatureState{};
    }
  }
}

}  // namespace arduhab
