#include "ArduHAB/HAB.h"

namespace arduhab {

void ArduHABController::UpdateFailsafe() {
  bool has_valid_gps = false;
  for (uint8_t i = 0; i < kMaxGpsModules; ++i) {
    if (gps_states_[i].valid) {
      has_valid_gps = true;
      break;
    }
  }

  bool has_valid_pressure = false;
  for (uint8_t i = 0; i < kMaxPressureModules; ++i) {
    if (pressure_states_[i].valid) {
      has_valid_pressure = true;
      break;
    }
  }

  failsafe_active_ = !has_valid_gps && !has_valid_pressure;
}

}  // namespace arduhab
