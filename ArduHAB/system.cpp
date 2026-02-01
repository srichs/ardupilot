#include "ArduHAB/HAB.h"

namespace arduhab {

void ArduHABController::UpdateSystem() {
  if (!failsafe_active_) {
    return;
  }

  actuator_state_.enabled = false;
  actuator_state_.position_percent = 0;
  for (uint8_t i = 0; i < kMaxHeaters; ++i) {
    heater_states_[i].enabled = false;
  }
}

}  // namespace arduhab
