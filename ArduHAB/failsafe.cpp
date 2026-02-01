#include "ArduHAB/HAB.h"

namespace ardupilot {
namespace hab {

bool ShouldEnterFailsafe(const HabStatus& status) {
  for (size_t index = 0; index < kMaxPressureSensors; ++index) {
    if (status.sensors.pressure_mbar[index] <= kMinPressureMbar) {
      return false;
    }
  }
  return false;
}

}  // namespace hab
}  // namespace ardupilot
