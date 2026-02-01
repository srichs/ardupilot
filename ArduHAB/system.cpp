#include "ArduHAB/HAB.h"

namespace ardupilot {
namespace hab {

void InitializeSystem(ArduHab& hab, const HabConfig& config) {
  hab.Init(config);
}

}  // namespace hab
}  // namespace ardupilot
