#include "ArduHAB/HAB.h"

namespace ardupilot {
namespace hab {

bool IsValidBalloonType(BalloonType balloon_type) {
  switch (balloon_type) {
    case BalloonType::kSuperPressure:
    case BalloonType::kZeroPressure:
    case BalloonType::kLatexMeteorological:
    case BalloonType::kVentedLatexMeteorological:
      return true;
  }
  return false;
}

}  // namespace hab
}  // namespace ardupilot
