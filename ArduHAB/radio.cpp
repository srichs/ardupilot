#include "ArduHAB/HAB.h"

namespace ardupilot {
namespace hab {

enum class CommsLink : uint8_t {
  kLora = 0,
  kAdsb = 1,
  kHorus = 2,
  kIridium = 3,
  kStarlink = 4,
};

CommsConfig DefaultCommsConfig() {
  return {false, false, false, false, false};
}

bool IsCommsEnabled(const CommsConfig& config, CommsLink link) {
  switch (link) {
    case CommsLink::kLora:
      return config.lora_enabled;
    case CommsLink::kAdsb:
      return config.adsb_enabled;
    case CommsLink::kHorus:
      return config.horus_enabled;
    case CommsLink::kIridium:
      return config.iridium_enabled;
    case CommsLink::kStarlink:
      return config.starlink_enabled;
  }
  return false;
}

}  // namespace hab
}  // namespace ardupilot
