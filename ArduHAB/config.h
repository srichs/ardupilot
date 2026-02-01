#ifndef ARDUHAB_CONFIG_H_
#define ARDUHAB_CONFIG_H_

#include <cstdint>

namespace ardupilot {
namespace hab {

constexpr uint8_t kMaxGpsModules = 2;
constexpr uint8_t kMaxPressureSensors = 2;
constexpr uint8_t kMaxTemperatureSensors = 2;
constexpr uint8_t kMaxCutawayCircuits = 2;
constexpr uint8_t kMaxHeaters = 2;

constexpr float kMinPressureMbar = 10.0f;
constexpr float kMinTemperatureC = -55.0f;

}  // namespace hab
}  // namespace ardupilot

#endif  // ARDUHAB_CONFIG_H_
