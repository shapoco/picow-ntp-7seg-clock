#ifndef BRIGHTNESS_HPP
#define BRIGHTNESS_HPP

#include <cstdint>

#include "common.hpp"

namespace ntpc::brightness {

void init();
uint16_t read_adc();
void update(Context &ctx, uint64_t tick_ms);

}  // namespace ntpc::brightness

#endif  // BRIGHTNESS_HPP
