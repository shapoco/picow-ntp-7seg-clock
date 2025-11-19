#ifndef BRIGHTNESS_HPP
#define BRIGHTNESS_HPP

#include <cstdint>

#include "common.hpp"

namespace ntpc::brightness {

void init();
void update(Context &ctx, uint64_t tick_ms);
uint16_t read_adc();

}  // namespace ntpc::brightness

#endif  // BRIGHTNESS_HPP
