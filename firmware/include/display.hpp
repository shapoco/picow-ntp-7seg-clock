#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <cstdint>
#include <cstring>

#include "common.hpp"
#include "config.hpp"

namespace display {

using namespace ntpc;

void init();
void update_display(Context &ctx, uint64_t tick_ms);
void turn_off();

}  // namespace display

#endif
