#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <cstdint>
#include <cstring>

#include "common.hpp"
#include "config.hpp"

namespace display {

using namespace ntpc;

void init();
void update_display(Context &ctx);
void turn_off();

}  // namespace display

#endif
