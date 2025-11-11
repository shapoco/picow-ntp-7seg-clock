#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <hardware/adc.h>
#include <pico/stdlib.h>

#include <cstdint>
#include <cstdio>

#include "common.hpp"
#include "eeprom.hpp"

#include <vlcfg/vlconfig.hpp>

namespace config {

using namespace ntpc;

bool init(Config &cfg);
void start(Config &cfg);
bool update(Config &cfg, bool *success);
bool is_receiving();
bool last_bit();
vlcfg::Result last_vlcfg_error();

}  // namespace config

#endif
