#ifndef NTPC_CLI_HPP
#define NTPC_CLI_HPP

#include <pico/stdlib.h>

#include "common.hpp"

namespace cli {
using namespace ntpc;

void start(Config &cfg);
bool update(Config &cfg);

}  // namespace cli
#endif