#ifndef NTP_CLIENT_HPP
#define NTP_CLIENT_HPP

#include "common.hpp"
#include "config.hpp"

namespace ntp {

using namespace ntpc;

result_t get_time(const config::Config &cfg, uint64_t *out_time);

}  // namespace ntp

#endif
