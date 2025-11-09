#ifndef NTP_CLOCK_COMMON_HPP
#define NTP_CLOCK_COMMON_HPP

namespace ntpc {

enum class State {
  IDLE,
  SETUP,
  VLCFG_ERROR,
};

struct Context {
  State state;
  uint64_t epoch_ms;
    uint32_t epoch_sec;
  uint16_t epoch_tick_ms;
};

static constexpr int SYSTEM_CLOCK_HZ = 125000000;

}  // namespace ntpc

#endif
