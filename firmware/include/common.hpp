#ifndef NTP_CLOCK_COMMON_HPP
#define NTP_CLOCK_COMMON_HPP

#include <cstdint>

namespace ntpc {

static constexpr int REGION_MAX_LEN = 4;
static constexpr int SSID_MAX_LEN = 32;
static constexpr int PASSWORD_MAX_LEN = 32;
static constexpr int NTP_HOST_MAX_LEN = 128;
static constexpr int TIMEZONE_MAX_LEN = 8;

enum class state_t {
  IDLE,
  SETUP,
  VLCFG_ERROR,
};

enum class result_t {
  SUCCESS,
  UNKNOWN_ERROR,
  NULL_POINTER,
  LWIP_ARCH_INIT_FAILED,
  LWIP_WIFI_CONNECT_FAILED,
  DNS_FAILED,
  DNS_TIMEOUT,
  NTP_FAILED,
  NTP_TIMEOUT,
};

struct Context {
  state_t state;
  uint64_t origin_time_ms;
  uint64_t origin_tick_ms;
  uint64_t last_try_time_ms;
  uint64_t next_sync_tick_ms;
  result_t last_error;
};

struct Config {
  char region[REGION_MAX_LEN + 1];
  char ssid[SSID_MAX_LEN + 1];
  char password[PASSWORD_MAX_LEN + 1];
  char ntp_host[NTP_HOST_MAX_LEN + 1];
  char timezone[TIMEZONE_MAX_LEN + 1];
};

static constexpr int SYSTEM_CLOCK_HZ = 125000000;

extern bool stdio_inited;
#define NTPC_PRINTF(...)                           \
  do {                                             \
    if (ntpc::stdio_inited) {                      \
      printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
      printf(__VA_ARGS__);                         \
    }                                              \
  } while (0)

}  // namespace ntpc

#endif
