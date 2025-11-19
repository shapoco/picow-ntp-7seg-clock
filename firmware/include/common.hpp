#ifndef NTP_CLOCK_COMMON_HPP
#define NTP_CLOCK_COMMON_HPP

#include <cstdint>
#include <cstdio>

namespace ntpc {

static constexpr int REGION_MAX_LEN = 4;
static constexpr int SSID_MAX_LEN = 32;
static constexpr int PASSWORD_MAX_LEN = 32;
static constexpr int NTP_HOST_MAX_LEN = 128;
static constexpr int TIMEZONE_MAX_LEN = 8;

// AT24C32E
// https://akizukidenshi.com/catalog/g/g115715/
static constexpr int I2C_HOST = 1;
static constexpr int I2C_FREQ_HZ = 400000;
static constexpr int SDA_PORT = 26;
static constexpr int SCL_PORT = 27;

static constexpr int LIGHT_SENSOR_ADC_CH = 2;

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
  RTC_INIT_FAILED,
  RTC_READ_FAILED,
  RTC_WRITE_FAILED,
};

struct Context {
  state_t state;
  uint64_t origin_time_ms;
  uint64_t origin_tick_ms;
  uint64_t last_try_time_ms;
  uint64_t next_sync_tick_ms;
  result_t last_error;
  uint16_t brightness;
};

struct Config {
  char region[REGION_MAX_LEN + 1];
  char ssid[SSID_MAX_LEN + 1];
  char password[PASSWORD_MAX_LEN + 1];
  char ntp_host[NTP_HOST_MAX_LEN + 1];
  char timezone[TIMEZONE_MAX_LEN + 1];
};

static constexpr int SYSTEM_CLOCK_HZ = 125000000;

extern bool debug_mode;
#define NTPC_VERBOSE(...)                          \
  do {                                             \
    if (ntpc::debug_mode) {                        \
      printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
      printf(__VA_ARGS__);                         \
    }                                              \
  } while (0)

}  // namespace ntpc

#endif
