#include "config.hpp"
#include "cli.hpp"

namespace config {

using namespace ntpc;

// AT24C32E
// https://akizukidenshi.com/catalog/g/g115715/
static constexpr int I2C_HOST = 1;
static constexpr int I2C_FREQ_HZ = 400000;
static constexpr int SDA_PORT = 26;
static constexpr int SCL_PORT = 27;
static constexpr int PAGE_SIZE = 32;
static constexpr int MEMORY_SIZE = 1024 * 4;

static constexpr int LIGHT_SENSOR_ADC_CH = 2;

static constexpr eeprom::addr_t OFFSET_REGION = 0;
static constexpr eeprom::addr_t OFFSET_SSID = OFFSET_REGION + REGION_MAX_LEN;
static constexpr eeprom::addr_t OFFSET_PASSWORD = OFFSET_SSID + SSID_MAX_LEN;
static constexpr eeprom::addr_t OFFSET_NTP_HOST =
    OFFSET_PASSWORD + PASSWORD_MAX_LEN;
static constexpr eeprom::addr_t OFFSET_TIMEZONE =
    OFFSET_NTP_HOST + NTP_HOST_MAX_LEN;
static constexpr eeprom::addr_t OFFSET_CRC = OFFSET_TIMEZONE + TIMEZONE_MAX_LEN;
static constexpr eeprom::addr_t EEPROM_CONFIG_SIZE =
    OFFSET_CRC + sizeof(uint32_t);

const char *KEY_REGION = "c";
const char *KEY_SSID = "s";
const char *KEY_PASSWORD = "p";
const char *KEY_NTP_HOST = "n";
const char *KEY_TIMEZONE = "z";

Config buff;

vlcfg::ConfigEntry config_entries[] = {
    {KEY_REGION, buff.region, vlcfg::ValueType::TEXT_STR, REGION_MAX_LEN},
    {KEY_SSID, buff.ssid, vlcfg::ValueType::TEXT_STR, SSID_MAX_LEN},
    {KEY_PASSWORD, buff.password, vlcfg::ValueType::TEXT_STR, PASSWORD_MAX_LEN},
    {KEY_NTP_HOST, buff.ntp_host, vlcfg::ValueType::TEXT_STR, NTP_HOST_MAX_LEN},
    {KEY_TIMEZONE, buff.timezone, vlcfg::ValueType::TEXT_STR, TIMEZONE_MAX_LEN},
    {nullptr, nullptr, vlcfg::ValueType::NONE, 0},  // terminator
};
vlcfg::Receiver receiver(256);
vlcfg::Result vlcfg_err = vlcfg::Result::SUCCESS;

eeprom::Driver rom(I2C_HOST, SDA_PORT, SCL_PORT, MEMORY_SIZE, PAGE_SIZE);

static uint64_t t_next_sensor_read_ms = 0;

static bool eeprom_load(Config &cfg);
static bool eeprom_save(const Config &cfg);
static uint32_t calc_crc32(const uint8_t *data, size_t length);

#define strncpy_nullterm(dest, src, max_len)                 \
  do {                                                       \
    strncpy((char *)(dest), (const char *)(src), (max_len)); \
    ((char *)(dest))[(max_len)] = '\0';                      \
  } while (0)

bool init(Config &cfg) {
  adc_init();
  adc_gpio_init(26 + LIGHT_SENSOR_ADC_CH);
  adc_select_input(LIGHT_SENSOR_ADC_CH);
  NTPC_PRINTF("Initializing EEPROM...\n");
  rom.init();
  NTPC_PRINTF("Loading config from EEPROM...\n");
  return eeprom_load(cfg);
}

void start(Config &cfg) {
  buff.region[0] = '\0';
  buff.ssid[0] = '\0';
  buff.password[0] = '\0';
  buff.ntp_host[0] = '\0';
  buff.timezone[0] = '\0';
  receiver.init(config_entries);
  const uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  t_next_sensor_read_ms = now_ms + 10;

  if (ntpc::stdio_inited) {
    cli::start(cfg);
  }
}

bool update(Config &cfg, bool *success) {
  bool completed = false;

  *success = false;

  const uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  if (now_ms > t_next_sensor_read_ms) {
    t_next_sensor_read_ms += 10;

    vlcfg::RxState rx_state;
    vlcfg_err = receiver.update(adc_read(), &rx_state);
    if (vlcfg_err != vlcfg::Result::SUCCESS ||
        rx_state == vlcfg::RxState::ERROR) {
      NTPC_PRINTF("RX error: %d\n", static_cast<int>(vlcfg_err));
      return true;
    } else if (rx_state == vlcfg::RxState::COMPLETED) {
      cfg = buff;
      NTPC_PRINTF("RX completed successfully\n");
      completed = true;
    }
  }

  if (!completed && ntpc::stdio_inited) {
    completed |= cli::update(cfg);
  }

  if (completed) {
    if (cfg.ntp_host[0] == '\0') {
      strncpy(cfg.ntp_host, "pool.ntp.org", NTP_HOST_MAX_LEN);
    }
    if (cfg.timezone[0] == '\0') {
      strncpy(cfg.timezone, "0000", TIMEZONE_MAX_LEN);
    }

    NTPC_PRINTF("Region  : '%s'\n", cfg.region);
    NTPC_PRINTF("SSID    : '%s'\n", cfg.ssid);
    NTPC_PRINTF("Password: '%s'\n", cfg.password);
    NTPC_PRINTF("NTP Host: '%s'\n", cfg.ntp_host);
    NTPC_PRINTF("Timezone: '%s'\n", cfg.timezone);

    if (eeprom_save(cfg)) {
      NTPC_PRINTF("Config saved to EEPROM\n");
    } else {
      NTPC_PRINTF("Failed to save config to EEPROM\n");
    }

    *success = true;
  }

  return completed;
}

bool is_receiving() {
  return receiver.decoder.get_state() == vlcfg::RxState::RECEIVING;
}

bool last_bit() { return receiver.get_last_bit(); }

vlcfg::Result last_vlcfg_error() { return vlcfg_err; }

static bool eeprom_load(Config &cfg) {
  bool success = false;
  uint8_t *buff = new uint8_t[EEPROM_CONFIG_SIZE];

  if (rom.read_data(0, buff, EEPROM_CONFIG_SIZE) != eeprom::result_t::SUCCESS) {
    NTPC_PRINTF("Failed to read config from EEPROM\n");
    goto failed;
  }

  {
    uint32_t stored_crc = ((uint32_t)(buff[OFFSET_CRC + 0]) << 0) |
                          ((uint32_t)(buff[OFFSET_CRC + 1]) << 8) |
                          ((uint32_t)(buff[OFFSET_CRC + 2]) << 16) |
                          ((uint32_t)(buff[OFFSET_CRC + 3]) << 24);
    uint32_t calc_crc = calc_crc32(buff, OFFSET_CRC);
    if (stored_crc != calc_crc) {
      NTPC_PRINTF("EEPROM config CRC mismatch: stored=0x%08lX, calc=0x%08lX\n",
                  stored_crc, calc_crc);
      goto failed;
    }
  }

  strncpy_nullterm(cfg.region, buff + OFFSET_REGION, REGION_MAX_LEN);
  strncpy_nullterm(cfg.ssid, buff + OFFSET_SSID, SSID_MAX_LEN);
  strncpy_nullterm(cfg.password, buff + OFFSET_PASSWORD, PASSWORD_MAX_LEN);
  strncpy_nullterm(cfg.ntp_host, buff + OFFSET_NTP_HOST, NTP_HOST_MAX_LEN);
  strncpy_nullterm(cfg.timezone, buff + OFFSET_TIMEZONE, TIMEZONE_MAX_LEN);

  success = true;

failed:
  delete[] buff;
  return success;
}

static bool eeprom_save(const Config &cfg) {
  bool success = false;
  uint8_t *buff = new uint8_t[EEPROM_CONFIG_SIZE];

  memset(buff, 0, EEPROM_CONFIG_SIZE);
  memcpy(buff + OFFSET_REGION, cfg.region, REGION_MAX_LEN);
  memcpy(buff + OFFSET_SSID, cfg.ssid, SSID_MAX_LEN);
  memcpy(buff + OFFSET_PASSWORD, cfg.password, PASSWORD_MAX_LEN);
  memcpy(buff + OFFSET_NTP_HOST, cfg.ntp_host, NTP_HOST_MAX_LEN);
  memcpy(buff + OFFSET_TIMEZONE, cfg.timezone, TIMEZONE_MAX_LEN);
  uint32_t crc = calc_crc32(buff, OFFSET_CRC);
  buff[OFFSET_CRC + 0] = (crc >> 0) & 0xFF;
  buff[OFFSET_CRC + 1] = (crc >> 8) & 0xFF;
  buff[OFFSET_CRC + 2] = (crc >> 16) & 0xFF;
  buff[OFFSET_CRC + 3] = (crc >> 24) & 0xFF;

  if (rom.write_data(0, buff, EEPROM_CONFIG_SIZE) !=
      eeprom::result_t::SUCCESS) {
    NTPC_PRINTF("Failed to write config to EEPROM\n");
    goto failed;
  }
  success = true;

failed:
  delete[] buff;

  return success;
}

static uint32_t calc_crc32(const uint8_t *data, size_t length) {
  static constexpr uint32_t POLY = 0xEDB88320;

  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (uint8_t j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (POLY & mask);
    }
  }
  return ~crc;
}

}  // namespace config
