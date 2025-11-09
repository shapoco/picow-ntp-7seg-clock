#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <hardware/adc.h>
#include <pico/stdlib.h>

#include <cstdint>
#include <cstdio>

#include "eeprom.hpp"

#include <vlcfg/vlconfig.hpp>

namespace config {

// AT24C32E
// https://akizukidenshi.com/catalog/g/g115715/
static constexpr int EEPROM_I2C_HOST = 1;
static constexpr int EEPROM_SDA_PORT = 26;
static constexpr int EEPROM_SCL_PORT = 27;
static constexpr int EEPROM_PAGE_SIZE = 32;
static constexpr int EEPROM_MEMORY_SIZE = 1024 * 4;
static constexpr int EEPROM_I2C_FREQ_HZ = 400000;

static constexpr int LIGHT_SENSOR_ADC_CH = 2;

static constexpr int COUNTRY_MAX_LEN = 4;
static constexpr int SSID_MAX_LEN = 32;
static constexpr int PASS_MAX_LEN = 32;
static constexpr int NTP_MAX_LEN = 128;
static constexpr int TIMEZONE_MAX_LEN = 8;

static constexpr eeprom::addr_t EEPROM_COUNTRY_OFFSET = 0;
static constexpr eeprom::addr_t EEPROM_SSID_OFFSET =
    EEPROM_COUNTRY_OFFSET + COUNTRY_MAX_LEN;
static constexpr eeprom::addr_t EEPROM_PASS_OFFSET =
    EEPROM_SSID_OFFSET + SSID_MAX_LEN;
static constexpr eeprom::addr_t EEPROM_NTP_OFFSET =
    EEPROM_PASS_OFFSET + PASS_MAX_LEN;
static constexpr eeprom::addr_t EEPROM_TIMEZONE_OFFSET =
    EEPROM_NTP_OFFSET + NTP_MAX_LEN;
static constexpr eeprom::addr_t EEPROM_CONFIG_CRC_OFFSET =
    EEPROM_TIMEZONE_OFFSET + TIMEZONE_MAX_LEN;
static constexpr eeprom::addr_t EEPROM_CONFIG_SIZE =
    EEPROM_CONFIG_CRC_OFFSET + sizeof(uint32_t);

struct Data {
  char country[COUNTRY_MAX_LEN + 1];
  char ssid[SSID_MAX_LEN + 1];
  char pass[PASS_MAX_LEN + 1];
  char ntp_host[NTP_MAX_LEN + 1];
  char timezone[8 + 1];
};

bool init(Data &cfg);
void recv_start();
bool recv_update(bool *success, Data &cfg);
bool is_receiving();
bool last_bit();
vlcfg::Result last_vlcfg_error();

#ifdef NTPC_CONFIG_IMPLEMENTATION

const char *KEY_COUNTRY = "c";
const char *KEY_SSID = "s";
const char *KEY_PASS = "p";
const char *KEY_NTP = "n";
const char *KEY_TIMEZONE = "z";

static char country_buff[COUNTRY_MAX_LEN + 1];
static char ssid_buff[SSID_MAX_LEN + 1];
static char pass_buff[PASS_MAX_LEN + 1];
static char ntp_buff[NTP_MAX_LEN + 1];
static char timezone_buff[TIMEZONE_MAX_LEN + 1];

vlcfg::ConfigEntry config_entries[] = {
    {KEY_COUNTRY, country_buff, vlcfg::ValueType::TEXT_STR,
     sizeof(country_buff)},
    {KEY_SSID, ssid_buff, vlcfg::ValueType::TEXT_STR, sizeof(ssid_buff)},
    {KEY_PASS, pass_buff, vlcfg::ValueType::TEXT_STR, sizeof(pass_buff)},
    {KEY_NTP, ntp_buff, vlcfg::ValueType::TEXT_STR, sizeof(ntp_buff)},
    {KEY_TIMEZONE, timezone_buff, vlcfg::ValueType::TEXT_STR,
     sizeof(timezone_buff)},
    {nullptr, nullptr, vlcfg::ValueType::NONE, 0},  // terminator
};
vlcfg::Receiver receiver(256);
vlcfg::Result vlcfg_err = vlcfg::Result::SUCCESS;

eeprom::Driver rom(EEPROM_I2C_HOST, EEPROM_SDA_PORT, EEPROM_SCL_PORT,
                   EEPROM_MEMORY_SIZE, EEPROM_PAGE_SIZE);

static uint64_t t_next_sensor_read_ms = 0;

static bool eeprom_load_config(Data &cfg);
static bool eeprom_save_config(const Data &cfg);
static uint32_t calc_crc32(const uint8_t *data, size_t length);

bool init(Data &cfg) {
  adc_init();
  adc_gpio_init(26 + LIGHT_SENSOR_ADC_CH);
  adc_select_input(LIGHT_SENSOR_ADC_CH);
  printf("Initializing EEPROM...\n");
  rom.init();
  printf("Loading config from EEPROM...\n");
  return eeprom_load_config(cfg);
}

void recv_start() {
  strncpy(country_buff, "JP", sizeof(country_buff) - 1);
  ssid_buff[0] = '\0';
  pass_buff[0] = '\0';
  strncpy(ntp_buff, "ntp.nict.jp", sizeof(ntp_buff) - 1);
  strncpy(timezone_buff, "0900", sizeof(timezone_buff) - 1);
  receiver.init(config_entries);
  const uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  t_next_sensor_read_ms = now_ms + 10;
}

bool recv_update(bool *success, Data &cfg) {
  const uint64_t now_ms = to_ms_since_boot(get_absolute_time());
  if (now_ms < t_next_sensor_read_ms) {
    return false;
  }

  t_next_sensor_read_ms += 10;

  vlcfg::RxState rx_state;
  vlcfg_err = receiver.update(adc_read(), &rx_state);
  if (vlcfg_err != vlcfg::Result::SUCCESS ||
      rx_state == vlcfg::RxState::ERROR) {
    printf("RX error: %d\n", static_cast<int>(vlcfg_err));
    if (success) *success = false;
    return true;
  } else if (rx_state == vlcfg::RxState::COMPLETED) {
    if (receiver.entry_from_key(KEY_COUNTRY)->was_received()) {
      strncpy(cfg.country, country_buff, COUNTRY_MAX_LEN);
      cfg.country[COUNTRY_MAX_LEN] = '\0';
    }
    if (receiver.entry_from_key(KEY_SSID)->was_received()) {
      strncpy(cfg.ssid, ssid_buff, SSID_MAX_LEN);
      cfg.ssid[SSID_MAX_LEN] = '\0';
    }
    if (receiver.entry_from_key(KEY_PASS)->was_received()) {
      strncpy(cfg.pass, pass_buff, PASS_MAX_LEN);
      cfg.pass[PASS_MAX_LEN] = '\0';
    }
    if (receiver.entry_from_key(KEY_NTP)->was_received()) {
      strncpy(cfg.ntp_host, ntp_buff, NTP_MAX_LEN);
      cfg.ntp_host[NTP_MAX_LEN] = '\0';
    }
    if (receiver.entry_from_key(KEY_TIMEZONE)->was_received()) {
      strncpy(cfg.timezone, timezone_buff, TIMEZONE_MAX_LEN);
      cfg.timezone[TIMEZONE_MAX_LEN] = '\0';
    }
    printf("RX completed successfully\n");
    printf("Country: '%s'\n", country_buff);
    printf("SSID: '%s'\n", ssid_buff);
    printf("PASS: '%s'\n", pass_buff);
    printf("NTP Server: '%s'\n", ntp_buff);
    printf("Timezone: '%s'\n", timezone_buff);

    if (eeprom_save_config(cfg)) {
      printf("Config saved to EEPROM\n");
    } else {
      printf("Failed to save config to EEPROM\n");
    }

    if (success) *success = true;

    return true;
  }

  return false;
}

bool is_receiving() {
  return receiver.decoder.get_state() == vlcfg::RxState::RECEIVING;
}

bool last_bit() { return receiver.get_last_bit(); }

vlcfg::Result last_vlcfg_error() { return vlcfg_err; }

static bool eeprom_load_config(Data &cfg) {
  bool success = false;
  uint8_t *buff = new uint8_t[EEPROM_CONFIG_SIZE];

  if (rom.read_data(0, buff, EEPROM_CONFIG_SIZE) != eeprom::result_t::SUCCESS) {
    printf("Failed to read config from EEPROM\n");
    goto failed;
  }

  {
    uint32_t stored_crc =
        ((uint32_t)(buff[EEPROM_CONFIG_CRC_OFFSET]) << 0) |
        ((uint32_t)(buff[EEPROM_CONFIG_CRC_OFFSET + 1]) << 8) |
        ((uint32_t)(buff[EEPROM_CONFIG_CRC_OFFSET + 2]) << 16) |
        ((uint32_t)(buff[EEPROM_CONFIG_CRC_OFFSET + 3]) << 24);
    uint32_t calc_crc = calc_crc32(buff, EEPROM_CONFIG_CRC_OFFSET);
    if (stored_crc != calc_crc) {
      printf("EEPROM config CRC mismatch: stored=0x%08lX, calc=0x%08lX\n",
             stored_crc, calc_crc);
      goto failed;
    }
  }

  strncpy(cfg.country, (char *)(buff + EEPROM_COUNTRY_OFFSET), COUNTRY_MAX_LEN);
  cfg.country[COUNTRY_MAX_LEN] = '\0';
  strncpy(cfg.ssid, (char *)(buff + EEPROM_SSID_OFFSET), SSID_MAX_LEN);
  cfg.ssid[SSID_MAX_LEN] = '\0';
  strncpy(cfg.pass, (char *)(buff + EEPROM_PASS_OFFSET), PASS_MAX_LEN);
  cfg.pass[PASS_MAX_LEN] = '\0';
  strncpy(cfg.ntp_host, (char *)(buff + EEPROM_NTP_OFFSET), NTP_MAX_LEN);
  cfg.ntp_host[NTP_MAX_LEN] = '\0';
  strncpy(cfg.timezone, (char *)(buff + EEPROM_TIMEZONE_OFFSET),
          TIMEZONE_MAX_LEN);
  cfg.timezone[TIMEZONE_MAX_LEN] = '\0';

  success = true;

failed:
  delete[] buff;
  return success;
}

static bool eeprom_save_config(const Data &cfg) {
  bool success = false;
  uint8_t *buff = new uint8_t[EEPROM_CONFIG_SIZE];

  memset(buff, 0, EEPROM_CONFIG_SIZE);
  memcpy(buff + EEPROM_COUNTRY_OFFSET, cfg.country, COUNTRY_MAX_LEN);
  memcpy(buff + EEPROM_SSID_OFFSET, cfg.ssid, SSID_MAX_LEN);
  memcpy(buff + EEPROM_PASS_OFFSET, cfg.pass, PASS_MAX_LEN);
  memcpy(buff + EEPROM_NTP_OFFSET, cfg.ntp_host, NTP_MAX_LEN);
  memcpy(buff + EEPROM_TIMEZONE_OFFSET, cfg.timezone, TIMEZONE_MAX_LEN);
  uint32_t crc = calc_crc32(buff, EEPROM_CONFIG_CRC_OFFSET);
  buff[EEPROM_CONFIG_CRC_OFFSET + 0] = (crc >> 0) & 0xFF;
  buff[EEPROM_CONFIG_CRC_OFFSET + 1] = (crc >> 8) & 0xFF;
  buff[EEPROM_CONFIG_CRC_OFFSET + 2] = (crc >> 16) & 0xFF;
  buff[EEPROM_CONFIG_CRC_OFFSET + 3] = (crc >> 24) & 0xFF;

  if (rom.write_data(0, buff, EEPROM_CONFIG_SIZE) !=
      eeprom::result_t::SUCCESS) {
    printf("Failed to write config to EEPROM\n");
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

#endif

}  // namespace config

#endif
