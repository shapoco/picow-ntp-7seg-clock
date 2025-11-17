#ifndef rx8025nb_HPP
#define rx8025nb_HPP

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

#include <cstdint>
#include <cstring>
#include <ctime>

#include "common.hpp"

namespace rx8025nb {

enum class result_t {
  SUCCESS,
  I2C_ACCESS_FAILED,
};

enum class reg_t : uint8_t {
  SECONDS = 0x00,
  MINUTES = 0x01,
  HOURS = 0x02,
  WEEKDAYS = 0x03,
  DAYS = 0x04,
  MONTHS = 0x05,
  YEARS = 0x06,
  DIGITAL_OFFSET = 0x07,
  ALARM_W_MINUTE = 0x08,
  ALARM_W_HOUR = 0x09,
  ALARM_W_WEEKDAY = 0x0A,
  ALARM_D_MINUTE = 0x0B,
  ALARM_D_HOUR = 0x0C,
  RESERVED_1 = 0x0D,
  CONTROL_1 = 0x0E,
  CONTROL_2 = 0x0F,
  NUM_REGISTERS = 0x10,
};

static constexpr uint8_t MASK_CONTROL1_H24 = 0x20;
static constexpr uint8_t MASK_CONTROL2_PON = 0x10;

static constexpr int I2C_INTERVAL_US = 100;

static const uint8_t INIT_SEQUENCE[] = {
    static_cast<uint8_t>(reg_t::SECONDS),
    0x00,
    static_cast<uint8_t>(reg_t::MINUTES),
    0x00,
    static_cast<uint8_t>(reg_t::HOURS),
    0x00,
    static_cast<uint8_t>(reg_t::WEEKDAYS),
    0x00,
    static_cast<uint8_t>(reg_t::DAYS),
    0x01,
    static_cast<uint8_t>(reg_t::MONTHS),
    0x01,
    static_cast<uint8_t>(reg_t::YEARS),
    0x00,
    static_cast<uint8_t>(reg_t::DIGITAL_OFFSET),
    0x00,
    static_cast<uint8_t>(reg_t::ALARM_W_MINUTE),
    0x00,
    static_cast<uint8_t>(reg_t::ALARM_W_HOUR),
    0x00,
    static_cast<uint8_t>(reg_t::ALARM_W_WEEKDAY),
    0x00,
    static_cast<uint8_t>(reg_t::ALARM_D_MINUTE),
    0x00,
    static_cast<uint8_t>(reg_t::ALARM_D_HOUR),
    0x00,
    static_cast<uint8_t>(reg_t::CONTROL_1),
    MASK_CONTROL1_H24,
    static_cast<uint8_t>(reg_t::CONTROL_2),
    0x00,
};

static uint8_t from_bcd(uint8_t bcd, uint8_t mask) {
  bcd &= mask;
  return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t to_bcd(uint8_t value) {
  return ((value / 10) << 4) | (value % 10);
}

class Driver {
 public:
  i2c_inst_t* i2c;
  const int sda_port;
  const int scl_port;
  const uint8_t dev_addr;
  const uint32_t clock_hz;

  Driver(int i2c_host, int sda_port, int scl_port, uint8_t dev_addr = 0x32,
         uint32_t clock_hz = 100000)
      : i2c(i2c_host == 0 ? i2c0 : i2c1),
        sda_port(sda_port),
        scl_port(scl_port),
        dev_addr(dev_addr),
        clock_hz(clock_hz) {}

  result_t init() {
    result_t res;
    uint8_t control2 = 0;
    res = read_reg(reg_t::CONTROL_2, &control2);
    if (res != result_t::SUCCESS) {
      NTPC_PRINTF("RTC init failed.\r\n");
      return res;
    }

    if ((control2 & MASK_CONTROL2_PON) != 0) {
      NTPC_PRINTF("RTC power-on detected, initializing registers.\r\n");
      // Power-on sequence
      int init_len = sizeof(INIT_SEQUENCE) / sizeof(INIT_SEQUENCE[0]);
      for (int i = 0; i < init_len; i += 2) {
        reg_t reg = static_cast<reg_t>(INIT_SEQUENCE[i]);
        uint8_t value = INIT_SEQUENCE[i + 1];
        res = write_reg(reg, &value);
        if (res != result_t::SUCCESS) {
          NTPC_PRINTF("RTC write_reg failed: reg=0x%02X, value=0x%02X\n",
                      static_cast<uint8_t>(reg), value);
          return res;
        }
      }
    }

    NTPC_PRINTF("RTC initialized\n");

    return result_t::SUCCESS;
  }

  result_t get_time(uint64_t* time) {
    const int size =
        static_cast<int>(reg_t::YEARS) + 1 - static_cast<int>(reg_t::SECONDS);
    uint8_t bcd[size];
    result_t res = read_reg(reg_t::SECONDS, bcd, size);
    if (res != result_t::SUCCESS) {
      NTPC_PRINTF("RTC get_time read_reg failed\n");
      return res;
    }

    struct tm t;
    t.tm_sec = from_bcd(bcd[0], 0x7F);
    t.tm_min = from_bcd(bcd[1], 0x7F);
    t.tm_hour = from_bcd(bcd[2], 0x3F);
    t.tm_mday = from_bcd(bcd[4], 0x3F);
    t.tm_mon = from_bcd(bcd[5], 0x1F) - 1;
    t.tm_year = from_bcd(bcd[6], 0xFF) + 100;
    t.tm_isdst = -1;

    NTPC_PRINTF("RTC time fetched: %04d-%02d-%02d %02d:%02d:%02d\n",
                t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min,
                t.tm_sec);

    time_t epoch_time = mktime(&t);
    *time = static_cast<uint64_t>(epoch_time);
    return result_t::SUCCESS;
  }

  result_t set_time(uint64_t time) {
    const int size =
        static_cast<int>(reg_t::YEARS) + 1 - static_cast<int>(reg_t::SECONDS);
    struct tm* t = gmtime(reinterpret_cast<time_t*>(&time));

    uint8_t bcd[size];
    bcd[0] = to_bcd(t->tm_sec);
    bcd[1] = to_bcd(t->tm_min);
    bcd[2] = to_bcd(t->tm_hour);
    bcd[3] = to_bcd(t->tm_wday + 1);
    bcd[4] = to_bcd(t->tm_mday);
    bcd[5] = to_bcd(t->tm_mon + 1);
    bcd[6] = to_bcd(t->tm_year % 100);

    result_t res = write_reg(reg_t::SECONDS, bcd, size);
    if (res != result_t::SUCCESS) {
      NTPC_PRINTF("RTC set_time write_reg failed\n");
      return res;
    }

    NTPC_PRINTF("RTC time set to %04d-%02d-%02d %02d:%02d:%02d\n",
                t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour,
                t->tm_min, t->tm_sec);
    return result_t::SUCCESS;
  }

  result_t write_reg(reg_t addr, uint8_t* value, int size = 1) {
    int ret = 0;
    uint8_t buf[1 + size];
    buf[0] = static_cast<uint8_t>(addr) << 4;
    memcpy(&buf[1], value, size);
    ret = i2c_write_blocking(i2c, dev_addr, buf, 1 + size, false);
    sleep_us(I2C_INTERVAL_US);
    if (ret != 1 + size) {
      NTPC_PRINTF(
          "RTC write_reg i2c_write_blocking failed: addr=0x%02X, ret=%d\n",
          static_cast<int>(addr), ret);
      return result_t::I2C_ACCESS_FAILED;
    }
    return result_t::SUCCESS;
  }

  result_t read_reg(reg_t addr, uint8_t* value, int size = 1) {
    int ret = 0;
    uint8_t reg = static_cast<uint8_t>(addr) << 4;
    ret = i2c_write_blocking(i2c, dev_addr, &reg, 1, true);
    sleep_us(I2C_INTERVAL_US);
    if (ret != 1) {
      NTPC_PRINTF(
          "RTC read_reg i2c_write_blocking failed: addr=0x%02X, ret=%d\n",
          static_cast<int>(addr), ret);
      return result_t::I2C_ACCESS_FAILED;
    }
    ret = i2c_read_blocking(i2c, dev_addr, value, size, false);
    sleep_us(I2C_INTERVAL_US);
    if (ret != size) {
      NTPC_PRINTF(
          "RTC read_reg i2c_read_blocking failed: addr=0x%02X, ret=%d\n",
          static_cast<int>(addr), ret);
      return result_t::I2C_ACCESS_FAILED;
    }
    return result_t::SUCCESS;
  }
};
}  // namespace rx8025nb
#endif
