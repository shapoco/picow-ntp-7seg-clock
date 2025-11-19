#ifndef EEPROM_HPP
#define EEPROM_HPP

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

#include <cstdint>
#include <cstring>

#include "common.hpp"

namespace eeprom {

enum class result_t {
  SUCCESS,
  ADDR_OUT_OF_BOUNDS,
  I2C_ACCESS_FAILED,
};

using addr_t = uint32_t;

class Driver {
 public:
  i2c_inst_t* i2c;
  const int sda_port;
  const int scl_port;
  const addr_t size_bytes;
  const addr_t page_bytes;
  const uint8_t dev_addr;
  const uint32_t clock_hz;
  const uint32_t write_cycle_time_ms = 5;

  Driver(int i2c_host, int sda_port, int scl_port, addr_t size_bytes,
         addr_t page_size, uint8_t dev_addr = 0x50, uint32_t clock_hz = 100000,
         uint32_t write_cycle_time_ms = 5)
      : i2c(i2c_host == 0 ? i2c0 : i2c1),
        sda_port(sda_port),
        scl_port(scl_port),
        size_bytes(size_bytes),
        page_bytes(page_size),
        dev_addr(dev_addr),
        clock_hz(clock_hz),
        write_cycle_time_ms(write_cycle_time_ms) {}

  void init() {}

  result_t read_data(addr_t mem_addr, uint8_t* buffer, addr_t length) {
    if (mem_addr + length > size_bytes) {
      NTPC_VERBOSE("EEPROM read addr out of bounds: addr=0x%04lX, length=%lu\n",
                   mem_addr, length);
      return result_t::ADDR_OUT_OF_BOUNDS;
    }

    uint8_t addr_buf[2];
    addr_buf[0] = (mem_addr >> 8) & 0xFF;
    addr_buf[1] = mem_addr & 0xFF;

    int ret = i2c_write_blocking(i2c, dev_addr, addr_buf, 2, true);
    if (ret != 2) {
      NTPC_VERBOSE("EEPROM read i2c_write_blocking failed: ret=%d\n", ret);
      return result_t::I2C_ACCESS_FAILED;
    }

    ret = i2c_read_blocking(i2c, dev_addr, buffer, length, false);
    if (ret != (int)length) {
      NTPC_VERBOSE("EEPROM read i2c_read_blocking failed: ret=%d\n", ret);
      return result_t::I2C_ACCESS_FAILED;
    }

    if (ntpc::debug_mode) {
      NTPC_VERBOSE("EEPROM read successful: addr=0x%04lX, length=%lu\n",
                   mem_addr, length);
      for (addr_t i = 0; i < length; i++) {
        if (i % 16 == 0) {
          printf("\n0x%04lX: ", mem_addr + i);
        }
        printf("%02X ", buffer[i]);
      }
      printf("\n");
    }

    return result_t::SUCCESS;
  }

  result_t write_data(addr_t mem_addr, const uint8_t* data, addr_t length) {
    result_t res = result_t::SUCCESS;
    uint8_t* write_buf = new uint8_t[page_bytes];
    addr_t offset_mask = page_bytes - 1;

    if (ntpc::debug_mode) {
      NTPC_VERBOSE("EEPROM write: addr=0x%04lX, length=%lu\n", mem_addr,
                   length);
      for (addr_t i = 0; i < length; i++) {
        if (i % 16 == 0) {
          printf("\n0x%04lX: ", mem_addr + i);
        }
        printf("%02X ", data[i]);
      }
      printf("\n");
    }

    if (mem_addr + length > size_bytes) {
      NTPC_VERBOSE(
          "EEPROM write addr out of bounds: addr=0x%04lX, length=%lu\n",
          mem_addr, length);
      res = result_t::ADDR_OUT_OF_BOUNDS;
      goto failed;
    }

    while (length > 0) {
      addr_t page_offset = mem_addr & offset_mask;
      addr_t space_in_page = page_bytes - page_offset;
      addr_t bytes_to_write = (length < space_in_page) ? length : space_in_page;

      write_buf[0] = (mem_addr >> 8) & 0xFF;
      write_buf[1] = mem_addr & 0xFF;
      memcpy(write_buf + 2, data, bytes_to_write);
      int ret = i2c_write_blocking(i2c, dev_addr, write_buf, 2 + bytes_to_write,
                                   false);
      if (ret != (int)(2 + bytes_to_write)) {
        NTPC_VERBOSE("EEPROM write i2c_write_blocking failed: ret=%d\n", ret);
        res = result_t::I2C_ACCESS_FAILED;
        goto failed;
      }

      mem_addr += bytes_to_write;
      data += bytes_to_write;
      length -= bytes_to_write;
      sleep_ms(5);  // Wait for EEPROM write cycle to complete
    }
  failed:
    delete[] write_buf;

    return res;
  }
};
}  // namespace eeprom
#endif
