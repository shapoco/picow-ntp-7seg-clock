#include <math.h>
#include <stdio.h>
#include <time.h>
#include <cstring>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <pico/stdlib.h>

#include "seg7_pwm_tx.pio.h"

static constexpr int DIGIT_PORTS[] = {0, 1, 2, 3,  4,  5,  6,
                                      7, 8, 9, 10, 11, 12, 13};
static constexpr int DIGIT_PORT_BASE = 0;

static constexpr int SEGMENT_PORTS[] = {20, 21, 14, 16, 17, 19, 18, 15};
static constexpr int SEGMENT_PORT_BASE = 14;

static constexpr uint8_t SEGMENT_TABLE[] = {
    0b00111111,  // 0
    0b00000110,  // 1
    0b01011011,  // 2
    0b01001111,  // 3
    0b01100110,  // 4
    0b01101101,  // 5
    0b01111101,  // 6
    0b00000111,  // 7
    0b01111111,  // 8
    0b01101111,  // 9
    0b01110111,  // A
    0b01111100,  // b
    0b00111001,  // C
    0b01011110,  // d
    0b01111001,  // E
    0b01110001,  // F
};

static constexpr int NUM_DIGITS = 14;
static constexpr int NUM_SEGMENTS = 8;
static constexpr int PWM_PREC = 10;
static constexpr int PWM_PERIOD = 1 << PWM_PREC;

static constexpr int FADE_TIME_MS = 150;
static constexpr int FADE_DELAY_TABLE[] = {
    FADE_TIME_MS * 0 / 60,   // a
    FADE_TIME_MS * 10 / 60,  // b
    FADE_TIME_MS * 40 / 60,  // c
    FADE_TIME_MS * 60 / 60,  // d
    FADE_TIME_MS * 50 / 60,  // e
    FADE_TIME_MS * 20 / 60,  // f
    FADE_TIME_MS * 30 / 60,  // g
    FADE_TIME_MS * 55 / 60,  // dp
};
static constexpr int DIGIT_DELAY_MS = (1000 - FADE_TIME_MS * 3) / NUM_DIGITS;

static constexpr int SYSTEM_CLOCK_HZ = 125000000;
// static constexpr int SYSTEM_CLOCK_HZ = 250000000;

static constexpr int FRAME_RATE = 120;
static constexpr int ANODE_SEL_HZ = FRAME_RATE * NUM_DIGITS;
static constexpr int ANODE_SEL_PERIOD_US = 1000000 / ANODE_SEL_HZ;
static constexpr int DMA_TX_HZ = ANODE_SEL_HZ * PWM_PERIOD * 11 / 10;
static constexpr int PIO_CLKDIV = SYSTEM_CLOCK_HZ / DMA_TX_HZ;

uint8_t digits0[NUM_DIGITS] = {0};
uint8_t digits1[NUM_DIGITS] = {0};
uint8_t dram0[NUM_DIGITS * NUM_SEGMENTS] = {0};
uint8_t dram1[NUM_DIGITS * NUM_SEGMENTS] = {0};
uint8_t dram2[NUM_DIGITS * NUM_SEGMENTS] = {0};
uint8_t dma_buff[NUM_DIGITS * PWM_PERIOD] = {0};
int digit_sel = 0;

int transition_num_digits = 0;
uint64_t transition_start_ms = 0;

repeating_timer_t st_timer;
int dma_chan;

uint64_t epoch_tick_ms;
time_t epoch_time;

bool repeating_timer_callback(repeating_timer_t *rt);
void print_digit(int index, int width, int value);

int main() {
  set_sys_clock_khz(SYSTEM_CLOCK_HZ / 1000, true);
  sleep_ms(100);

  // Initialize stdio
  stdio_init_all();
  sleep_ms(100);

  // Define the GPIO pin number for the LED
  // const uint LED_PIN = 25;

  // epoch_time = 1767193185 + 32400;  // 2025-12-31 23:59:45+09:00
  epoch_time = 1735657185 + 32400;  // 2024-12-31 23:59:45+09:00
  epoch_tick_ms = to_ms_since_boot(get_absolute_time());

  for (int port : DIGIT_PORTS) {
    gpio_init(port);
    gpio_set_dir(port, GPIO_OUT);
    gpio_put(port, false);
    gpio_drive_strength(GPIO_DRIVE_STRENGTH_12MA);
  }

#if 0
  for (int port : SEGMENT_PORTS) {
    gpio_init(port);
    gpio_set_dir(port, GPIO_OUT);
    gpio_put(port, true);
  }
#else
  // Set up a PIO state machine to serialise our bits
  uint offset = pio_add_program(pio0, &seg7_pwm_tx_program);
  seg7_pwm_tx_program_init(pio0, 0, offset, SEGMENT_PORT_BASE, PIO_CLKDIV);

  // Configure a channel to write the same word (32 bits) repeatedly to PIO0
  // SM0's TX FIFO, paced by the data request signal from that peripheral.
  dma_chan = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_PIO0_TX0);

  dma_channel_configure(dma_chan, &c, &pio0_hw->txf[0], NULL, PWM_PERIOD,
                        false);
#endif

  // Tell the DMA to raise IRQ line 0 when the channel finishes a block
  // dma_channel_set_irq0_enabled(dma_chan, true);

  add_repeating_timer_us(-ANODE_SEL_PERIOD_US, repeating_timer_callback, NULL,
                         &st_timer);

  while (true) {
    const uint64_t nowMs = to_ms_since_boot(get_absolute_time());

    const int64_t elapsedMs = (nowMs - epoch_tick_ms);
    const time_t now = epoch_time + ((elapsedMs + FADE_TIME_MS * 2) / 1000);

    long timezone_offset = -9 * 3600;
    struct tm *t = gmtime(&now);

    int sec = t->tm_sec;
    int min = t->tm_min;
    int hour = t->tm_hour;
    int day = t->tm_mday;
    int mon = t->tm_mon + 1;
    int year = t->tm_year + 1900;
    int millisec = elapsedMs % 1000;

    print_digit(0, 2, sec);
    print_digit(2, 2, min);
    print_digit(4, 2, hour);
    print_digit(6, 2, day);
    print_digit(8, 2, mon);
    print_digit(10, 4, year);

    if (transition_num_digits == 0) {
      bool digit_changed = false;
      for (int idig = NUM_DIGITS - 1; idig >= 0; idig--) {
        if (digits0[idig] != digits1[idig]) {
          digit_changed = true;
          transition_num_digits = idig + 1;
          transition_start_ms = nowMs;
          break;
        }
      }
      memcpy(digits1, digits0, sizeof(digits0));

      for (int idig = 0; idig < NUM_DIGITS; idig++) {
        uint8_t seg_bits = SEGMENT_TABLE[digits1[idig] & 0xF];
        for (int iseg = 0; iseg < NUM_SEGMENTS - 1; iseg++) {
          dram0[idig * NUM_SEGMENTS + iseg] = (seg_bits & 1) ? 255 : 0;
          seg_bits >>= 1;
        }
      }

      dram0[8 * NUM_SEGMENTS + 7] = 96;
      dram0[10 * NUM_SEGMENTS + 7] = 96;

    } else {
      uint32_t t = nowMs - transition_start_ms;

      for (int idig = 0; idig < NUM_DIGITS; idig++) {
        if (idig < transition_num_digits) {
          uint32_t t_dig_offset = idig * DIGIT_DELAY_MS + FADE_TIME_MS;
          for (int iseg = 0; iseg < NUM_SEGMENTS; iseg++) {
            uint32_t t_seg_offset = t_dig_offset + FADE_DELAY_TABLE[iseg];
            int idx = idig * NUM_SEGMENTS + iseg;
            if (t <= t_seg_offset - FADE_TIME_MS) {
              dram2[idx] = dram1[idx];
            } else if (t <= t_seg_offset) {
              dram2[idx] = dram1[idx] * (t_seg_offset - t) / FADE_TIME_MS;
            } else if (t <= t_seg_offset + FADE_TIME_MS) {
              dram2[idx] = dram0[idx] * (t - t_seg_offset) / FADE_TIME_MS;
            } else {
              dram2[idx] = dram0[idx];
            }
          }
        } else {
          for (int iseg = 0; iseg < NUM_SEGMENTS - 1; iseg++) {
            int idx = idig * NUM_SEGMENTS + iseg;
            dram2[idx] = dram0[idx];
          }
        }
      }

      if (t >= transition_num_digits * DIGIT_DELAY_MS + FADE_TIME_MS * 3) {
        memcpy(dram1, dram0, sizeof(dram0));
        transition_num_digits = 0;
      }
    }

    {
      constexpr int FADEIN_MS = 100;
      constexpr int FADEOUT_MS = 500 - FADEIN_MS;
      int alpha;
      if (millisec < 500) {
        alpha = 255;
      } else if (millisec < 1000 - FADEIN_MS) {
        alpha = 256 * (FADEOUT_MS - (millisec - 500)) / FADEOUT_MS;
        alpha = alpha * alpha / 256;
      } else {
        alpha = 256 - (256 * (millisec - (1000 - FADEIN_MS)) / FADEIN_MS);
        alpha = 256 - (alpha * alpha) / 256;
      }
      dram2[2 * NUM_SEGMENTS + 7] = 128 * alpha / 256;
      dram2[4 * NUM_SEGMENTS + 7] = 128 * alpha / 256;
    }

    for (int d = 0; d < NUM_DIGITS; d++) {
      uint16_t thresh[NUM_SEGMENTS] = {0};
      for (int s = 0; s < NUM_SEGMENTS; s++) {
        uint32_t val = dram2[d * NUM_SEGMENTS + s];
#if 0
        val = val * val * val;
        val /= (255 * 255 * 255) / (PWM_PERIOD - 1);
#else
        val = val * val;
        val = val * (PWM_PERIOD - 1) / (255 * 255);
#endif
        thresh[s] = val;
      }

      int ofst = d * PWM_PERIOD;
      for (int t = 0; t < PWM_PERIOD; t++) {
        uint8_t seg_bits = 0;
        for (int iseg = 0; iseg < NUM_SEGMENTS; iseg++) {
          if (t < thresh[iseg]) {
            int iport = SEGMENT_PORTS[iseg] - SEGMENT_PORT_BASE;
            seg_bits |= (1 << iport);
          }
        }
        dma_buff[ofst + t] = ~seg_bits;
      }
    }

    sleep_ms(5);
  }

  return 0;
}

void print_digit(int index, int width, int value) {
  uint8_t *wptr = digits0 + index;
  for (int i = 0; i < width; i++) {
    *(wptr++) = value % 10;
    value /= 10;
  }
}

// タイマ割り込みハンドラ
bool repeating_timer_callback(repeating_timer_t *rt) {
  constexpr uint32_t ANODE_PORT_MASK = ((1 << NUM_DIGITS) - 1)
                                       << DIGIT_PORT_BASE;
#if 0
  for (int iseg = 0; iseg < NUM_SEGMENTS; iseg++) {
    uint8_t val = dram[digit_sel * NUM_SEGMENTS + iseg];
    gpio_put(SEGMENT_PORTS[iseg], val >= 128);
  }
  gpio_put_masked(ANODE_PORT_MASK, (1 << digit_sel));
  digit_sel = (digit_sel + 1) % NUM_DIGITS;
#else
  if (!dma_channel_is_busy(dma_chan)) {
    gpio_put_masked(ANODE_PORT_MASK, (1 << digit_sel));
    uint8_t *read_addr = dma_buff + (digit_sel * PWM_PERIOD);
    if (true) {
      dma_channel_set_read_addr(dma_chan, read_addr, true);
    } else {
      dma_channel_config c = dma_channel_get_default_config(dma_chan);
      channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
      channel_config_set_write_increment(&c, false);
      channel_config_set_read_increment(&c, true);
      channel_config_set_dreq(&c, DREQ_PIO0_TX0);
      dma_channel_configure(dma_chan, &c, &pio0_hw->txf[0], read_addr,
                            PWM_PERIOD, true);
    }
    digit_sel = (digit_sel + 1) % NUM_DIGITS;
  }
#endif
  return true;
}
