#include <math.h>
#include <stdio.h>
#include <time.h>
#include <cstring>

#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <pico/stdlib.h>

#include <vlcfg/vlconfig.hpp>

#include "button.hpp"
#include "ntp_client.hpp"

#include "seg7_pwm_tx.pio.h"

enum class State {
  IDLE,
  SETUP,
};

static constexpr int SETUP_SW_PORT = 22;
static constexpr int LIGHT_SENSOR_ADC_CH = 2;

static constexpr int DIGIT_PORTS[] = {0, 1, 2, 3,  4,  5,  6,
                                      7, 8, 9, 10, 11, 12, 13};
static constexpr int DIGIT_PORT_BASE = 0;

static constexpr int SEGMENT_PORTS[] = {20, 21, 14, 16, 17, 19, 18, 15};
static constexpr int SEGMENT_PORT_BASE = 14;

static constexpr uint8_t DIGIT_BLANK = 0x10;
static constexpr uint8_t DIGIT_HYPHEN = 0x11;

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
    0b00000000,  // blank
    0b01000000,  // hyphen
    0b00000000,  // 0x12
    0b00000000,  // 0x13
    0b00000000,  // 0x14
    0b00000000,  // 0x15
    0b00000000,  // 0x16
    0b00000000,  // 0x17
    0b00000000,  // 0x18
    0b00000000,  // 0x19
    0b00000000,  // 0x1A
    0b00000000,  // 0x1B
    0b00000000,  // 0x1C
    0b00000000,  // 0x1D
    0b00000000,  // 0x1E
    0b00000000,  // 0x1F
};

static constexpr int COLON_COEFF = 128;
static constexpr int HYPHEN_COEFF = 96;

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
time_t epoch_sec = 0;
int epoch_ms = 0;

Button setup_button(SETUP_SW_PORT);

const char *KEY_SSID = "s";
const char *KEY_PASS = "p";
const char *KEY_NTP = "n";
const char *KEY_TIMEZONE = "z";
char ssid_buff[32 + 1];
char pass_buff[32 + 1];
char ntp_buff[64 + 1];
char tz_buff[8 + 1] = "0900";
vlcfg::ConfigEntry config_entries[] = {
    {KEY_SSID, ssid_buff, vlcfg::ValueType::TEXT_STR, sizeof(ssid_buff)},
    {KEY_PASS, pass_buff, vlcfg::ValueType::TEXT_STR, sizeof(pass_buff)},
    {KEY_NTP, ntp_buff, vlcfg::ValueType::TEXT_STR, sizeof(ntp_buff)},
    {KEY_TIMEZONE, tz_buff, vlcfg::ValueType::TEXT_STR, sizeof(tz_buff)},
    {nullptr, nullptr, vlcfg::ValueType::NONE, 0},  // terminator
};
vlcfg::Receiver receiver(256);

State state = State::IDLE;

void update_display(uint64_t nowMs);
void print_digit(int index, int width, int value);
bool repeating_timer_callback(repeating_timer_t *rt);

void sync_with_ntp();

int main() {
  set_sys_clock_khz(SYSTEM_CLOCK_HZ / 1000, true);
  sleep_ms(100);

  // Initialize stdio
  stdio_init_all();
  sleep_ms(100);

  // Define the GPIO pin number for the LED
  // const uint LED_PIN = 25;

  // epoch_time = 1767193185 + 32400;  // 2025-12-31 23:59:45+09:00
  // epoch_time = 1735657185 + 32400;  // 2024-12-31 23:59:45+09:00
  // epoch_time = 0;  // 2024-12-31 23:59:45+09:00

  setup_button.init();

  adc_init();
  adc_gpio_init(26 + LIGHT_SENSOR_ADC_CH);
  adc_select_input(LIGHT_SENSOR_ADC_CH);

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

  uint64_t t_next_update_ms = 0;
  uint64_t t_next_sensor_read_ms = 0;

  while (true) {
    const uint64_t now_ms = to_ms_since_boot(get_absolute_time());

    setup_button.update();

    switch (state) {
      case State::IDLE:
        if (setup_button.on_clicked()) {
          state = State::SETUP;
          receiver.init(config_entries);
          t_next_sensor_read_ms = now_ms + 10;
          printf("Enter SETUP state\n");
        }
        break;

      case State::SETUP:
        if (now_ms >= t_next_sensor_read_ms) {
          t_next_sensor_read_ms += 10;
          vlcfg::RxState rx_state;
          auto ret = receiver.update(adc_read(), &rx_state);
          if (ret != vlcfg::Result::SUCCESS ||
              rx_state == vlcfg::RxState::ERROR) {
            printf("RX error: %d\n", static_cast<int>(ret));
            state = State::IDLE;
          } else if (rx_state == vlcfg::RxState::COMPLETED) {
            bool filled = receiver.entry_from_key(KEY_SSID)->was_received() |
                          receiver.entry_from_key(KEY_PASS)->was_received() |
                          receiver.entry_from_key(KEY_NTP)->was_received();

            if (filled) {
              printf("RX completed successfully\n");
              printf("SSID: '%s'\n", ssid_buff);
              printf("PASS: '%s'\n", pass_buff);
              printf("NTP Server: '%s'\n", ntp_buff);
              sync_with_ntp();
            } else {
              printf("RX completed but some fields are empty\n");
            }
            state = State::IDLE;
          }
        } else if (setup_button.on_clicked()) {
          state = State::IDLE;
          printf("Return to IDLE state\n");
        }
        break;
    }

    if (now_ms >= t_next_update_ms) {
      update_display(now_ms);
      t_next_update_ms = now_ms + 5;
    }

    sleep_us(100);
  }

  return 0;
}

void update_display(uint64_t now_ms) {
  const int64_t elapsed_ms = (now_ms + epoch_ms - epoch_tick_ms);
  const time_t now = epoch_sec + ((elapsed_ms + FADE_TIME_MS * 2) / 1000);

  long timezone_offset = -9 * 3600;
  struct tm *t = gmtime(&now);

  int sec = t->tm_sec;
  int min = t->tm_min;
  int hour = t->tm_hour;
  int day = t->tm_mday;
  int mon = t->tm_mon + 1;
  int year = t->tm_year + 1900;
  int millisec = elapsed_ms % 1000;

  int blink_alpha;
  {
    constexpr int FADEIN_MS = 100;
    constexpr int FADEOUT_MS = 500 - FADEIN_MS;
    if (millisec < 500) {
      blink_alpha = 255;
    } else if (millisec < 1000 - FADEIN_MS) {
      blink_alpha = 256 * (FADEOUT_MS - (millisec - 500)) / FADEOUT_MS;
      blink_alpha = blink_alpha * blink_alpha / 256;
    } else {
      blink_alpha = 256 - (256 * (millisec - (1000 - FADEIN_MS)) / FADEIN_MS);
      blink_alpha = 256 - (blink_alpha * blink_alpha) / 256;
    }
    if (blink_alpha < 0) blink_alpha = 0;
    if (blink_alpha > 255) blink_alpha = 255;
  }

  if (epoch_sec != 0) {
    print_digit(0, 2, sec);
    print_digit(2, 2, min);
    print_digit(4, 2, hour);
    print_digit(6, 2, day);
    print_digit(8, 2, mon);
    print_digit(10, 4, year);

    if (transition_num_digits == 0) {
      // 数字の変化点検出
      bool digit_changed = false;
      for (int idig = NUM_DIGITS - 1; idig >= 0; idig--) {
        if (digits0[idig] != digits1[idig]) {
          digit_changed = true;
          transition_num_digits = idig + 1;
          transition_start_ms = now_ms;
          break;
        }
      }
      memcpy(digits1, digits0, sizeof(digits0));

      // セグメントにデコード
      for (int idig = 0; idig < NUM_DIGITS; idig++) {
        uint8_t seg_bits = SEGMENT_TABLE[digits1[idig] & 0x1F];
        for (int iseg = 0; iseg < NUM_SEGMENTS - 1; iseg++) {
          dram0[idig * NUM_SEGMENTS + iseg] = (seg_bits & 1) ? 255 : 0;
          seg_bits >>= 1;
        }
      }

      // 日付のハイフン点灯
      dram0[8 * NUM_SEGMENTS + 7] = HYPHEN_COEFF;
      dram0[10 * NUM_SEGMENTS + 7] = HYPHEN_COEFF;

    } else {
      uint32_t t = now_ms - transition_start_ms;

      // アニメーション
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

      // アニメーション完了
      if (t >= transition_num_digits * DIGIT_DELAY_MS + FADE_TIME_MS * 3) {
        memcpy(dram1, dram0, sizeof(dram0));
        transition_num_digits = 0;
      }
    }

    // コロンの点滅
    dram2[2 * NUM_SEGMENTS + 7] = 128 * blink_alpha / 256;
    dram2[4 * NUM_SEGMENTS + 7] = 128 * blink_alpha / 256;
  } else {
    // 時刻未設定時の表示
    for (int idig = 0; idig < NUM_DIGITS; idig++) {
      digits0[idig] = DIGIT_HYPHEN;
    }

    if (state == State::SETUP) {
      uint8_t hex = receiver.get_last_byte();
      digits0[6] = hex & 0x0F;
      digits0[7] = (hex >> 4) & 0x0F;
    }

    memcpy(digits1, digits0, sizeof(digits0));

    for (int idig = 0; idig < NUM_DIGITS; idig++) {
      uint8_t seg_bits = SEGMENT_TABLE[digits1[idig] & 0x1F];
      for (int iseg = 0; iseg < NUM_SEGMENTS - 1; iseg++) {
        dram0[idig * NUM_SEGMENTS + iseg] = (seg_bits & 1) ? blink_alpha : 0;
        seg_bits >>= 1;
      }
    }
    dram0[2 * NUM_SEGMENTS + 7] = COLON_COEFF * blink_alpha / 256;
    dram0[4 * NUM_SEGMENTS + 7] = COLON_COEFF * blink_alpha / 256;
    dram0[8 * NUM_SEGMENTS + 7] = HYPHEN_COEFF * blink_alpha / 256;
    dram0[10 * NUM_SEGMENTS + 7] = HYPHEN_COEFF * blink_alpha / 256;

    if (state == State::SETUP) {
      dram0[6 * NUM_SEGMENTS + 7] = receiver.get_last_bit() ? 255 : 0;
      dram0[7 * NUM_SEGMENTS + 7] = 255;
    }

    memcpy(dram1, dram0, sizeof(dram0));
    memcpy(dram2, dram0, sizeof(dram0));
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

void sync_with_ntp() {
  for (int idig = 0; idig < NUM_DIGITS; idig++) {
    for (int iseg = 0; iseg < NUM_SEGMENTS; iseg++) {
      dram0[idig * NUM_SEGMENTS + iseg] = 0;
      dram1[idig * NUM_SEGMENTS + iseg] = 0;
      dram2[idig * NUM_SEGMENTS + iseg] = 0;
    }
  }
  for (int i = 0; i < NUM_DIGITS * PWM_PERIOD; i++) {
    dma_buff[i] = 0xFF;
  }
  ntp::WiFiConfig wifi_config = {
      .country = CYW43_COUNTRY_JAPAN,
      .ssid = ssid_buff,
      .password = pass_buff,
  };
  uint64_t time;
  auto res = ntp::get_time(wifi_config, ntp_buff, &time);
  if (res == ntp::result_t::SUCCESS) {
    int tz = atoi(tz_buff);
    int tz_hour = tz / 100;
    int tz_min = tz % 100;
    time += (uint64_t)(tz_hour * 3600 + tz_min * 60) * 1000;

    epoch_sec = (time / 1000);
    epoch_ms = (time % 1000);
    epoch_tick_ms = to_ms_since_boot(get_absolute_time());
  } else {
    printf("NTP time fetch failed: %d\n", static_cast<int>(res));
  }
}