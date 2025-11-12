#include <hardware/dma.h>
#include <pico/stdlib.h>
#include <ctime>

#include "seg7_pwm_tx.pio.h"

#include "display.hpp"

namespace display {

using namespace ntpc;

static constexpr int DIGIT_PORTS[] = {0, 1, 2, 3,  4,  5,  6,
                                      7, 8, 9, 10, 11, 12, 13};
static constexpr int DIGIT_PORT_BASE = 0;

static constexpr int SEGMENT_PORTS[] = {20, 21, 14, 16, 17, 19, 18, 15};
static constexpr int SEGMENT_PORT_BASE = 14;

static constexpr int COLON_COEFF = 128;
static constexpr int HYPHEN_COEFF = 96;

static constexpr int NUM_DIGITS = 14;
static constexpr int NUM_SEGMENTS = 8;
static constexpr int PWM_PREC = 8;
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

static constexpr int FRAME_RATE = 120;
static constexpr int ANODE_SEL_HZ = FRAME_RATE * NUM_DIGITS;
static constexpr int ANODE_SEL_PERIOD_US = 1000000 / ANODE_SEL_HZ;
static constexpr int DMA_TX_HZ = ANODE_SEL_HZ * PWM_PERIOD * 11 / 10;
static constexpr int PIO_CLKDIV = ntpc::SYSTEM_CLOCK_HZ / DMA_TX_HZ;

static uint8_t segment_table[256] = {0};

static char digits0[NUM_DIGITS] = {0};
static char digits1[NUM_DIGITS] = {0};
static uint8_t dram0[NUM_DIGITS * NUM_SEGMENTS] = {0};
static uint8_t dram1[NUM_DIGITS * NUM_SEGMENTS] = {0};
static uint8_t dram2[NUM_DIGITS * NUM_SEGMENTS] = {0};
static uint8_t dma_buff[NUM_DIGITS * PWM_PERIOD] = {0};
static int digit_sel = 0;

static int transition_num_digits = 0;
static uint64_t transition_start_ms = 0;

static repeating_timer_t st_timer;
static int dma_chan;

static void render_clock(uint64_t time_ms);
static int get_blink_alpha(uint64_t time_ms);
static void dram_draw_string(const char *s, uint8_t alpha = 255,
                             int start_digit = NUM_DIGITS - 1);
static void dram_draw_char(char c, uint8_t alpha, int idig);
static void dram_draw_dp(int digit_index, uint8_t alpha = 255);
static void dram_clear(char c = ' ', uint8_t alpha = 255);
static void decode_segments(uint8_t alpha = 255);
static void update_dma_buff();
static void print_digit(int index, int width, int value);

static bool repeating_timer_callback(repeating_timer_t *rt);

void init() {
  memset(segment_table, 0, sizeof(segment_table));
  segment_table[' '] = 0b00000000;
  segment_table['0'] = 0b00111111;
  segment_table['1'] = 0b00000110;
  segment_table['2'] = 0b01011011;
  segment_table['3'] = 0b01001111;
  segment_table['4'] = 0b01100110;
  segment_table['5'] = 0b01101101;
  segment_table['6'] = 0b01111101;
  segment_table['7'] = 0b00000111;
  segment_table['8'] = 0b01111111;
  segment_table['9'] = 0b01101111;
  segment_table['A'] = 0b01110111;
  segment_table['B'] = 0b01111100;
  segment_table['C'] = 0b00111001;
  segment_table['D'] = 0b01011110;
  segment_table['E'] = 0b01111001;
  segment_table['F'] = 0b01110001;
  segment_table['G'] = 0b00111101;
  segment_table['I'] = 0b00010000;
  segment_table['L'] = 0b00111000;
  segment_table['N'] = 0b01010100;
  segment_table['O'] = 0b01011100;
  segment_table['P'] = 0b01110011;
  segment_table['T'] = 0b01111000;
  segment_table['R'] = 0b01010000;
  segment_table['-'] = 0b01000000;

  for (int port : DIGIT_PORTS) {
    gpio_init(port);
    gpio_set_dir(port, GPIO_OUT);
    gpio_put(port, false);
    gpio_set_drive_strength(port, GPIO_DRIVE_STRENGTH_12MA);
  }
  memset(dma_buff, 0xFF, sizeof(dma_buff));

  turn_off();

  uint offset = pio_add_program(pio0, &seg7_pwm_tx_program);
  seg7_pwm_tx_program_init(pio0, 0, offset, SEGMENT_PORT_BASE, PIO_CLKDIV);

  dma_chan = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_PIO0_TX0);

  dma_channel_configure(dma_chan, &c, &pio0_hw->txf[0], NULL, PWM_PERIOD,
                        false);

  add_repeating_timer_us(-ANODE_SEL_PERIOD_US, repeating_timer_callback, NULL,
                         &st_timer);
}

void update_display(Context &ctx) {
  uint64_t tick_ms = to_ms_since_boot(get_absolute_time());
  const uint64_t time_ms = ctx.origin_time_ms + (tick_ms - ctx.origin_tick_ms);

  if (ctx.state == state_t::IDLE) {
    uint64_t elapsed_ms = tick_ms - ctx.last_try_time_ms;
    if (ctx.last_error != result_t::SUCCESS && elapsed_ms < 5000) {
      // エラー表示
      dram_clear();
      char msg[16];
      int err = static_cast<int>(ctx.last_error);
      snprintf(msg, sizeof(msg), "NTPERR%02X", err);
      dram_draw_string(msg);
    } else if (ctx.origin_time_ms == 0) {
      // 時刻未設定
      int alpha = get_blink_alpha(time_ms);
      dram_clear('-', alpha);
      dram_draw_dp(10, HYPHEN_COEFF * alpha / 256);
      dram_draw_dp(8, HYPHEN_COEFF * alpha / 256);
      dram_draw_dp(4, COLON_COEFF * alpha / 256);
      dram_draw_dp(2, COLON_COEFF * alpha / 256);
    } else {
      // 通常時刻表示
      render_clock(time_ms);
    }
  } else if (ctx.state == state_t::SETUP) {
    dram_clear();
    if (config::is_receiving()) {
      const char *s = "LOADING";
      for (int i = 0; s[i] != '\0'; i++) {
        int pos = NUM_DIGITS - 1 - i;
        uint8_t alpha = get_blink_alpha(time_ms + pos * 100);
        dram_draw_char(s[i], alpha, pos);
      }
    } else {
      dram_draw_string("CONFIG", get_blink_alpha(time_ms));
    }
  } else if (ctx.state == state_t::VLCFG_ERROR) {
    dram_clear();
    char msg[16];
    int err = static_cast<int>(config::last_vlcfg_error());
    snprintf(msg, sizeof(msg), "CFGERR%02X", err);
    dram_draw_string(msg);
    dram_draw_dp(6, config::last_bit() ? 255 : 0);
  }

  if (tick_ms - ctx.origin_tick_ms > 25 * 3600 * 1000) {
    // 前回の同期から25時間以上経っていたら右端のドットを点滅
    uint8_t alpha = (tick_ms % 1000 < 500) ? 255 : 0;
    dram_draw_dp(0, alpha);
  }

  update_dma_buff();
}

void turn_off() {
  memset(digits0, 0, sizeof(digits0));
  memset(digits1, 0, sizeof(digits1));
  transition_num_digits = 0;

  memset(dram0, 0, sizeof(dram0));
  memset(dram1, 0, sizeof(dram1));
  memset(dram2, 0, sizeof(dram2));
  memset(dma_buff, 0xFF, sizeof(dma_buff));
}

static void render_clock(uint64_t time_ms) {
  time_t now_sec = (time_ms + FADE_TIME_MS) / 1000;
  struct tm *t = gmtime(&now_sec);

  int sec = t->tm_sec;
  int min = t->tm_min;
  int hour = t->tm_hour;
  int day = t->tm_mday;
  int mon = t->tm_mon + 1;
  int year = t->tm_year + 1900;

  // 時刻表示
  print_digit(0, 2, sec);
  print_digit(2, 2, min);
  print_digit(4, 2, hour);
  print_digit(6, 2, day);
  print_digit(8, 2, mon);
  print_digit(10, 4, year);

  if (transition_num_digits == 0) {
    // 数字の変化点検出
    for (int idig = NUM_DIGITS - 1; idig >= 0; idig--) {
      if (digits0[idig] != digits1[idig]) {
        transition_num_digits = idig + 1;
        transition_start_ms = time_ms;
        break;
      }
    }
    memcpy(digits1, digits0, sizeof(digits0));

    decode_segments(255);

    // 日付のハイフン点灯
    dram0[8 * NUM_SEGMENTS + 7] = HYPHEN_COEFF;
    dram0[10 * NUM_SEGMENTS + 7] = HYPHEN_COEFF;

  } else {
    uint32_t t = time_ms - transition_start_ms;

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
    if ((int)t >= transition_num_digits * DIGIT_DELAY_MS + FADE_TIME_MS * 3) {
      memcpy(dram1, dram0, sizeof(dram0));
      transition_num_digits = 0;
    }
  }

  // コロンの点滅
  int alpha = get_blink_alpha(time_ms);
  dram2[2 * NUM_SEGMENTS + 7] = 128 * alpha / 256;
  dram2[4 * NUM_SEGMENTS + 7] = 128 * alpha / 256;
}

static int get_blink_alpha(uint64_t time_ms) {
  int alpha;
  int millisec = time_ms % 1000;
  constexpr int FADEIN_MS = 100;
  constexpr int FADEOUT_MS = 500 - FADEIN_MS;
  if (millisec < 500) {
    alpha = 255;
  } else if (millisec < 1000 - FADEIN_MS) {
    alpha = 256 * (FADEOUT_MS - (millisec - 500)) / FADEOUT_MS;
    alpha = alpha * alpha / 256;
  } else {
  }
  if (millisec < FADEIN_MS) {
    alpha = 255 * millisec / FADEIN_MS;
    alpha = 255 - alpha;
    alpha = 255 - (alpha * alpha) / 255;
  } else if (millisec < 1000 - FADEOUT_MS) {
    alpha = 255;
  } else {
    alpha = 255 * (999 - millisec) / FADEOUT_MS;
    alpha = 255 - alpha;
    alpha = 255 - (alpha * alpha) / 255;
  }
  if (alpha < 0) alpha = 0;
  if (alpha > 255) alpha = 255;
  return alpha;
}

static void print_digit(int index, int width, int value) {
  char *wptr = digits0 + index;
  for (int i = 0; i < width; i++) {
    *(wptr++) = '0' + (value % 10);
    value /= 10;
  }
}

// タイマ割り込みハンドラ
static bool repeating_timer_callback(repeating_timer_t *rt) {
  constexpr uint32_t ANODE_PORT_MASK = ((1 << NUM_DIGITS) - 1)
                                       << DIGIT_PORT_BASE;
  if (!dma_channel_is_busy(dma_chan)) {
    // anode select
    gpio_put_masked(ANODE_PORT_MASK, (1 << digit_sel));

    // kick DMA
    uint8_t *read_addr = dma_buff + (digit_sel * PWM_PERIOD);
    dma_channel_set_read_addr(dma_chan, read_addr, true);
    digit_sel = (digit_sel + 1) % NUM_DIGITS;
  }
  return true;
}

static void dram_draw_string(const char *s, uint8_t alpha, int start_digit) {
  int idig = start_digit;
  while (*s && idig >= 0) {
    char c = *s;
    dram_draw_char(c, alpha, idig);
    idig--;
    s++;
  }
}

static void dram_draw_char(char c, uint8_t alpha, int idig) {
  uint8_t seg_bits = segment_table[(int)c];
  for (int iseg = 0; iseg < NUM_SEGMENTS - 1; iseg++) {
    dram2[idig * NUM_SEGMENTS + iseg] = (seg_bits & 1) ? alpha : 0;
    seg_bits >>= 1;
  }
}

static void dram_draw_dp(int digit_index, uint8_t alpha) {
  if (digit_index < 0 || digit_index >= NUM_DIGITS) return;
  dram2[digit_index * NUM_SEGMENTS + 7] = alpha;
}

static void dram_clear(char c, uint8_t alpha) {
  uint8_t seg_bits = segment_table[(int)c];
  for (int iseg = 0; iseg < NUM_SEGMENTS - 1; iseg++) {
    dram2[iseg] = (seg_bits & 1) ? alpha : 0;
    seg_bits >>= 1;
  }
  for (int idig = 1; idig < NUM_DIGITS; idig++) {
    memcpy(dram2 + idig * NUM_SEGMENTS, dram2, NUM_SEGMENTS);
  }
}

static void decode_segments(uint8_t alpha) {
  for (int idig = 0; idig < NUM_DIGITS; idig++) {
    uint8_t seg_bits = segment_table[(int)digits1[idig]];
    for (int iseg = 0; iseg < NUM_SEGMENTS - 1; iseg++) {
      dram0[idig * NUM_SEGMENTS + iseg] = (seg_bits & 1) ? alpha : 0;
      seg_bits >>= 1;
    }
  }
}

static void update_dma_buff() {
  for (int d = 0; d < NUM_DIGITS; d++) {
    uint16_t thresh[NUM_SEGMENTS] = {0};
    for (int s = 0; s < NUM_SEGMENTS; s++) {
      uint32_t val = dram2[d * NUM_SEGMENTS + s];
      val = val * val;
      val = val * (PWM_PERIOD - 1) / (255 * 255);
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
}  // namespace display
