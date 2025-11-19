#include <hardware/adc.h>

#include <algorithm>
#include <cstring>

#include "brightness.hpp"

namespace ntpc::brightness {

enum State {
  INTERVAL_SEC,
  INTERVAL_MIN,
  INTERVAL_HOUR,
};

static constexpr int SMOOTH_PERIOD = 64;
static constexpr int SMOOTH_INTERVAL_MS = 15 * 1000 / SMOOTH_PERIOD;
static constexpr int STARTUP_PERIOD = 60 * 1000 / SMOOTH_INTERVAL_MS;
static constexpr int HOURLY_PERIOD = 24 * 3;
static constexpr uint16_t BRIGHTNESS_MIN = 64;
static constexpr uint16_t BRIGHTNESS_MAX = 256;

static uint16_t smooth_buff_array[SMOOTH_PERIOD];
static int smooth_buff_index = 0;
static int smooth_buff_filled = 0;
static uint16_t smooth_value = 0;

static uint16_t startup_log[STARTUP_PERIOD];
static int startup_index = 0;
static uint16_t startup_min = 65535;
static uint16_t startup_max = 0;

#ifdef HOURLY_LOG
static uint32_t hourly_accum_sum = 0;
static int hourly_accum_count = 0;
static uint16_t hourly_log[HOURLY_PERIOD];
static int hourly_index = 0;
static int hourly_filled = 0;
static uint16_t hourly_min = 65535;
static uint16_t hourly_max = 0;
#endif

static float brightness = 256;

static State state = INTERVAL_SEC;

static uint64_t t_next_smooth_ms = 0;
static uint64_t t_next_hour_ms = 3600 * 1000;
static uint64_t t_next_brightness_update_ms = 0;

static uint16_t log2u16(uint16_t x);

void init() {
  adc_init();
  adc_gpio_init(26 + LIGHT_SENSOR_ADC_CH);
  adc_select_input(LIGHT_SENSOR_ADC_CH);
}

void update(Context &ctx, uint64_t tick_ms) {
  if (tick_ms >= t_next_smooth_ms) {
    t_next_smooth_ms = tick_ms + SMOOTH_INTERVAL_MS;

    // smooth brightness value
    smooth_buff_array[smooth_buff_index] = log2u16(read_adc());
    smooth_buff_index = (smooth_buff_index + 1) % SMOOTH_PERIOD;
    if (smooth_buff_filled < SMOOTH_PERIOD) {
      smooth_buff_filled += 1;
    }
    uint32_t sum = 0;
    for (int i = 0; i < smooth_buff_filled; i++) {
      sum += smooth_buff_array[i];
    }
    smooth_value = sum / smooth_buff_filled;

    if (startup_index < STARTUP_PERIOD) {
      startup_log[startup_index] = smooth_value;
      startup_index += 1;
      if (startup_index >= STARTUP_PERIOD) {
        std::sort(startup_log, startup_log + STARTUP_PERIOD);
        startup_max = startup_log[STARTUP_PERIOD * 5 / 6];
      }
    }
    if (smooth_value < startup_min) {
      startup_min = smooth_value;
    }

#if HOURLY_LOG
    // update brightness range
    hourly_accum_sum += smooth_value;
    hourly_accum_count += 1;
    if (tick_ms >= t_next_hour_ms) {
      t_next_hour_ms = tick_ms + 3600 * 1000;
      hourly_log[hourly_index] = hourly_accum_sum / hourly_accum_count;
      hourly_index = (hourly_index + 1) % HOURLY_PERIOD;
      if (hourly_filled < HOURLY_PERIOD) {
        hourly_filled += 1;
      }
      hourly_accum_count = 0;
      hourly_accum_sum = 0;

      int n = (hourly_filled / 24) * 24;
      uint16_t tmp[n];
      memcpy(tmp, hourly_log, sizeof(uint16_t) * n);
      std::sort(tmp, tmp + n);
      hourly_min = tmp[0];
      hourly_max = tmp[n * 7 / 8];
    }
#endif
  }

  if (tick_ms >= t_next_brightness_update_ms) {
    t_next_brightness_update_ms = tick_ms + 20;

    int32_t range_min, range_max;
#if HOURLY_LOG
    if (startup_index < STARTUP_PERIOD) {
      range_min = 0;
      range_max = 0;
    } else if (hourly_filled < 24) {
      range_min = startup_min;
      range_max = startup_max;
    } else {
      range_min = hourly_min;
      range_max = hourly_max;
    }
#else
    if (startup_index < STARTUP_PERIOD) {
      range_min = 0;
      range_max = 0;
    } else {
      range_min = startup_min;
      range_max = startup_max;
    }
#endif
    if (range_max - range_min < 256) {
      range_min = range_max - 256;
    }

    float goal = (float)(smooth_value - range_min) *
                     (float)(BRIGHTNESS_MAX - BRIGHTNESS_MIN) /
                     (float)(range_max - range_min) +
                 BRIGHTNESS_MIN;
    if (goal < BRIGHTNESS_MIN) {
      goal = BRIGHTNESS_MIN;
    } else if (goal > BRIGHTNESS_MAX) {
      goal = BRIGHTNESS_MAX;
    }

    float p = 0.01f;
    brightness = brightness * (1 - p) + goal * p;
    ctx.brightness = static_cast<uint16_t>(brightness);
  }
}

uint16_t read_adc() { return adc_read(); }

static uint16_t log2u16(uint16_t x) {
  static constexpr uint16_t table[] = {
      0,   22,  44,  63,  82,  100, 118, 134, 150,
      165, 179, 193, 207, 220, 232, 244, 256,
  };

  uint16_t ret = 0xc000;
  if (x & 0xf000) {
    if (x & 0xc000) {
      x >>= 2;
      ret += 0x2000;
    }
    if (x & 0x2000) {
      x >>= 1;
      ret += 0x1000;
    }
  } else {
    if (!(x & 0xffc0)) {
      x <<= 6;
      ret -= 0x6000;
    }
    if (!(x & 0xfe00)) {
      x <<= 3;
      ret -= 0x3000;
    }
    if (!(x & 0xf800)) {
      x <<= 2;
      ret -= 0x2000;
    }
    if (!(x & 0xf000)) {
      x <<= 1;
      ret -= 0x1000;
    }
  }

  int index = (x >> 8) & 0xf;
  uint16_t a = table[index];
  uint16_t b = table[index + 1];
  uint16_t q = x & 0xff;
  uint16_t p = 256 - q;
  ret += ((a * p + b * q) + 8) >> 4;

  return ret;
}

}  // namespace ntpc::brightness
