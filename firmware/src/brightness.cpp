#include <hardware/adc.h>

#include <algorithm>
#include <cstring>

#include "brightness.hpp"

namespace ntpc::brightness {

static constexpr int SMOOTH_PERIOD = 64;
static constexpr int HOURLY_PERIOD = 24 * 3;
static constexpr uint16_t BRIGHTNESS_MIN = 64;
static constexpr uint16_t BRIGHTNESS_MAX = 256;

static uint16_t smooth_buff_array[SMOOTH_PERIOD];
static int smooth_buff_index = 0;
static int smooth_buff_filled = 0;
static float smooth_value = 0;
static float smooth_min = 4095;
static float smooth_max = 0;

static float hourly_accum_sum = 0;
static int hourly_accum_count = 0;
static float hourly_buff[HOURLY_PERIOD];
static int hourly_index = 0;
static int hourly_filled = 0;
static uint16_t hourly_min = 4095;
static uint16_t hourly_max = 0;

static float brightness = 256;

static uint64_t t_next_smooth_ms = 0;
static uint64_t t_next_hour_ms = 3600 * 1000;
static uint64_t t_next_brightness_update_ms = 0;

void init() {
  adc_init();
  adc_gpio_init(26 + LIGHT_SENSOR_ADC_CH);
  adc_select_input(LIGHT_SENSOR_ADC_CH);
}

uint16_t read_adc() { return adc_read(); }

void update(Context &ctx, uint64_t tick_ms) {
  if (tick_ms >= t_next_smooth_ms) {
    t_next_smooth_ms = tick_ms + 500;

    // smooth brightness value
    smooth_buff_array[smooth_buff_index] = read_adc();
    smooth_buff_index = (smooth_buff_index + 1) % SMOOTH_PERIOD;
    if (smooth_buff_filled < SMOOTH_PERIOD) {
      smooth_buff_filled += 1;
    }
    uint32_t sum = 0;
    for (int i = 0; i < smooth_buff_filled; i++) {
      sum += smooth_buff_array[i];
    }
    smooth_value = (float)sum / smooth_buff_filled;
    if (smooth_value < smooth_min) {
      smooth_min = smooth_value;
    }
    if (smooth_value > smooth_max) {
      smooth_max = smooth_value;
    }

    // update brightness range
    hourly_accum_sum += smooth_value;
    hourly_accum_count += 1;
    if (tick_ms >= t_next_hour_ms) {
      t_next_hour_ms = tick_ms + 3600 * 1000;
      hourly_buff[hourly_index] = hourly_accum_sum / hourly_accum_count;
      hourly_index = (hourly_index + 1) % HOURLY_PERIOD;
      if (hourly_filled < HOURLY_PERIOD) {
        hourly_filled += 1;
      }
      hourly_accum_count = 0;
      hourly_accum_sum = 0;

      float tmp[hourly_filled];
      memcpy(tmp, hourly_buff, sizeof(float) * hourly_filled);
      std::sort(tmp, tmp + hourly_filled);
      hourly_min = tmp[hourly_filled / 8];
      hourly_max = tmp[hourly_filled * 7 / 8];
    }
  }

  if (tick_ms >= t_next_brightness_update_ms) {
    t_next_brightness_update_ms = tick_ms + 20;

    float range_min = hourly_min;
    float range_max = hourly_max;
    if (hourly_filled < 24) {
      range_min = smooth_min + (smooth_max - smooth_min) / 4;
      range_max = smooth_min + (smooth_max - smooth_min) * 3 / 4;
    }
    if (range_max - range_min < 1) {
      range_min = range_max - 1;
    }

    int br = (smooth_value - range_min) * (BRIGHTNESS_MAX - BRIGHTNESS_MIN) /
                 (range_max - range_min) +
             BRIGHTNESS_MIN;
    if (br < BRIGHTNESS_MIN) {
      br = BRIGHTNESS_MIN;
    } else if (br > BRIGHTNESS_MAX) {
      br = BRIGHTNESS_MAX;
    }

    float p = 0.01f;
    brightness = brightness * (1 - p) + br * p;
    ctx.brightness = static_cast<uint16_t>(brightness);
  }
}

}  // namespace ntpc::brightness
