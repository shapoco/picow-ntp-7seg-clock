#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <hardware/clocks.h>
#include <pico/stdlib.h>

#include <vlcfg/vlconfig.hpp>

#include "button.hpp"
#include "common.hpp"
#include "config.hpp"
#include "display.hpp"
#include "eeprom.hpp"
#include "ntp_client.hpp"

using namespace ntpc;

static constexpr int SETUP_SW_PORT = 22;

Context ctx;

Button setup_button(SETUP_SW_PORT);

config::Data cfg;

void sync_with_ntp();

int main() {
  set_sys_clock_khz(SYSTEM_CLOCK_HZ / 1000, true);
  sleep_ms(100);

  stdio_init_all();
  sleep_ms(500);

  printf("Hello.\r\n");

  ctx.state = State::IDLE;
  ctx.epoch_ms = 0;
  ctx.epoch_sec = 0;
  ctx.epoch_tick_ms = 0;

  display::init();

  setup_button.init();

  ctx.epoch_tick_ms = to_ms_since_boot(get_absolute_time());
  if (config::init(cfg)) {
    sync_with_ntp();
  }

  uint64_t t_next_update_ms = 0;

  while (true) {
    const uint64_t now_ms = to_ms_since_boot(get_absolute_time());
    bool recv_success;

    setup_button.update();
    if (setup_button.on_clicked()) {
      printf("Setup button clicked\r\n");
    }

    switch (ctx.state) {
      case State::IDLE:
        if (setup_button.on_clicked()) {
          ctx.state = State::SETUP;
          config::recv_start();
          printf("Enter SETUP state\n");
        }
        break;

      case State::SETUP:
        if (config::recv_update(&recv_success, cfg)) {
          if (recv_success) {
            sync_with_ntp();
            ctx.state = State::IDLE;
          } else {
            ctx.state = State::VLCFG_ERROR;
          }
        } else if (setup_button.on_clicked()) {
          ctx.state = State::IDLE;
          printf("Return to IDLE state\n");
        }
        break;

      case State::VLCFG_ERROR:
        if (setup_button.on_clicked()) {
          ctx.state = State::IDLE;
          printf("Return to IDLE state\n");
        }
        break;
    }

    if (now_ms >= t_next_update_ms) {
      display::update_display(now_ms, ctx);
      t_next_update_ms = now_ms + 5;
    }
  }

  return 0;
}

void sync_with_ntp() {
  display::turn_off();

  uint64_t time;
  auto res = ntp::get_time(cfg, &time);
  if (res == ntp::result_t::SUCCESS) {
    int tz = atoi(cfg.timezone);
    int tz_hour = tz / 100;
    int tz_min = tz % 100;
    time += (uint64_t)(tz_hour * 3600 + tz_min * 60) * 1000;

    ctx.epoch_sec = (time / 1000);
    ctx.epoch_ms = (time % 1000);
    ctx.epoch_tick_ms = to_ms_since_boot(get_absolute_time());
    printf("NTP time synchronized: epoch_sec=%lu, epoch_ms=%llu\n",
           ctx.epoch_sec, ctx.epoch_ms);
  } else {
    printf("NTP time fetch failed: %d\n", static_cast<int>(res));
  }
}
