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

config::Config cfg;

void sync_with_ntp();

int main() {
  set_sys_clock_khz(SYSTEM_CLOCK_HZ / 1000, true);
  sleep_ms(100);

  stdio_inited = stdio_init_all();
  sleep_ms(500);

  NTPC_PRINTF("Hello.\r\n");

  ctx.state = state_t::IDLE;
  ctx.origin_time_ms = 0;
  ctx.origin_tick_ms = to_ms_since_boot(get_absolute_time());
  ctx.next_sync_tick_ms = 0;
  ctx.last_error = result_t::SUCCESS;

  display::init();

  setup_button.init();

  ctx.origin_tick_ms = to_ms_since_boot(get_absolute_time());
  config::init(cfg);

  uint64_t t_next_button_update_ms = 0;
  uint64_t t_next_display_update_ms = 0;

  while (true) {
    const uint64_t tick_ms = to_ms_since_boot(get_absolute_time());
    bool pulse_1ms = (tick_ms >= t_next_button_update_ms);
    if (pulse_1ms) {
      t_next_button_update_ms = tick_ms + 1;
    }

    setup_button.update(pulse_1ms);

    if (setup_button.on_clicked()) {
      NTPC_PRINTF("Setup button clicked\r\n");
    }

    switch (ctx.state) {
      case state_t::IDLE:
        if (tick_ms >= ctx.next_sync_tick_ms) {
          sync_with_ntp();
        } else if (setup_button.on_clicked()) {
          NTPC_PRINTF("Enter SETUP state\n");
          ctx.state = state_t::SETUP;
          config::start(cfg);
        }
        break;

      case state_t::SETUP: {
        bool success = false;
        if (config::update(cfg, &success)) {
          if (success) {
            sync_with_ntp();
            ctx.state = state_t::IDLE;
          } else {
            ctx.state = state_t::VLCFG_ERROR;
          }
        } else if (setup_button.on_clicked()) {
          ctx.state = state_t::IDLE;
          NTPC_PRINTF("Return to IDLE state\n");
        }
      } break;

      case state_t::VLCFG_ERROR:
        if (setup_button.on_clicked()) {
          ctx.state = state_t::IDLE;
          NTPC_PRINTF("Return to IDLE state\n");
        }
        break;
    }

    if (tick_ms >= t_next_display_update_ms) {
      display::update_display(ctx, tick_ms);
      t_next_display_update_ms = tick_ms + 5;
    }
  }

  return 0;
}

void sync_with_ntp() {
  display::turn_off();

  uint64_t time;
  auto res = ntp::get_time(cfg, &time);

  uint64_t tick_ms = to_ms_since_boot(get_absolute_time());
  ctx.last_try_time_ms = tick_ms;

  if (res != ntp::result_t::SUCCESS) {
    ctx.last_error = res;
    // If failed, retry in 10 minutes
    ctx.next_sync_tick_ms = tick_ms + 600 * 1000;
    NTPC_PRINTF("NTP time fetch failed: %d\n", static_cast<int>(res));
    return;
  }

  // Add timezone offset
  int tz = atoi(cfg.timezone);
  int tz_hour = tz / 100;
  int tz_min = tz % 100;
  time += (uint64_t)(tz_hour * 3600 + tz_min * 60) * 1000;

  ctx.origin_time_ms = time;
  ctx.origin_tick_ms = tick_ms;
  ctx.last_error = result_t::SUCCESS;

  // Schedule the next sync between 4 AM and 5 AM the next day
  time_t now_sec = time / 1000;
  struct tm *t = gmtime(&now_sec);
  int time_sec = t->tm_hour * 3600 + t->tm_min * 60 + t->tm_sec;
  int expire_sec = (24 + 4) * 3600 + (rand() % 3600);
  int sec_to_next_sync = (expire_sec - time_sec) % (24 * 3600);
  ctx.next_sync_tick_ms = ctx.origin_tick_ms + sec_to_next_sync * 1000;

  NTPC_PRINTF("NTP time synchronized.\n");
}
