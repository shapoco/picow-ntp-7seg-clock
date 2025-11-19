#ifndef BUTTON_HPP
#define BUTTON_HPP

#include <hardware/gpio.h>
#include <pico/stdlib.h>

class Button {
 public:
  static constexpr int SHIFT_REG_BITS = 10;
  static constexpr uint32_t SHIFT_REG_MASK = (1 << SHIFT_REG_BITS) - 1;

  const int port;
  uint32_t shift_reg = 0;
  bool state = false;
  bool clicked = false;

  inline Button(int port) : port(port) {}

  inline void init() {
    gpio_init(port);
    gpio_set_dir(port, GPIO_IN);
    gpio_pull_up(port);

    // charge input port
    sleep_ms(1);

    state = gpio_get(port) == 0;
    shift_reg = state ? SHIFT_REG_MASK : 0x00;
  }

  inline void update(bool pulse_1ms) {
    if (pulse_1ms) {
      bool down = gpio_get(port) == 0;
      shift_reg = (shift_reg << 1) | (down ? 1 : 0);
      shift_reg &= SHIFT_REG_MASK;
    }

    bool last_state = state;
    if (shift_reg == SHIFT_REG_MASK) {
      state = true;
    } else if (shift_reg == 0x00) {
      state = false;
    }
    clicked = !state && last_state;
  }

  inline bool is_down() const { return state; }

  inline bool on_clicked() const { return clicked; }
};

#endif
