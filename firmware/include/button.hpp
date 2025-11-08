#ifndef BUTTON_HPP
#define BUTTON_HPP

#include <hardware/gpio.h>
#include <pico/stdlib.h>

class Button {
 public:
  const int port;
  uint8_t shift_reg = 0;
  bool state = false;
  bool clicked = false;

  inline Button(int port) : port(port) {}

  inline void init() {
    gpio_init(port);
    gpio_set_dir(port, GPIO_IN);
    gpio_pull_up(port);
    shift_reg = 0;
    state = false;
  }

  inline void update() {
    bool down = gpio_get(port) == 0;
    shift_reg = (shift_reg << 1) | (down ? 1 : 0);

    bool last_state = state;
    if (shift_reg == 0xFF) {
      state = true;
    } else if (shift_reg == 0x00) {
      state = false;
    }
    clicked = !state && last_state;
  }

  inline bool on_clicked() const { return clicked; }
};

#endif
