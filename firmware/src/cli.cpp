#include "cli.hpp"

#include <cstdio>
#include <cstring>

namespace cli {
using namespace ntpc;

struct ConfigEntry {
  char *buffer;
  size_t max_len;
  char alt_char;
  const char *prompt;
};

Config buff;
ConfigEntry entries[] = {
    {buff.region, REGION_MAX_LEN, '\0',
     "Enter Wi-Fi region code"
     " (Leave blank to use worldwide settings):"},
    {buff.ssid, SSID_MAX_LEN, '\0', "Enter Wi-Fi SSID:"},
    {buff.password, PASSWORD_MAX_LEN, '*', "Enter Wi-Fi Password:"},
    {buff.ntp_host, NTP_HOST_MAX_LEN, '\0',
     "Enter NTP hostname (Leave blank to use pool.ntp.org):"},
    {buff.timezone, TIMEZONE_MAX_LEN, '\0',
     "Enter UTC offset (e.g., -0400, 0900, ...):"},
    {nullptr, 0},  // terminator
};
static constexpr int NUM_ENTRIES = sizeof(entries) / sizeof(entries[0]) - 1;

int entry_index = 0;
size_t input_size = 0;

static void input_init();
static bool input_accept();

void start(Config &cfg) {
  if (!ntpc::debug_mode) return;

  buff = cfg;
  buff.password[0] = '\0';

  printf("==================================================\r\n");
  printf("NTP Clock Setup Interface\r\n");
  printf("==================================================\r\n");

  entry_index = 0;
  input_init();
}

bool update(Config &cfg) {
  if (!ntpc::debug_mode) return false;

  if (entry_index < NUM_ENTRIES && input_accept()) {
    entry_index++;
    input_size = 0;
    input_init();
    if (entry_index >= NUM_ENTRIES) {
      cfg = buff;
    }
  }

  return (entry_index >= NUM_ENTRIES);
}

static void input_init() {
  if (entry_index >= NUM_ENTRIES) {
    printf("Thank you.\r\n");
    printf("==================================================\r\n");
    return;
  }

  ConfigEntry &entry = entries[entry_index];
  input_size = strnlen(entry.buffer, entry.max_len);
  printf("%s\r\n", entry.prompt);
  printf("> %s", entry.buffer);
}

static bool input_accept() {
  if (entry_index > NUM_ENTRIES) {
    return false;
  }

  ConfigEntry &entry = entries[entry_index];

  int c = getchar_timeout_us(0);
  if (c == PICO_ERROR_TIMEOUT) {
    return false;
  }

  if (c == '\r' || c == '\n') {
    putchar('\r');
    putchar('\n');
    entry.buffer[input_size] = '\0';
    return true;
  }

  if (c == '\b' || c == 127) {  // Backspace
    if (input_size > 0) {
      input_size--;
      putchar('\b');
      putchar(' ');
      putchar('\b');
    }
  } else if (c >= 32 && c <= 126) {  // Printable characters
    if (input_size < entry.max_len - 1) {
      entry.buffer[input_size++] = (char)c;
      if (entry.alt_char != '\0') {
        putchar(entry.alt_char);
      } else {
        putchar(c);
      }
    }
  }

  return false;
}

}  // namespace cli