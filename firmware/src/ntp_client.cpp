#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <cstring>
#include <ctime>

#include "ntp_client.hpp"

namespace ntp {

using namespace ntpc;

struct DnsContext {
  bool called_back;
  bool success;
  ip_addr_t result;
};

struct NtpContext {
  ip_addr_t addr;
  bool called_back;
  bool success;
  uint64_t result_ms;
};

static constexpr int NTP_REQ_SIZE = 48;
static constexpr int PORT = 123;
static constexpr uint64_t EPOCH = 2208988800;

static constexpr int TIMEOUT_MS = 5000;

static result_t dns_solve(const char *host_name, ip_addr_t *addr);
static void dns_found(const char *name, const ip_addr_t *addr, void *arg);
static result_t ntp_request(const ip_addr_t addr, uint64_t *result);
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port);

result_t get_time(const config::Config &cfg, uint64_t *out_time) {
  if (!out_time) {
    NTPC_VERBOSE("NTP client: NULL pointer for output time\n");
    return result_t::NULL_POINTER;
  }

  result_t ret = result_t::UNKNOWN_ERROR;
  ip_addr_t addr;

  uint32_t country = 0;
  {
    const char *c = cfg.region;
    int n = strnlen(c, 4);
    for (int i = 0; i < n; i++) {
      country |= (uint32_t)(*(c++)) << (i * 8);
    }
  }

  {
    NTPC_VERBOSE("Initializing CYW43 arch with country code 0x%08lX\n",
                 country);
    int err;
    if (country) {
      err = cyw43_arch_init_with_country(country);
    } else {
      err = cyw43_arch_init();
    }
    if (err != 0) {
      NTPC_VERBOSE("NTP client: ARCH init failed\n");
      ret = result_t::LWIP_ARCH_INIT_FAILED;
      goto init_failed;
    }
  }

  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

  cyw43_arch_enable_sta_mode();

  NTPC_VERBOSE("NTP client: Connecting to WiFi SSID='%s'\n", cfg.ssid);
  if (cyw43_arch_wifi_connect_timeout_ms(cfg.ssid, cfg.password,
                                         CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    NTPC_VERBOSE("NTP client: WiFi connect failed\n");
    ret = result_t::LWIP_WIFI_CONNECT_FAILED;
    goto connect_failed;
  }
  NTPC_VERBOSE("NTP client: WiFi connected\n");

  ret = dns_solve(cfg.ntp_host, &addr);
  if (ret != result_t::SUCCESS) {
    NTPC_VERBOSE("NTP client: DNS resolution failed\n");
    goto dns_failed;
  }
  NTPC_VERBOSE("NTP client: Resolved NTP server %s to %s\n", cfg.ntp_host,
               ipaddr_ntoa(&addr));

  ret = ntp_request(addr, out_time);
  if (ret != result_t::SUCCESS) {
    NTPC_VERBOSE("NTP client: NTP request failed\n");
    goto ntp_failed;
  }
  NTPC_VERBOSE("NTP client: NTP time received: %llu ms since epoch\n",
               *out_time);

ntp_failed:
dns_failed:
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
connect_failed:
  cyw43_arch_deinit();
init_failed:
  return ret;
}

static result_t dns_solve(const char *host_name, ip_addr_t *addr) {
  DnsContext ctx;
  ctx.called_back = false;
  ctx.success = false;

  if (!host_name || !addr) {
    return result_t::NULL_POINTER;
  }

  auto start = get_absolute_time();

  int err = dns_gethostbyname(host_name, &ctx.result, dns_found, &ctx);
  if (err == ERR_OK) {
    ctx.success = true;
  } else if (err == ERR_INPROGRESS) {
    while (!ctx.called_back) {
      cyw43_arch_poll();
      auto diff = absolute_time_diff_us(start, get_absolute_time()) / 1000;
      if (diff >= TIMEOUT_MS) {
        NTPC_VERBOSE("NTP client: DNS resolution timed out\n");
        return result_t::DNS_TIMEOUT;
      }
    }
  } else {
    ctx.success = false;
  }

  if (ctx.success) {
    *addr = ctx.result;
  }

  return ctx.success ? result_t::SUCCESS : result_t::DNS_FAILED;
}

static void dns_found(const char *name, const ip_addr_t *addr, void *arg) {
  DnsContext &ctx = *(DnsContext *)arg;

  NTPC_VERBOSE("NTP client: dns_found called: name=%s, addr=%s\n", name,
               addr ? ipaddr_ntoa(addr) : "null");

  ctx.called_back = true;
  ctx.success = !!addr;
  if (addr) {
    ctx.result = *addr;
  }
}

static result_t ntp_request(const ip_addr_t addr, uint64_t *result) {
  result_t ret = result_t::UNKNOWN_ERROR;
  NtpContext ctx;
  udp_pcb *ntp_pcb = nullptr;
  ctx.called_back = false;
  ctx.success = false;
  ctx.result_ms = 0;
  ctx.addr = addr;

  if (!result) {
    ret = result_t::NULL_POINTER;
    goto fatal;
  }

  //  udp_pcb *ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  ntp_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
  if (!ntp_pcb) {
    return result_t::LWIP_ARCH_INIT_FAILED;
  }
  udp_recv(ntp_pcb, ntp_recv, &ctx);

  {
    absolute_time_t start = get_absolute_time();

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_REQ_SIZE, PBUF_RAM);
    if (!p) {
      NTPC_VERBOSE("NTP client: pbuf_alloc failed\n");
      ret = result_t::NTP_FAILED;
      goto fatal;
    }

    uint8_t *req = (uint8_t *)p->payload;
    memset(req, 0, NTP_REQ_SIZE);
    req[0] = 0x1b;
    // req[0] = 0x23;
    err_t err = udp_sendto(ntp_pcb, p, &addr, PORT);
    if (err != ERR_OK) {
      NTPC_VERBOSE("NTP client: udp_sendto failed: %d\n", err);
      pbuf_free(p);
      ret = result_t::NTP_FAILED;
      goto send_failed;
    }

    pbuf_free(p);

    while (!ctx.called_back) {
      cyw43_arch_poll();
      auto diff = absolute_time_diff_us(start, get_absolute_time()) / 1000;
      if (diff >= TIMEOUT_MS) {
        NTPC_VERBOSE("NTP client: NTP request timed out\n");
        ret = result_t::NTP_TIMEOUT;
        goto send_failed;
      }
    }

    uint32_t elapsed_ms =
        absolute_time_diff_us(start, get_absolute_time()) / 1000;
    if (ctx.success) {
      *result = ctx.result_ms + elapsed_ms / 2;
      ret = result_t::SUCCESS;
    } else {
      ret = result_t::NTP_FAILED;
    }
  }

send_failed:
  udp_remove(ntp_pcb);
fatal:
  return ret;
}

static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port) {
  NtpContext &ctx = *(NtpContext *)arg;
  uint8_t mode = pbuf_get_at(p, 0) & 0x7;
  uint8_t stratum = pbuf_get_at(p, 1);

  NTPC_VERBOSE(
      "NTP client: ntp_recv called: port=%u, len=%u, mode=%u, stratum=%u\n",
      port, p->tot_len, mode, stratum);

  if (ip_addr_cmp(addr, &ctx.addr) && port == PORT &&
      p->tot_len == NTP_REQ_SIZE && mode == 0x4 && stratum != 0) {
    uint8_t buff[8] = {0};
    pbuf_copy_partial(p, buff, sizeof(buff), 40);
    uint32_t s = (uint32_t)buff[0] << 24 | (uint32_t)buff[1] << 16 |
                 (uint32_t)buff[2] << 8 | (uint32_t)buff[3];
    uint32_t f = (uint32_t)buff[4] << 24 | (uint32_t)buff[5] << 16 |
                 (uint32_t)buff[6] << 8 | (uint32_t)buff[7];
    ctx.result_ms = (uint64_t)(s - EPOCH) * 1000;
    ctx.result_ms += (uint64_t)f * 1000 / 0x100000000;
    ctx.called_back = true;
    ctx.success = true;
  }

  pbuf_free(p);
}

}  // namespace ntp
