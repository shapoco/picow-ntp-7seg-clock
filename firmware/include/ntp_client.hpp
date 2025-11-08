#ifndef NTP_CLIENT_HPP
#define NTP_CLIENT_HPP

#include <string.h>
#include <time.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

namespace ntp {

enum class result_t {
  SUCCESS,
  UNKNOWN_ERROR,
  NULL_POINTER,
  ARCH_INIT_FAILED,
  WIFI_CONNECT_FAILED,
  DNS_FAILED,
  NTP_FAILED,
};

struct WiFiConfig {
  uint32_t country;
  const char *ssid;
  const char *password;
};

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

result_t get_time(const WiFiConfig &wifi_config, const char *host_name,
                  uint64_t *out_time);

#ifdef NTP_CLIENT_IMPLEMENTATION

static result_t dns_solve(const char *host_name, ip_addr_t *addr);
static void dns_found(const char *name, const ip_addr_t *addr, void *arg);
static result_t ntp_request(const ip_addr_t addr, uint64_t *result);
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port);

result_t get_time(const WiFiConfig &wifi_config, const char *host_name,
                  uint64_t *out_time) {
  result_t ret = result_t::UNKNOWN_ERROR;
  ip_addr_t addr;

  if (cyw43_arch_init_with_country(wifi_config.country)) {
    ret = result_t::ARCH_INIT_FAILED;
    goto init_failed;
  }
  cyw43_arch_enable_sta_mode();
  if (cyw43_arch_wifi_connect_timeout_ms(wifi_config.ssid, wifi_config.password,
                                         CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    ret = result_t::WIFI_CONNECT_FAILED;
    goto init_failed;
  }
  printf("%d\r\n", __LINE__);

  ret = dns_solve(host_name, &addr);
  if (ret != result_t::SUCCESS) {
    goto dns_failed;
  }
  printf("%d\r\n", __LINE__);

  ret = ntp_request(addr, out_time);
  if (ret != result_t::SUCCESS) {
    goto ntp_failed;
  }
  printf("%d\r\n", __LINE__);

ntp_failed:
dns_failed:
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

  int err = dns_gethostbyname(host_name, addr, dns_found, &ctx);
  if (err == ERR_OK) {
    ctx.success = true;
    printf("%d\r\n", __LINE__);
  } else if (err == ERR_INPROGRESS) {
    printf("%d\r\n", __LINE__);
    while (!ctx.called_back) {
      cyw43_arch_poll();
    }
    printf("%d\r\n", __LINE__);
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
  ctx.called_back = true;
  ctx.success = !!addr;
  if (addr) {
    ctx.result = *addr;
  }
}

static result_t ntp_request(const ip_addr_t addr, uint64_t *result) {
  NtpContext ctx;
  ctx.called_back = false;
  ctx.success = false;
  ctx.result_ms = 0;
  ctx.addr = addr;

  if (!result) {
    return result_t::NULL_POINTER;
  }

  //  udp_pcb *ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  udp_pcb *ntp_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
  if (!ntp_pcb) {
    return result_t::ARCH_INIT_FAILED;
  }
  udp_recv(ntp_pcb, ntp_recv, &ctx);
  printf("%d\r\n", __LINE__);

  absolute_time_t start = get_absolute_time();

  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_REQ_SIZE, PBUF_RAM);
  uint8_t *req = (uint8_t *)p->payload;
  memset(req, 0, NTP_REQ_SIZE);
  req[0] = 0x1b;
  // req[0] = 0x23;
  udp_sendto(ntp_pcb, p, &addr, PORT);
  pbuf_free(p);
  printf("%d\r\n", __LINE__);

  while (!ctx.called_back) {
    cyw43_arch_poll();
  }

  uint32_t elapsed_ms =
      absolute_time_diff_us(start, get_absolute_time()) / 1000;
  printf("%d ms elapsed\r\n", elapsed_ms);

  printf("%d\r\n", __LINE__);
  if (ctx.success) {
    *result = ctx.result_ms + elapsed_ms / 2;
  }
  printf("%d\r\n", __LINE__);

  udp_remove(ntp_pcb);
  return ctx.success ? result_t::SUCCESS : result_t::NTP_FAILED;
}

static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port) {
  NtpContext &ctx = *(NtpContext *)arg;
  uint8_t mode = pbuf_get_at(p, 0) & 0x7;
  uint8_t stratum = pbuf_get_at(p, 1);

  ctx.called_back = true;
  ctx.success = false;
  if (ip_addr_cmp(addr, &ctx.addr) && port == PORT &&
      p->tot_len == NTP_REQ_SIZE && mode == 0x4 && stratum != 0) {
    uint8_t buff[8] = {0};
    pbuf_copy_partial(p, buff, sizeof(buff), 40);
    uint32_t s = (uint32_t)buff[0] << 24 | (uint32_t)buff[1] << 16 |
                 (uint32_t)buff[2] << 8 | (uint32_t)buff[3];
    uint32_t f = (uint32_t)buff[4] << 24 | (uint32_t)buff[5] << 16 |
                 (uint32_t)buff[6] << 8 | (uint32_t)buff[7];
    printf("%08x, %08x\r\n", s, f);
    ctx.result_ms = (uint64_t)(s - EPOCH) * 1000;
    ctx.result_ms += (uint64_t)f * 1000 / 0x100000000;
    ctx.success = true;
  }

  pbuf_free(p);
}

#endif

}  // namespace ntp

#endif
