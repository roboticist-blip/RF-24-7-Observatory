#include "shared_state.h"

Metrics g_metrics;
Control g_ctl;
SweepRing g_sweepRing;
QueueHandle_t g_eventQueue = nullptr, g_diagQueue = nullptr;
TaskHandle_t g_rfTask = nullptr, g_svcTask = nullptr, g_netTask = nullptr;
SPIClass g_hspi(HSPI);

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static SweepSnapshot s_snap;
static volatile uint32_t s_seq = 0;
static EnvData s_env;
static RfEvent s_recent[CFG_RECENT_EVENTS];
static uint8_t s_recentHead = 0, s_recentCount = 0;

void sharedInit() {
  memset(&g_metrics, 0, sizeof(g_metrics));
  memset(&g_ctl, 0, sizeof(g_ctl));
  memset(&s_snap, 0, sizeof(s_snap));
  memset(&s_env, 0, sizeof(s_env));
  g_ctl.quiet_gate = CFG_QUIET_ACQUISITION;
  g_ctl.tft_enable = true;
  g_ctl.bl_enable = true;
  g_ctl.wifi_enable = CFG_ENABLE_WIFI_DASHBOARD || CFG_ENABLE_NTP_AT_BOOT;
  g_ctl.allow_display = g_ctl.allow_sd = g_ctl.allow_wifi = true;
  g_eventQueue = xQueueCreate(32, sizeof(RfEvent));
  g_diagQueue = xQueueCreate(8, sizeof(DiagRow));
}

void sharedPublishSweep(const SweepSnapshot& s) {
  portENTER_CRITICAL(&s_mux);
  memcpy(&s_snap, &s, sizeof(s_snap));
  s_seq = s_seq + 1;
  portEXIT_CRITICAL(&s_mux);
}

uint32_t sharedSweepSeq() { return s_seq; }

uint32_t sharedGetSweep(SweepSnapshot& out) {
  portENTER_CRITICAL(&s_mux);
  memcpy(&out, &s_snap, sizeof(out));
  uint32_t q = s_seq;
  portEXIT_CRITICAL(&s_mux);
  return q;
}

void sharedSetEnv(const EnvData& e) {
  portENTER_CRITICAL(&s_mux);
  s_env = e;
  portEXIT_CRITICAL(&s_mux);
}

void sharedGetEnv(EnvData& e) {
  portENTER_CRITICAL(&s_mux);
  e = s_env;
  portEXIT_CRITICAL(&s_mux);
}

void sharedPushRecentEvent(const RfEvent& e) {
  portENTER_CRITICAL(&s_mux);
  s_recent[s_recentHead] = e;
  s_recentHead = (uint8_t)((s_recentHead + 1) % CFG_RECENT_EVENTS);
  if (s_recentCount < CFG_RECENT_EVENTS) s_recentCount++;
  portEXIT_CRITICAL(&s_mux);
}

uint8_t sharedGetRecentEvents(RfEvent* out, uint8_t maxN) {
  portENTER_CRITICAL(&s_mux);
  uint8_t n = s_recentCount < maxN ? s_recentCount : maxN;
  for (uint8_t i = 0; i < n; i++) {
    int idx = (int)s_recentHead - 1 - i;
    if (idx < 0) idx += CFG_RECENT_EVENTS;
    out[i] = s_recent[idx];
  }
  portEXIT_CRITICAL(&s_mux);
  return n;
}
