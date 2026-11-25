#include "system_health.h"
#include "shared_state.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_arduino_version.h"

#define RTC_MAGIC 0x52463234UL   // "RF24"

RTC_NOINIT_ATTR static uint32_t rtc_magic;
RTC_NOINIT_ATTR static uint32_t rtc_boots;
RTC_NOINIT_ATTR static uint8_t  rtc_streak;

static BootInfo s_boot;
static volatile bool s_restart = false;
static volatile HealthState s_last = HEALTH_OK;
static uint32_t s_lastRfHb = 0;
static uint32_t s_lastRfHbMs = 0;

static const char* reasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

void healthBootInit() {
  const esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_POWERON || rtc_magic != RTC_MAGIC) {
    rtc_magic = RTC_MAGIC;
    rtc_boots = 0;
    rtc_streak = 0;
  }
  rtc_boots++;
  const bool abnormal = (r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT ||
                         r == ESP_RST_WDT || r == ESP_RST_BROWNOUT);
  if (abnormal) { if (rtc_streak < 250) rtc_streak++; }
  else if (r == ESP_RST_POWERON) rtc_streak = 0;      // a controlled SW restart keeps the streak untouched
  s_boot.reason = r;
  s_boot.reasonName = reasonName(r);
  s_boot.streak = rtc_streak;
  s_boot.bootCount = rtc_boots;
  s_boot.brownout = (r == ESP_RST_BROWNOUT);

  // degraded profiles: RF acquisition always stays enabled
  if (rtc_streak >= CFG_SAFE_STAGE1_STREAK) { g_ctl.allow_display = false; g_ctl.allow_wifi = false; }
  if (rtc_streak >= CFG_SAFE_STAGE2_STREAK) { g_ctl.allow_sd = false; }
  if (s_boot.brownout) g_ctl.allow_wifi = false;      // Wi-Fi TX bursts are the largest supply transients

  if (rtc_streak) {                                   // back-off against fast reboot loops
    uint32_t d = (uint32_t)rtc_streak * CFG_BOOT_BACKOFF_STEP_MS;
    if (d > CFG_BOOT_BACKOFF_MAX_MS) d = CFG_BOOT_BACKOFF_MAX_MS;
    delay(d);
  }
}

const BootInfo& healthBoot() { return s_boot; }

void healthWdtInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t c;
  c.timeout_ms = CFG_WDT_TIMEOUT_S * 1000;
  c.idle_core_mask = 0;
  c.trigger_panic = true;
  esp_task_wdt_reconfigure(&c);
#else
  esp_task_wdt_init(CFG_WDT_TIMEOUT_S, true);
#endif
}

void healthMarkStable() { rtc_streak = 0; }

void healthCollect(HealthSnapshot& h) {
  h.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
  h.heap_free = ESP.getFreeHeap();
  h.heap_min = ESP.getMinFreeHeap();
  h.heap_largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  h.stack_rf = g_rfTask ? (uint32_t)uxTaskGetStackHighWaterMark(g_rfTask) : 0;
  h.stack_svc = g_svcTask ? (uint32_t)uxTaskGetStackHighWaterMark(g_svcTask) : 0;
  h.stack_net = g_netTask ? (uint32_t)uxTaskGetStackHighWaterMark(g_netTask) : 0;
  h.tasks_alive = 0;
  h.state = HEALTH_OK;
}

HealthState healthEvaluate(HealthSnapshot& h) {
  healthCollect(h);
  const uint32_t nowMs = millis();
  if (g_metrics.rf_heartbeat != s_lastRfHb) { s_lastRfHb = g_metrics.rf_heartbeat; s_lastRfHbMs = nowMs; }
  const bool rfAlive = (nowMs - s_lastRfHbMs) < (uint32_t)CFG_RF_STALL_LIMIT_S * 1000UL;
  h.tasks_alive = (uint8_t)((rfAlive ? 1 : 0) | 2 | (g_netTask ? 4 : 0));
  HealthState st = HEALTH_OK;
  if (h.heap_free < CFG_HEAP_WARN_BYTES || g_metrics.rf_state == RF_RECOVERING || g_metrics.sd_state == SD_ERROR ||
      g_metrics.env_failures > 0 || h.stack_rf < 512 || h.stack_svc < 512)
    st = HEALTH_WARN;
  if (g_metrics.rf_state == RF_FAULT || !rfAlive) st = HEALTH_FAULT;
  if (h.heap_free < CFG_HEAP_CRIT_BYTES || (!rfAlive && (nowMs - s_lastRfHbMs) > 2UL * CFG_RF_STALL_LIMIT_S * 1000UL))
    s_restart = true;                                  // controlled restart after the logger has been flushed
  h.state = st;
  s_last = st;
  return st;
}

HealthState healthLast() { return s_last; }
bool healthRestartRequested() { return s_restart; }
void healthRequestRestart() { s_restart = true; }
