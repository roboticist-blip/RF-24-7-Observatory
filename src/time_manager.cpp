#include "time_manager.h"
#include <sys/time.h>
#include <time.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "config.h"

static volatile bool s_synced = false;
static volatile bool s_ntpPending = false;
static uint64_t s_ntpStartMs = 0;
static QueueHandle_t s_q = nullptr;

void timeInit() { s_q = xQueueCreate(4, sizeof(TimeSyncEvent)); }

uint64_t timeMonoMs() { return (uint64_t)(esp_timer_get_time() / 1000); }
bool timeSynced() { return s_synced; }

uint32_t timeEpoch() {
  if (!s_synced) return 0;
  return (uint32_t)time(nullptr);
}

static void recordSync(uint32_t epoch, uint8_t src) {
  s_synced = true;
  if (!s_q) return;
  TimeSyncEvent e;
  e.t_ms = timeMonoMs();
  e.epoch_s = epoch;
  e.source = src;
  xQueueSend(s_q, &e, 0);
}

void timeSetEpoch(uint32_t epoch, TimeSource src) {
  if (epoch < CFG_TIME_MIN_VALID_EPOCH) return;       // refuse implausible dates
  struct timeval tv;
  tv.tv_sec = (time_t)epoch;
  tv.tv_usec = 0;
  if (settimeofday(&tv, nullptr) == 0) recordSync(epoch, src);
}

void timeStartNtp() {
#if CFG_WIFI_SUPPORT
  configTime(0, 0, CFG_NTP_SERVER1, CFG_NTP_SERVER2);
  s_ntpStartMs = timeMonoMs();
  s_ntpPending = true;
#endif
}

void timePoll() {
  if (!s_ntpPending) return;
  const time_t t = time(nullptr);
  if (t >= (time_t)CFG_TIME_MIN_VALID_EPOCH) {
    s_ntpPending = false;
    recordSync((uint32_t)t, TS_NTP);
  } else if (timeMonoMs() - s_ntpStartMs >= CFG_NTP_TIMEOUT_S * 1000ULL) s_ntpPending = false;
}

bool timePeekSyncEvent(TimeSyncEvent& e) { return s_q && xQueuePeek(s_q, &e, 0) == pdTRUE; }
void timePopSyncEvent() { TimeSyncEvent e; if (s_q) xQueueReceive(s_q, &e, 0); }

const char* timeSourceName(uint8_t s) {
  switch (s) { case TS_NTP: return "NTP"; case TS_SERIAL: return "SERIAL"; case TS_WEB: return "WEB"; default: return "NONE"; }
}

void timeFormatUtc(char* out, size_t n, uint32_t epoch) {
  if (!epoch) { if (n) out[0] = 0; return; }
  time_t t = (time_t)epoch;
  struct tm tmv;
  gmtime_r(&t, &tmv);
  snprintf(out, n, "%04d-%02d-%02dT%02d:%02d:%02dZ", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

void timeFormatUptime(char* out, size_t n, uint64_t ms) {
  uint32_t s = (uint32_t)(ms / 1000ULL);
  uint32_t d = s / 86400U, h = (s / 3600U) % 24U, m = (s / 60U) % 60U, sec = s % 60U;
  if (d) snprintf(out, n, "%ud%02uh", (unsigned)d, (unsigned)h);
  else if (h) snprintf(out, n, "%02uh%02um", (unsigned)h, (unsigned)m);
  else snprintf(out, n, "%02um%02us", (unsigned)m, (unsigned)sec);
}
