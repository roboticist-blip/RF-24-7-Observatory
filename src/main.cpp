#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>
#include <errno.h>
#include "shared_state.h"
#include "rf_acquisition.h"
#include "display_ui.h"
#include "data_logger.h"
#include "environment.h"
#include "time_manager.h"
#include "system_health.h"
#include "diagnostics.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#if CFG_WIFI_SUPPORT
#include <WiFi.h>
#if CFG_ENABLE_WIFI_DASHBOARD
#include <WebServer.h>
#endif
#endif

namespace {
DataLogger logger;
bool displayReady = false;
#if CFG_WIFI_SUPPORT
bool wifiStarted = false, ntpStarted = false;
uint64_t wifiRetryAt = 0;
#if CFG_ENABLE_WIFI_DASHBOARD
WebServer server(CFG_WEB_PORT);
bool webStarted = false;
void configureWeb() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
      "<title>RF247</title><style>body{background:#101418;color:#cce8ee;font:16px monospace;padding:2em}pre{white-space:pre-wrap}</style>"
      "<h1>RF-24/7 Observatory</h1><pre id=s>Connecting...</pre><script>async function u(){try{let r=await fetch('/status');"
      "document.getElementById('s').textContent=JSON.stringify(await r.json(),null,2)}catch(e){}}u();setInterval(u,2000)</script>");
  });
  server.on("/status", HTTP_GET, []() {
    SweepSnapshot s; EnvData e; uint32_t seq = sharedGetSweep(s); sharedGetEnv(e);
    char b[480];
    snprintf(b, sizeof(b), "{\"sweeps\":%lu,\"rf_state\":%u,\"sd_state\":%u,\"valid_bins\":%u,\"peak_dbm\":%.1f,\"noise_dbm\":%.1f,\"occupancy_pct\":%u,\"environment_valid\":%s,\"temperature_c\":%.2f,\"pressure_hpa\":%.2f,\"humidity_pct\":null,\"heap\":%lu}",
      (unsigned long)seq, (unsigned)g_metrics.rf_state, (unsigned)g_metrics.sd_state, s.g.n_valid,
      rfLvlToDbm(s.g.peak_sweep_lvl), (s.g.med_q8 / 256.0f - 255.0f) * .5f, s.g.occ_inst_pct,
      e.valid ? "true" : "false", e.t_c100 / 100.0f, e.p_pa / 100.0f, (unsigned long)ESP.getFreeHeap());
    server.send(200, "application/json", b);
  });
}
#endif
void wifiService(uint64_t now) {
  const bool wanted = g_ctl.wifi_enable && g_ctl.allow_wifi;
  if (!wanted && wifiStarted) {
#if CFG_ENABLE_WIFI_DASHBOARD
    if (webStarted) { server.stop(); webStarted = false; }
#endif
    WiFi.mode(WIFI_OFF); wifiStarted = ntpStarted = false; g_ctl.wifi_active = false;
  }
  if (wanted && !wifiStarted && now >= wifiRetryAt) {
    const bool sta = (CFG_WIFI_USE_STA || CFG_ENABLE_NTP_AT_BOOT) && strlen(CFG_WIFI_SSID) && !g_ctl.diag_active;
    if (sta) { wifiStarted = WiFi.mode(WIFI_STA); if (wifiStarted) WiFi.begin(CFG_WIFI_SSID, CFG_WIFI_PASS); }
    else { wifiStarted = WiFi.mode(WIFI_AP) && WiFi.softAP(CFG_WIFI_AP_SSID, CFG_WIFI_AP_PASS); }
    if (wifiStarted) WiFi.setTxPower(CFG_WIFI_TX_POWER);
    else { WiFi.mode(WIFI_OFF); wifiRetryAt = now + 30000; }
    g_ctl.wifi_active = wifiStarted;
  }
  if (!wifiStarted) return;
#if CFG_ENABLE_NTP_AT_BOOT
  if (!ntpStarted && WiFi.status() == WL_CONNECTED) { timeStartNtp(); ntpStarted = true; }
#endif
#if CFG_ENABLE_WIFI_DASHBOARD
  if (!webStarted) { server.begin(); webStarted = true; }
  server.handleClient();
#endif
}
#else
void wifiService(uint64_t) {}
#endif

void printStatus() {
  static SweepSnapshot s; EnvData e; HealthSnapshot h;
  const uint32_t seq = sharedGetSweep(s); sharedGetEnv(e); healthCollect(h);
  Serial.printf("RF=%u SD=%u sweeps=%lu valid=%u/%u sweep=%lu ms quiet_overruns=%lu heap=%lu min=%lu stack_RF=%lu stack_SVC=%lu\n",
    (unsigned)g_metrics.rf_state, (unsigned)g_metrics.sd_state, (unsigned long)seq, s.g.n_valid, (unsigned)RF_N,
    (unsigned long)g_metrics.sweep_ms_last, (unsigned long)g_metrics.quiet_overruns, (unsigned long)h.heap_free,
    (unsigned long)h.heap_min, (unsigned long)h.stack_rf, (unsigned long)h.stack_svc);
  if (seq) Serial.printf("peak=%.1f dBm at %lu Hz median=%.1f dBm occupancy=%u%% events=%lu\n",
    rfLvlToDbm(s.g.peak_sweep_lvl), (unsigned long)rfBinFreqHz(s.g.peak_sweep_bin),
    (s.g.med_q8 / 256.0f - 255.0f) * 0.5f, s.g.occ_inst_pct, (unsigned long)g_metrics.events_started);
  if (e.valid) Serial.printf("BMP280: %.2f C %.2f hPa\n", e.t_c100 / 100.0f, e.p_pa / 100.0f);
  else Serial.printf("BMP280: unavailable (status %u)\n", e.status);
  Serial.printf("time=%s logs=%s ring=%u dropped=%lu bytes_lost=%lu\n", timeSynced() ? "UTC" : "uptime", logger.dir(),
    g_sweepRing.depth(), (unsigned long)g_metrics.sweeps_not_logged, (unsigned long)g_metrics.log_bytes_lost);
}

void consoleCommand(char* s) {
  if (!strcmp(s, "help") || !strcmp(s, "?")) Serial.println("Commands: status | c (settling test) | diag | abort | flush | tftredraw | time <UTC epoch seconds> | reboot | help");
  else if (!strcmp(s, "status")) printStatus();
  else if (!strcmp(s, "c")) { if (g_metrics.rf_state == RF_OK && !g_ctl.diag_active) g_ctl.req_settle = true; else Serial.println("RF not ready or diagnostic running"); }
  else if (!strcmp(s, "diag")) { if (g_metrics.rf_state == RF_OK && g_metrics.sweeps_done >= CFG_BASELINE_SWEEPS) g_ctl.req_diag = true; else Serial.println("RF not ready or baseline still learning"); }
  else if (!strcmp(s, "abort")) g_ctl.req_diag_abort = true;
  else if (!strcmp(s, "flush")) { logger.service(timeMonoMs(), true); Serial.println("Flush requested; check SD status"); }
  else if (!strcmp(s, "tftredraw")) {
    if (displayReady && !g_ctl.diag_active) { uiForceRedraw(); Serial.println("TFT full redraw requested"); }
    else Serial.println("TFT unavailable or RF diagnostic active");
  }
  else if (!strcmp(s, "reboot")) healthRequestRestart();
  else if (!strncmp(s, "time ", 5)) {
    char* end; errno = 0; unsigned long v = strtoul(s + 5, &end, 10);
    if (errno || end == s + 5 || *end || v < CFG_TIME_MIN_VALID_EPOCH || v > 2147483647UL) Serial.println("Invalid epoch (2025 through 2038)");
    else { timeSetEpoch((uint32_t)v, TS_SERIAL); Serial.println(timeSynced() ? "UTC synchronized" : "Time sync failed"); }
  } else if (*s) Serial.println("Unknown command; enter help");
}
void consolePoll() {
#if CFG_SERIAL_CONSOLE
  static char line[64]; static size_t n = 0; static bool overflow = false;
  for (unsigned i = 0; i < 128 && Serial.available(); ++i) {
    int c = Serial.read();
    if (c == '\n' || c == '\r') { if (!overflow && n) { line[n] = 0; consoleCommand(line); } n = 0; overflow = false; }
    else if (c == 8 || c == 127) { if (n) --n; }
    else if (c >= 32 && c < 127) { if (n < sizeof(line)-1) line[n++] = (char)c; else overflow = true; }
  }
#endif
}

void serviceTask(void*) {
  esp_task_wdt_add(nullptr);
  uint64_t nextEnv = 0, nextProbe = 0, nextHealth = 0, nextHealthLog = 0;
  bool stable = false;
  for (;;) {
    // In quiet mode, only perform bus work on RF's explicit window notification.
    const bool notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(g_ctl.quiet_gate ? 1000 : CFG_UI_REFRESH_MS)) != 0;
    if (!notified && g_ctl.quiet_gate && g_rfTask) {
      // A silent RF task must still be detected even when no bus window arrives.
      HealthSnapshot h; healthEvaluate(h);
      if (healthRestartRequested()) ESP.restart(); // no safe bus window to flush in
      esp_task_wdt_reset(); continue;
    }
    const uint32_t window = __atomic_load_n(&g_ctl.window_seq, __ATOMIC_ACQUIRE);
    const int64_t start = esp_timer_get_time();
    const uint64_t now = timeMonoMs();
    wifiService(now); timePoll();
    // Normal SD work is kept in the notified window during diagnostic TFT-only sweeps.
    const bool logAllowed = !g_ctl.diag_active || g_ctl.sd_stress || notified;
    if (logAllowed) logger.service(now, false);
    if (g_ctl.sd_stress || notified || !g_ctl.diag_active) logger.stress(g_ctl.sd_stress);
#if CFG_ENABLE_BMP280
    if (notified || !g_rfTask || !g_ctl.quiet_gate) {
      if (!envPresent() && now >= nextProbe) { envInit(); nextProbe = now + CFG_ENV_REINIT_INTERVAL_S * 1000ULL; }
      if (now >= nextEnv) {
        nextEnv = now + CFG_ENV_SAMPLE_INTERVAL_S * 1000ULL;
        EnvData e; if (!envSample(e)) g_metrics.env_failures++;
        sharedSetEnv(e); if (logAllowed) logger.logEnv(e);
      }
    }
#endif
    if (displayReady) {
      if (g_ctl.bl_enable != g_ctl.bl_state && CFG_TFT_BL_SWITCHABLE) uiSetBacklight(g_ctl.bl_enable);
      if (g_ctl.tft_enable) uiUpdate(now);
    }
    if (now >= nextHealth) {
      nextHealth = now + 1000; HealthSnapshot h; healthEvaluate(h);
      if (now >= nextHealthLog && logAllowed) { nextHealthLog = now + CFG_LOG_HEALTH_INTERVAL_S * 1000ULL; logger.logHealth(h); }
      if (!stable && now >= CFG_STABLE_UPTIME_S * 1000ULL && g_metrics.rf_state == RF_OK && h.state != HEALTH_FAULT) { healthMarkStable(); stable = true; }
    }
    consolePoll();
    if (healthRestartRequested()) { logger.service(now, true); logger.flushAll(); delay(50); ESP.restart(); }
    g_metrics.svc_heartbeat++;
    const uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
    g_metrics.svc_busy_us_last = elapsed;
    if (elapsed > g_metrics.svc_busy_us_max) g_metrics.svc_busy_us_max = elapsed;
    if (notified && g_rfTask) {
      __atomic_store_n(&g_ctl.window_done, window, __ATOMIC_RELEASE);
      xTaskNotifyGive(g_rfTask);
    }
    esp_task_wdt_reset();
  }
}
} // namespace

void setup() {
  Serial.begin(115200);
  // sharedInit must precede the boot policy, which modifies the shared profile.
  sharedInit(); timeInit(); healthBootInit(); healthWdtInit();
  Serial.printf("\n%s %s | reset=%s streak=%u\n", CFG_FW_NAME, CFG_FW_VERSION, healthBoot().reasonName, healthBoot().streak);
  if (!g_eventQueue || !g_diagQueue) { Serial.println("Fatal: queue allocation failed"); delay(1000); ESP.restart(); return; }
  g_ctl.allow_display = g_ctl.allow_display && CFG_ENABLE_DISPLAY;
  g_ctl.allow_sd = g_ctl.allow_sd && CFG_ENABLE_SD;
  g_ctl.allow_wifi = g_ctl.allow_wifi && CFG_WIFI_SUPPORT;
  diagRestoreDefaults();
  // Both chip selects must be high before either shared-bus peripheral is initialized.
  pinMode(PIN_TFT_CS, OUTPUT); digitalWrite(PIN_TFT_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
  g_hspi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, -1);
  logger.begin(g_hspi, g_ctl.allow_sd);
  // Establish the display address mode AFTER the initial SD mount.
  digitalWrite(PIN_SD_CS, HIGH);
  displayReady = g_ctl.allow_display && uiInit(g_hspi);
#if CFG_WIFI_SUPPORT
  WiFi.persistent(false); WiFi.mode(WIFI_OFF);
#if CFG_ENABLE_WIFI_DASHBOARD
  configureWeb();
#endif
#endif
  if (xTaskCreatePinnedToCore(serviceTask, "service", CFG_TASK_SVC_STACK, nullptr, CFG_TASK_SVC_PRIO, &g_svcTask, CFG_TASK_SVC_CORE) != pdPASS ||
      xTaskCreatePinnedToCore(rfAcquisitionTask, "rf", CFG_TASK_RF_STACK, nullptr, CFG_TASK_RF_PRIO, &g_rfTask, CFG_TASK_RF_CORE) != pdPASS) {
    Serial.println("Fatal: task allocation failed"); delay(1000); ESP.restart(); return;
  }
  Serial.println("Enter help for commands. RF radio is receive-only; Wi-Fi is off by default.");
}
void loop() { delay(1000); }
