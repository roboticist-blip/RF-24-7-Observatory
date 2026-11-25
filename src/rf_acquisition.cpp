#include "rf_acquisition.h"
#include <SPI.h>
#include "esp_task_wdt.h"
#include "sx1278_driver.h"
#include "rf_timing.h"
#include "rf_statistics.h"
#include "anomaly_detector.h"
#include "time_manager.h"
#include "diagnostics.h"

#define RF_SAMPLE_GAP_US ((CFG_RF_DWELL_US - CFG_RF_SETTLE_US) / CFG_RF_RSSI_SAMPLES)
static_assert(CFG_RF_DWELL_US > CFG_RF_SETTLE_US, "dwell must exceed settle time");
static_assert(CFG_RF_RSSI_SAMPLES >= 1, "at least one RSSI sample");

namespace {
SPIClass s_fspi(FSPI);
Sx1278 s_radio;
RfStats s_stats;
StatisticalDetector s_det;
uint32_t s_frf[RF_N];
uint8_t s_raw[RF_N];
SweepSnapshot s_snap;
SweepRecord s_rec;
uint32_t s_sweepId = 0, s_cycle = 0;
uint32_t s_backoffMs = CFG_RF_RECOVER_BACKOFF_MIN_MS, s_nextRecoverMs = 0;
uint8_t s_flatCount = 0;
bool s_loraSince = false, s_diagAutoDone = false;

class QueueSink : public EventSink {
 public:
  bool push(const RfEvent& e) override {
    sharedPushRecentEvent(e);
    if (e.phase != PH_END) g_metrics.events_started++;
    if (xQueueSend(g_eventQueue, &e, 0) != pdTRUE) { g_metrics.evq_drops++; return false; }
    return true;
  }
} s_sink;

void buildFrfTable() {
  for (uint16_t i = 0; i < RF_N; i++) s_frf[i] = Sx1278::hzToFrf(rfBinFreqHz(i));
}

bool radioBringUp() {
  for (uint8_t a = 0; a < CFG_RF_INIT_RETRIES; a++) {
    if (s_radio.begin(s_fspi) && s_radio.configureFsk(s_frf[0])) return true;
    delay(250);
    esp_task_wdt_reset();
  }
  return false;
}

bool measurePoint(uint16_t i, uint8_t& out) {
  const int64_t t0 = rfNowUs();
  if (!s_radio.retune(s_frf[i])) return false;
  rfSleepUntilUs(t0 + CFG_RF_SETTLE_US);
  if (!s_radio.waitRxReady(CFG_RF_PLL_TIMEOUT_US)) { g_metrics.pll_timeouts++; return false; }
  uint32_t acc = 0;
  for (uint8_t k = 1; k <= CFG_RF_RSSI_SAMPLES; k++) {
    rfSleepUntilUs(t0 + CFG_RF_SETTLE_US + (int64_t)k * RF_SAMPLE_GAP_US);
    acc += s_radio.readRssiRaw();
  }
  const uint8_t v = (uint8_t)((acc + CFG_RF_RSSI_SAMPLES / 2) / CFG_RF_RSSI_SAMPLES);
  if (v == RF_RAW_INVALID) return false;         // 0xFF = floating MISO / dead bus (also our "no data" sentinel)
  out = v;
  return true;
}

// Returns false if the sweep was aborted (radio considered failed). s_raw is always fully initialised.
bool sweepOnce(uint16_t& valid) {
  valid = 0;
  uint8_t consecutive = 0;
  if (!s_radio.enterFskRx()) { memset(s_raw, RF_RAW_INVALID, sizeof(s_raw)); g_metrics.points_dropped += RF_N; return false; }
  for (uint16_t i = 0; i < RF_N; i++) {
    uint8_t v;
    if (measurePoint(i, v)) { s_raw[i] = v; valid++; consecutive = 0; }
    else {
      s_raw[i] = RF_RAW_INVALID;
      g_metrics.points_dropped++;
      if (++consecutive >= CFG_RF_FAIL_RECOVER_POINTS) {
        for (uint16_t j = i + 1; j < RF_N; j++) { s_raw[j] = RF_RAW_INVALID; g_metrics.points_dropped++; }
        return false;
      }
    }
    if ((i & 0x3F) == 0) esp_task_wdt_reset();
  }
  return true;
}

bool isFlat() {
  uint8_t lo = 255, hi = 0;
  for (uint16_t i = 0; i < RF_N; i++) {
    if (s_raw[i] == RF_RAW_INVALID) continue;
    if (s_raw[i] < lo) lo = s_raw[i];
    if (s_raw[i] > hi) hi = s_raw[i];
  }
  return hi <= lo;                                // zero variation across 175 bins: heuristic for a stuck RSSI register
}

void pushLoraEvent(uint8_t type, uint32_t freq, float rssiDbm, const char* detail) {
  RfEvent e;
  memset(&e, 0, sizeof(e));
  e.t_ms = timeMonoMs();
  e.epoch_s = timeEpoch();
  e.synced = timeSynced();
  e.freq_hz = freq;
  e.type = type;
  e.phase = PH_INSTANT;
  e.peak_lvl = rfDbmToLvl(rssiDbm);
  e.noise_lvl = (uint8_t)(s_stats.global().med_q8 >> 8);
  strncpy(e.detail, detail, sizeof(e.detail) - 1);
  s_sink.push(e);
}

void runLoraWindow() {
#if CFG_ENABLE_LORA_MONITOR
  static const uint32_t kFreqs[] = CFG_LORA_FREQS_HZ;
  static const uint8_t kSf[] = CFG_LORA_SF_LIST;
  g_metrics.lora_windows++;
  s_loraSince = true;
  if (!s_radio.enterLora()) { g_metrics.rf_comm_errors++; g_metrics.rf_state = RF_RECOVERING; return; }
  for (uint8_t fi = 0; fi < sizeof(kFreqs) / sizeof(kFreqs[0]); fi++) {
    for (uint8_t si = 0; si < sizeof(kSf); si++) {
      if (!s_radio.loraConfigure(kFreqs[fi], CFG_LORA_BW_IDX, kSf[si], CFG_LORA_CR_IDX, CFG_LORA_SYNC_WORD)) continue;
      bool det = false;
      if (!s_radio.loraCad(det, CFG_LORA_CAD_TIMEOUT_MS)) continue;
      g_metrics.lora_cad_runs++;
      if (det) {
        g_metrics.lora_cad_hits++;
        char d[24];
        snprintf(d, sizeof(d), "SF%u CAD", kSf[si]);
        pushLoraEvent(EV_LORA_CAD, kFreqs[fi], -164.0f + s_radio.readReg(sxreg::LR_RSSI_VALUE), d);
        if (s_radio.loraStartRx()) {
          Sx1278::LoraPacket pk;
          const int64_t dl = rfNowUs() + (int64_t)CFG_LORA_RX_LISTEN_MS * 1000;
          bool got = false;
          while (rfNowUs() < dl) {
            rfSleepUs(2000);
            if (s_radio.loraPollPacket(pk)) { got = true; break; }
          }
          s_radio.standby();
          if (got) {
            g_metrics.lora_packets++;
            snprintf(d, sizeof(d), "SF%u SNR%.1f FEI%+.0f%s", kSf[si], pk.snr_db, pk.fei_hz, pk.crcError ? "!" : "");
            pushLoraEvent(EV_LORA_PACKET, kFreqs[fi], pk.rssi_dbm, d);
          }
        }
      }
      esp_task_wdt_reset();
    }
  }
  if (!s_radio.configureFsk(s_frf[0])) { g_metrics.rf_comm_errors++; g_metrics.rf_state = RF_RECOVERING; }
#endif
}

// Step-response characterization of the receiver (serial command 'c').
void settleTest() {
  uint16_t tgt = 0;
  uint8_t best = 0;
  for (uint16_t i = 0; i < RF_N; i++) { if (s_stats.bin(i).cur >= best) { best = s_stats.bin(i).cur; tgt = i; } }
  const uint16_t ref = (uint16_t)((tgt + RF_N / 2) % RF_N);
  static const uint32_t kT[] = {100, 200, 300, 500, 750, 1000, 1500, 2000, 2500, 3000, 4000, 5000, 7000, 10000};
  Serial.printf("SETTLE target=%u Hz (lvl %u), reference=%u Hz\n", (unsigned)rfBinFreqHz(tgt), best, (unsigned)rfBinFreqHz(ref));
  Serial.println("trial,t_us,raw_rssi");
  s_radio.enterFskRx();
  for (uint8_t trial = 0; trial < 3; trial++) {
    s_radio.retune(s_frf[ref]);
    rfSleepUs(30000);
    const int64_t t0 = rfNowUs();
    s_radio.retune(s_frf[tgt]);
    for (uint8_t k = 0; k < sizeof(kT) / sizeof(kT[0]); k++) {
      rfSleepUntilUs(t0 + kT[k]);
      Serial.printf("%u,%u,%u\n", trial, (unsigned)kT[k], s_radio.readRssiRaw());
    }
    esp_task_wdt_reset();
  }
  s_radio.standby();
}

void handleRequests() {
  if (g_ctl.req_settle) { g_ctl.req_settle = false; if (g_metrics.rf_state == RF_OK) settleTest(); }
  if (g_ctl.req_diag) { g_ctl.req_diag = false; if (!diagActive()) diagStart(); }
  if (g_ctl.req_diag_abort) { g_ctl.req_diag_abort = false; diagAbort(); }
#if CFG_DIAG_AUTORUN
  if (!s_diagAutoDone && !s_det.learning() && s_stats.sweeps() > CFG_BASELINE_SWEEPS + 10) { s_diagAutoDone = true; diagStart(); }
#endif
}
}  // namespace

void rfAcquisitionTask(void*) {
  rfTimingInit();
  esp_task_wdt_add(NULL);
  buildFrfTable();
  s_stats.reset();
  s_det.reset();
  s_fspi.begin(PIN_SX_SCK, PIN_SX_MISO, PIN_SX_MOSI, -1);
  const bool up = radioBringUp();
  g_metrics.rf_state = up ? RF_OK : RF_FAULT;
  s_nextRecoverMs = millis() + s_backoffMs;
  Serial.printf("RF: SX1278 %s (RBW %lu Hz, %u points, dwell %u us)\n", up ? "OK" : "NOT FOUND - will retry",
                (unsigned long)CFG_RF_RXBW_HZ, (unsigned)RF_N, (unsigned)CFG_RF_DWELL_US);

  for (;;) {
    const int64_t cycleStart = rfNowUs();
    handleRequests();
    bool recovered = false;

    if (g_metrics.rf_state != RF_OK && (s_nextRecoverMs == 0 || (int32_t)(millis() - s_nextRecoverMs) >= 0)) {
      g_metrics.rf_recoveries++;
      g_metrics.rf_state = RF_RECOVERING;
      if (s_radio.recover(s_frf[0])) { g_metrics.rf_state = RF_OK; s_backoffMs = CFG_RF_RECOVER_BACKOFF_MIN_MS; recovered = true; }
      else {
        g_metrics.rf_state = RF_FAULT;
        s_backoffMs = s_backoffMs * 2 > CFG_RF_RECOVER_BACKOFF_MAX_MS ? CFG_RF_RECOVER_BACKOFF_MAX_MS : s_backoffMs * 2;
        s_nextRecoverMs = millis() + s_backoffMs;
      }
      esp_task_wdt_reset();
    }

    if (g_metrics.rf_state == RF_OK) {
      const uint32_t tft0 = g_ctl.tft_ops, sd0 = g_ctl.sd_ops;
      const bool tftB0 = g_ctl.tft_busy, sdB0 = g_ctl.sd_busy;
      const int64_t t0 = rfNowUs();
      uint16_t valid = 0;
      bool ok = sweepOnce(valid);
      const uint32_t durMs = (uint32_t)((rfNowUs() - t0) / 1000);
      uint16_t flags = 0;
      if (g_ctl.tft_ops != tft0 || tftB0 || g_ctl.tft_busy) flags |= SF_TFT_ACTIVE;
      if (g_ctl.sd_ops != sd0 || sdB0 || g_ctl.sd_busy) flags |= SF_SD_ACTIVE;

      if (ok) {
        if (!s_radio.verifyFskRx()) { ok = false; g_metrics.rf_comm_errors++; }
        else if (isFlat()) { if (++s_flatCount >= 3) { ok = false; g_metrics.rf_comm_errors++; s_flatCount = 0; } }
        else s_flatCount = 0;
      }
      s_radio.standby();
      if (!ok) { g_metrics.rf_state = RF_RECOVERING; s_nextRecoverMs = 0; }
      g_metrics.rf_tx_blocked = s_radio.txBlocked();

      if (valid > 0) {
        const int64_t p0 = rfNowUs();
        s_stats.ingest(s_raw, s_det.learning());
        DetectCtx ctx;
        ctx.t_ms = timeMonoMs();
        ctx.epoch_s = timeEpoch();
        ctx.synced = timeSynced();
        s_det.process(s_stats, ctx, s_sink);

        if (timeSynced()) flags |= SF_TIME_SYNCED;
        if (g_ctl.quiet_gate) flags |= SF_QUIET_GATE;
        if (g_ctl.wifi_active) flags |= SF_WIFI_ON;
        if (g_ctl.bl_state) flags |= SF_BL_ON;
        if (s_loraSince) { flags |= SF_LORA_WINDOW; s_loraSince = false; }
        if (valid < RF_N) flags |= SF_PARTIAL;
        if (recovered) flags |= SF_RADIO_RECOVERED;
        if (g_ctl.diag_active) flags |= SF_DIAG;

        s_sweepId++;
        s_snap.id = s_sweepId;
        s_snap.t_ms = ctx.t_ms;
        s_snap.epoch_s = ctx.epoch_s;
        s_snap.dur_ms = (uint16_t)(durMs > 65535 ? 65535 : durMs);
        s_snap.flags = flags;
        s_snap.radio_state = g_metrics.rf_state;
        memcpy(s_snap.raw, s_raw, RF_N);
        s_stats.exportNoiseLvl(s_snap.noise_lvl);
        s_snap.g = s_stats.global();
        s_snap.events_started = g_metrics.events_started;
        s_snap.events_open = s_det.openEvents();
        g_metrics.events_open = s_snap.events_open;

        if ((s_sweepId % CFG_LOG_SWEEP_EVERY_N) == 0) {
          s_rec.magic = SWEEP_RECORD_MAGIC;
          s_rec.sweep_id = s_sweepId;
          s_rec.t_ms = ctx.t_ms;
          s_rec.epoch_s = ctx.epoch_s;
          s_rec.sweep_ms = s_snap.dur_ms;
          s_rec.flags = flags;
          s_rec.cfg_id = 0;
          s_rec.n = (uint8_t)RF_N;
          memcpy(s_rec.raw, s_raw, RF_N);
          s_rec.crc = rfCrc16((const uint8_t*)&s_rec, sizeof(s_rec) - 2);
          if (!g_sweepRing.push(s_rec)) __atomic_fetch_add(&g_metrics.sweeps_not_logged, 1U, __ATOMIC_RELAXED);
        }
        sharedPublishSweep(s_snap);

        const uint32_t proc = (uint32_t)(rfNowUs() - p0);
        g_metrics.proc_us_last = proc;
        if (proc > g_metrics.proc_us_max) g_metrics.proc_us_max = proc;
        g_metrics.sweep_ms_last = durMs;
        if (durMs > g_metrics.sweep_ms_max) g_metrics.sweep_ms_max = durMs;
        const uint32_t avg = g_metrics.sweep_ms_avg_x10;
        g_metrics.sweep_ms_avg_x10 = avg ? (uint32_t)((avg * 7 + durMs * 10) / 8) : durMs * 10;
        g_metrics.sweeps_done = s_sweepId;
        diagOnSweep(s_snap);
      }
    }
    g_metrics.rf_heartbeat++;
    esp_task_wdt_reset();

    // hand the bus-free window to the service task (TFT / SD / I2C) and, in quiet mode, wait for it
    if (g_svcTask) {
      ulTaskNotifyTake(pdTRUE, 0);               // drop a stale acknowledgement
      const uint32_t window = __atomic_add_fetch(&g_ctl.window_seq, 1U, __ATOMIC_RELEASE);
      xTaskNotifyGive(g_svcTask);
      if (g_ctl.quiet_gate) {
        const TickType_t start = xTaskGetTickCount();
        const TickType_t budget = pdMS_TO_TICKS(CFG_QUIET_MAX_WAIT_MS);
        while (__atomic_load_n(&g_ctl.window_done, __ATOMIC_ACQUIRE) != window) {
          const TickType_t elapsed = xTaskGetTickCount() - start;
          if (elapsed >= budget) { g_metrics.quiet_overruns++; break; }
          ulTaskNotifyTake(pdTRUE, budget - elapsed);
        }
      }
    }
    s_cycle++;
    if (CFG_LORA_WINDOW_EVERY_SWEEPS > 0 && g_metrics.rf_state == RF_OK && (s_cycle % CFG_LORA_WINDOW_EVERY_SWEEPS) == 0 &&
        !diagActive())
      runLoraWindow();

    const int64_t next = cycleStart + (int64_t)CFG_RF_SWEEP_PERIOD_MS * 1000;
    if (rfNowUs() < next) rfSleepUntilUs(next);
    else vTaskDelay(1);                           // always yield to idle / lower-priority work
  }
}
