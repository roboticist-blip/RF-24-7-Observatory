#include "data_logger.h"
#include <SD.h>
#include <FS.h>
#include <Preferences.h>
#include "time_manager.h"

namespace {
struct LogStream {
  const char* base; const char* ext; const char* hdr; bool bin;
  File f; bool isOpen; char path[72];
  uint16_t part; uint32_t bytes, maxBytes;
  uint8_t* buf; uint16_t cap, len;
};

uint8_t bufBin[CFG_LOG_BIN_BUF_BYTES], bufEvt[CFG_LOG_EVT_BUF_BYTES], bufEnv[CFG_LOG_ENV_BUF_BYTES],
        bufHlt[CFG_LOG_HLTH_BUF_BYTES], bufTs[CFG_LOG_TS_BUF_BYTES], bufDiag[CFG_LOG_DIAG_BUF_BYTES];
LogStream S[6];
enum { I_BIN = 0, I_EVT, I_ENV, I_HLT, I_TS, I_DIAG };

SPIClass* s_bus = nullptr;
SdState s_state = SD_DISABLED;
char s_dir[40];
uint32_t s_sessionId = 0, s_sessionUid = 0;
uint32_t s_nextRetryMs = 0, s_lastFlushMs = 0;
int64_t s_free = 0;
bool s_haveFree = false;
uint64_t s_sessionBytes = 0;
File s_stressFile;
bool s_stressOpen = false;

const char H_EVT[] = "utc,epoch_s,t_ms,time_sync,frequency_hz,event_type,phase,peak_rssi_dbm,noise_floor_dbm,duration_ms,threshold_dbm,detail\n";
const char H_ENV[] = "utc,epoch_s,t_ms,time_sync,valid,temperature_c,humidity_pct,pressure_hpa\n";
const char H_HLT[] = "utc,epoch_s,t_ms,time_sync,uptime_s,heap_free,heap_min,heap_largest,reset_reason,boot_streak,sweeps,dropped_points,"
                     "dropped_sweeps,sd_errors,sd_free_mb,rf_state,rf_recoveries,rf_comm_errors,sweep_ms_avg,sweep_ms_max,proc_us_max,"
                     "svc_busy_us_max,quiet_overruns,evq_drops,env_failures,events,stack_rf,stack_svc,stack_net,tasks_alive\n";
const char H_TS[] = "utc,epoch_s,t_ms,source\n";
const char H_DIAG[] = "utc,epoch_s,t_ms,time_sync,step,tft,sd,backlight,wifi,sweeps,median_dbm,mean_dbm,active_bins,max_dbm,"
                      "max_bin_delta_db,max_bin_delta_freq_hz,bins_up,bl_switchable\n";

void setSd(SdState s) { s_state = s; g_metrics.sd_state = s; }

int fmtTs(char* o, size_t n, uint64_t t_ms, uint32_t epoch, bool synced) {
  if (epoch && synced) {
    char u[24];
    timeFormatUtc(u, sizeof(u), epoch);
    return snprintf(o, n, "%s,%lu,%llu,1,", u, (unsigned long)epoch, (unsigned long long)t_ms);
  }
  return snprintf(o, n, ",,%llu,0,", (unsigned long long)t_ms);
}

void fillHeader(LogFileHeader& h, uint16_t part) {
  memset(&h, 0, sizeof(h));
  memcpy(h.magic, "RF247BIN", 8);
  h.version = CFG_LOG_FORMAT_VERSION;
  h.header_size = sizeof(LogFileHeader);
  h.session_id = s_sessionId;
  h.session_uid = s_sessionUid;
  h.part_index = part;
  h.n_points = RF_N;
  h.start_freq_hz = RF_START_HZ;
  h.step_hz = RF_STEP_HZ;
  h.rx_bw_hz = CFG_RF_RXBW_HZ;
  h.dwell_us = CFG_RF_DWELL_US;
  h.settle_us = CFG_RF_SETTLE_US;
  h.rssi_samples = CFG_RF_RSSI_SAMPLES;
  h.rssi_smoothing_exp = CFG_RF_RSSI_SMOOTHING_EXP;
  h.quiet_mode = g_ctl.quiet_gate ? 1 : 0;
  h.file_start_t_ms = timeMonoMs();
  h.file_start_epoch_s = timeEpoch();
  h.record_size = sizeof(SweepRecord);
  h.reset_reason = (uint8_t)healthBoot().reason;
}

void closeStream(LogStream& s, bool verify) {
  if (!s.isOpen) return;
  s.f.flush();
  s.f.close();
  s.isOpen = false;
  if (verify) {                                  // integrity check: size on card == bytes we wrote
    File c = SD.open(s.path, FILE_READ);
    if (c) { if ((uint32_t)c.size() != s.bytes) g_metrics.log_integrity_errors++; c.close(); }
    else g_metrics.log_integrity_errors++;
  }
}

void sdFail(uint32_t nowMs) {
  g_metrics.sd_errors++;
  for (uint8_t i = 0; i < 6; i++) if (S[i].isOpen) { S[i].f.close(); S[i].isOpen = false; }
  if (s_stressOpen) { s_stressFile.close(); s_stressOpen = false; }
  setSd(SD_ERROR);
  s_nextRetryMs = nowMs + (uint32_t)CFG_LOG_SD_RETRY_S * 1000UL;
}

bool openStream(LogStream& s) {
  if (s.isOpen) return true;
  uint16_t p = s.part;
  for (;;) {                                      // never reuse an existing file name
    if (++p > 9999) return false;
    snprintf(s.path, sizeof(s.path), "%s/%s_%04u.%s", s_dir, s.base, (unsigned)p, s.ext);
    if (!SD.exists(s.path)) break;
  }
  s.f = SD.open(s.path, FILE_APPEND);
  if (!s.f) return false;
  size_t hn, want;
  if (s.bin) { LogFileHeader h; fillHeader(h, p); want = sizeof(h); hn = s.f.write((const uint8_t*)&h, want); }
  else { want = strlen(s.hdr); hn = s.f.write((const uint8_t*)s.hdr, want); }
  if (hn != want) { s.f.close(); return false; }
  s.part = p;
  s.bytes = (uint32_t)hn;
  s.isOpen = true;
  return true;
}

bool flushStream(LogStream& s) {
  if (!s.len) return true;
  if (s_state != SD_OK) return false;
  const uint32_t now = millis();
  g_ctl.sd_busy = true;
  bool ok = true;
  if (!s.isOpen && !openStream(s)) ok = false;
  if (ok && s.bytes + s.len > s.maxBytes && s.bytes > 64) {       // rotation
    closeStream(s, true);
    ok = openStream(s);
  }
  if (ok) {
    const size_t w = s.f.write(s.buf, s.len);
    if (w != s.len) ok = false;
    else {
      s.f.flush();
      s.bytes += (uint32_t)w;
      s_sessionBytes += w;
      s_free -= (int64_t)w;
      g_metrics.sd_bytes_written += (uint32_t)w;
      s.len = 0;
    }
  }
  g_ctl.sd_ops++;
  g_ctl.sd_busy = false;
  if (!ok) {
    g_metrics.log_bytes_lost += s.len;            // in-flight buffer cannot be trusted after a failed write
    s.len = 0;
    sdFail(now);
    return false;
  }
  g_metrics.sd_free_mb = (uint32_t)(s_free > 0 ? s_free / 1048576LL : 0);
  if (s_free < (int64_t)CFG_LOG_MIN_FREE_BYTES) setSd(SD_FULL);
  if (CFG_LOG_MAX_SESSION_BYTES && s_sessionBytes >= CFG_LOG_MAX_SESSION_BYTES) setSd(SD_FULL);
  return true;
}

bool append(LogStream& s, const void* d, size_t n) {
  if (n > s.cap) return false;
  if (s.len + n > s.cap) {
    if (!flushStream(s)) return false;
  }
  memcpy(s.buf + s.len, d, n);
  s.len = (uint16_t)(s.len + n);
  return true;
}

bool appendLine(LogStream& s, const char* line, int n) {
  if (n <= 0) return false;
  if (append(s, line, (size_t)n)) return true;
  g_metrics.log_bytes_lost += (uint32_t)n;       // CSV line dropped (SD unavailable and buffer full)
  return false;
}

int formatEvent(const RfEvent& e, char* l, size_t cap) {
  int n = fmtTs(l, cap, e.t_ms, e.epoch_s, e.synced != 0);
  n += snprintf(l + n, cap - n, "%lu,%s,%s,%.1f,%.1f,%lu,", (unsigned long)e.freq_hz, rfEvName(e.type), rfPhaseName(e.phase),
                rfLvlToDbm(e.peak_lvl), rfLvlToDbm(e.noise_lvl), (unsigned long)e.dur_ms);
  if (e.thr_lvl) n += snprintf(l + n, cap - n, "%.1f,", rfLvlToDbm(e.thr_lvl));
  else n += snprintf(l + n, cap - n, ",");
  n += snprintf(l + n, cap - n, "%s\n", e.detail);
  return n;
}

int formatDiag(const DiagRow& r, char* l, size_t cap) {
  int n = fmtTs(l, cap, r.t_ms, r.epoch_s, r.epoch_s != 0);
  n += snprintf(l + n, cap - n, "%s,%u,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.2f,%lu,%u,%u\n", r.step, r.tft, r.sd, r.bl, r.wifi, r.n,
                (r.med_q4 / 16.0f - 255) * 0.5f, (r.mean_q4 / 16.0f - 255) * 0.5f, r.active_x10 / 10.0f, ((int)r.max_lvl - 255) * 0.5f,
                r.dmax_q4 / 32.0f, (unsigned long)rfBinFreqHz(r.dmax_bin), r.bins_up, r.bl_switchable);
  return n;
}

void initStream(LogStream& s, const char* base, const char* ext, const char* hdr, bool bin, uint8_t* buf, uint16_t cap, uint32_t maxB) {
  s = LogStream{};
  s.base = base; s.ext = ext; s.hdr = hdr; s.bin = bin; s.buf = buf; s.cap = cap; s.maxBytes = maxB;
}

void writeSessionInfo() {
  char p[64];
  snprintf(p, sizeof(p), "%s/session.txt", s_dir);
  File f = SD.open(p, FILE_APPEND);
  if (!f) return;
  char u[24];
  timeFormatUtc(u, sizeof(u), timeEpoch());
  f.printf("--- mount at uptime_ms=%llu utc=%s synced=%d\n", (unsigned long long)timeMonoMs(), u[0] ? u : "unsynchronized", timeSynced());
  f.printf("firmware=%s %s log_format=%d session_id=%lu session_uid=%08lX\n", CFG_FW_NAME, CFG_FW_VERSION, CFG_LOG_FORMAT_VERSION,
           (unsigned long)s_sessionId, (unsigned long)s_sessionUid);
  f.printf("reset_reason=%s boot_streak=%u boot_count=%lu\n", healthBoot().reasonName, healthBoot().streak, (unsigned long)healthBoot().bootCount);
  f.printf("sweep=%lu..%lu Hz step=%lu points=%u rbw=%lu dwell_us=%d settle_us=%d samples=%d period_ms=%d quiet=%d\n",
           (unsigned long)RF_START_HZ, (unsigned long)RF_STOP_HZ, (unsigned long)RF_STEP_HZ, (unsigned)RF_N, (unsigned long)CFG_RF_RXBW_HZ,
           CFG_RF_DWELL_US, CFG_RF_SETTLE_US, CFG_RF_RSSI_SAMPLES, CFG_RF_SWEEP_PERIOD_MS, (int)g_ctl.quiet_gate);
  f.close();
}

bool ensureDir() {
  if (s_dir[0]) { return SD.exists(s_dir) || SD.mkdir(s_dir); }   // remount: keep the same session folder
  if (!SD.exists(CFG_LOG_DIR_ROOT) && !SD.mkdir(CFG_LOG_DIR_ROOT)) return false;
  for (uint32_t tries = 0; tries < 10000; tries++) {
    snprintf(s_dir, sizeof(s_dir), "%s/S%06lu", CFG_LOG_DIR_ROOT, (unsigned long)s_sessionId);
    if (!SD.exists(s_dir)) return SD.mkdir(s_dir);
    s_sessionId++;                                // never reuse an existing session folder
  }
  return false;
}

bool tryMount(uint32_t nowMs) {
  SD.end();
  if (!SD.begin(PIN_SD_CS, *s_bus, CFG_SD_SPI_HZ, "/sd", 5, false) || SD.cardType() == CARD_NONE) {   // never auto-format
    SD.end();
    if (s_state != SD_ERROR) setSd(SD_ABSENT);
    s_nextRetryMs = nowMs + (uint32_t)CFG_LOG_SD_RETRY_S * 1000UL;
    return false;
  }
  if (!s_haveFree) {                              // slow on big cards: once per boot
    s_free = (int64_t)SD.totalBytes() - (int64_t)SD.usedBytes();
    s_haveFree = true;
  }
  if (!ensureDir()) { setSd(SD_ERROR); s_nextRetryMs = nowMs + (uint32_t)CFG_LOG_SD_RETRY_S * 1000UL; g_metrics.sd_errors++; return false; }
  g_metrics.session_id = s_sessionId;
  setSd(s_free < (int64_t)CFG_LOG_MIN_FREE_BYTES ? SD_FULL : SD_OK);
  g_metrics.sd_free_mb = (uint32_t)(s_free > 0 ? s_free / 1048576LL : 0);
  if (s_state == SD_OK) writeSessionInfo();
  return s_state == SD_OK;
}

void discardAll() {
  while (g_sweepRing.peek()) { g_sweepRing.pop(); __atomic_fetch_add(&g_metrics.sweeps_not_logged, 1U, __ATOMIC_RELAXED); }
  RfEvent e; DiagRow d; TimeSyncEvent t;
  while (xQueueReceive(g_eventQueue, &e, 0) == pdTRUE) {}
  while (xQueueReceive(g_diagQueue, &d, 0) == pdTRUE) {}
  while (timePeekSyncEvent(t)) timePopSyncEvent();
  for (uint8_t i = 0; i < 6; i++) S[i].len = 0;
}
}  // namespace

SdState DataLogger::state() const { return s_state; }
const char* DataLogger::dir() const { return s_dir; }

void DataLogger::begin(SPIClass& bus, bool enabled) {
  s_bus = &bus;
  s_dir[0] = 0;
  initStream(S[I_BIN], "sweeps", "bin", nullptr, true, bufBin, sizeof(bufBin), CFG_LOG_MAX_BIN_FILE_BYTES);
  initStream(S[I_EVT], "events", "csv", H_EVT, false, bufEvt, sizeof(bufEvt), CFG_LOG_MAX_CSV_FILE_BYTES);
  initStream(S[I_ENV], "env", "csv", H_ENV, false, bufEnv, sizeof(bufEnv), CFG_LOG_MAX_CSV_FILE_BYTES);
  initStream(S[I_HLT], "health", "csv", H_HLT, false, bufHlt, sizeof(bufHlt), CFG_LOG_MAX_CSV_FILE_BYTES);
  initStream(S[I_TS], "timesync", "csv", H_TS, false, bufTs, sizeof(bufTs), CFG_LOG_MAX_CSV_FILE_BYTES);
  initStream(S[I_DIAG], "diag", "csv", H_DIAG, false, bufDiag, sizeof(bufDiag), CFG_LOG_MAX_CSV_FILE_BYTES);

  s_sessionUid = esp_random();                    // unique even if NVS is erased
  Preferences pr;
  if (pr.begin("rf247", false)) { s_sessionId = pr.getUInt("sess", 0) + 1; pr.putUInt("sess", s_sessionId); pr.end(); }
  else s_sessionId = (esp_random() & 0xFFFF) + 100000UL;
  g_metrics.session_id = s_sessionId;
  g_metrics.session_uid = s_sessionUid;

  setSd(SD_DISABLED);
  if (!enabled || !CFG_ENABLE_SD) return;
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  tryMount(millis());
}

void DataLogger::logEnv(const EnvData& e) {
  if (s_state != SD_OK) return;
  char l[128];
  int n = fmtTs(l, sizeof(l), e.t_ms, e.epoch_s, e.epoch_s != 0);
  if (e.valid) n += snprintf(l + n, sizeof(l) - n, "1,%.2f,,%.2f\n", e.t_c100 / 100.0f, e.p_pa / 100.0f);
  else n += snprintf(l + n, sizeof(l) - n, "0,,,\n");
  appendLine(S[I_ENV], l, n);
}

void DataLogger::logHealth(const HealthSnapshot& h) {
  if (s_state != SD_OK) return;
  char l[320];
  const uint64_t t = timeMonoMs();
  int n = fmtTs(l, sizeof(l), t, timeEpoch(), timeSynced());
  n += snprintf(l + n, sizeof(l) - n, "%lu,%lu,%lu,%lu,%s,%u,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu.%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u\n",
                (unsigned long)h.uptime_s, (unsigned long)h.heap_free, (unsigned long)h.heap_min, (unsigned long)h.heap_largest,
                healthBoot().reasonName, healthBoot().streak, (unsigned long)g_metrics.sweeps_done, (unsigned long)g_metrics.points_dropped,
                (unsigned long)(g_metrics.sweeps_not_logged), (unsigned long)g_metrics.sd_errors,
                (unsigned long)g_metrics.sd_free_mb, (unsigned)g_metrics.rf_state, (unsigned long)g_metrics.rf_recoveries,
                (unsigned long)g_metrics.rf_comm_errors, (unsigned long)(g_metrics.sweep_ms_avg_x10 / 10),
                (unsigned long)(g_metrics.sweep_ms_avg_x10 % 10), (unsigned long)g_metrics.sweep_ms_max, (unsigned long)g_metrics.proc_us_max,
                (unsigned long)g_metrics.svc_busy_us_max, (unsigned long)g_metrics.quiet_overruns, (unsigned long)g_metrics.evq_drops,
                (unsigned long)g_metrics.env_failures, (unsigned long)g_metrics.events_started, (unsigned long)h.stack_rf,
                (unsigned long)h.stack_svc, (unsigned long)h.stack_net, (unsigned)h.tasks_alive);
  appendLine(S[I_HLT], l, n);
}

void DataLogger::flushAll() {
  for (uint8_t i = 0; i < 6; i++) flushStream(S[i]);
}

void DataLogger::service(uint64_t nowMs64, bool force) {
  const uint32_t nowMs = (uint32_t)nowMs64;
  if (s_state == SD_DISABLED) { discardAll(); return; }
  if ((s_state == SD_ABSENT || s_state == SD_ERROR) && (int32_t)(nowMs - s_nextRetryMs) >= 0) tryMount(nowMs);
  if (s_state == SD_ABSENT || s_state == SD_FULL) { discardAll(); return; }
  if (s_state != SD_OK) return;                    // SD_ERROR: keep buffering (bounded), retry later

  const SweepRecord* r;
  while ((r = g_sweepRing.peek()) != nullptr) {
    if (!append(S[I_BIN], r, sizeof(SweepRecord))) break;
    g_sweepRing.pop();
    g_metrics.sweeps_logged++;
    if (s_state != SD_OK) break;
  }
  char l[256];
  RfEvent e;
  while (xQueuePeek(g_eventQueue, &e, 0) == pdTRUE) {
    const int n = formatEvent(e, l, sizeof(l));
    if (!append(S[I_EVT], l, (size_t)n)) break;
    xQueueReceive(g_eventQueue, &e, 0);
  }
  DiagRow d;
  while (xQueuePeek(g_diagQueue, &d, 0) == pdTRUE) {
    const int n = formatDiag(d, l, sizeof(l));
    if (!append(S[I_DIAG], l, (size_t)n)) break;
    xQueueReceive(g_diagQueue, &d, 0);
  }
  TimeSyncEvent t;
  while (timePeekSyncEvent(t)) {
    char u[24];
    timeFormatUtc(u, sizeof(u), t.epoch_s);
    const int n = snprintf(l, sizeof(l), "%s,%lu,%llu,%s\n", u, (unsigned long)t.epoch_s, (unsigned long long)t.t_ms, timeSourceName(t.source));
    if (!append(S[I_TS], l, (size_t)n)) break;
    timePopSyncEvent();
  }
  if (force || (uint32_t)(nowMs - s_lastFlushMs) >= (uint32_t)CFG_LOG_FLUSH_INTERVAL_S * 1000UL) {
    s_lastFlushMs = nowMs;
    flushAll();
  }
}

void DataLogger::stress(bool active) {
  char p[64];
  snprintf(p, sizeof(p), "%s/diag_stress.tmp", s_dir);
  if (!active) {
    if (s_stressOpen) { s_stressFile.close(); s_stressOpen = false; SD.remove(p); }
    return;
  }
  if (s_state != SD_OK) return;
  if (!s_stressOpen) {
    s_stressFile = SD.open(p, FILE_WRITE);         // our own scratch file: truncation is intended
    s_stressOpen = (bool)s_stressFile;
    if (!s_stressOpen) return;
  }
  static uint8_t pat[512];
  static bool init = false;
  if (!init) { memset(pat, 0xA5, sizeof(pat)); init = true; }
  g_ctl.sd_busy = true;
  if (s_stressFile.position() > 262144) s_stressFile.seek(0);
  for (uint8_t i = 0; i < 4; i++) s_stressFile.write(pat, sizeof(pat));
  s_stressFile.flush();
  g_ctl.sd_ops++;
  g_ctl.sd_busy = false;
}
