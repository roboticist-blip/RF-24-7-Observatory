#include "diagnostics.h"
#include "time_manager.h"

namespace {
struct StepDef { const char* name; bool tft, sd, bl, wifi; };
const StepDef kSteps[] = {
  {"QUIET",     false, false, false, false},
#if CFG_ENABLE_DISPLAY
  {"TFT",       true,  false, false, false},
#endif
#if CFG_ENABLE_SD
  {"SD",        false, true,  false, false},
#endif
#if CFG_TFT_BL_SWITCHABLE
  {"BACKLIGHT", false, false, true,  false},
#endif
#if CFG_DIAG_WIFI_STEP
  {"WIFI",      false, false, false, true },
#endif
  {"ALL",       true,  true,  true,  true },
};
const uint8_t kNSteps = (uint8_t)(sizeof(kSteps) / sizeof(kSteps[0]));

bool s_active = false;
uint8_t s_step = 0;
uint16_t s_incl = 0, s_attempts = 0, s_skip = 0;
uint32_t s_binSum[RF_N];
uint16_t s_base[RF_N];            // baseline per-bin mean (level x16)
bool s_haveBase = false;
uint32_t s_sumMed = 0, s_sumMean = 0, s_sumAct = 0;
uint8_t s_maxLvl = 0;

void applyStep(const StepDef& d) {
  g_ctl.quiet_gate = !(d.tft || d.sd);              // steps that exercise TFT/SD must overlap the sweeps
  g_ctl.tft_enable = d.tft;
  g_ctl.sd_stress = d.sd;
  g_ctl.bl_enable = d.bl;
  g_ctl.wifi_enable = d.wifi;
}

void beginStep() {
  s_incl = 0; s_attempts = 0; s_skip = CFG_DIAG_SKIP_SWEEPS;
  s_sumMed = s_sumMean = s_sumAct = 0; s_maxLvl = 0;
  memset(s_binSum, 0, sizeof(s_binSum));
  applyStep(kSteps[s_step]);
  Serial.printf("DIAG step %u/%u: %s (tft=%d sd=%d bl=%d wifi=%d)\n", s_step + 1, kNSteps, kSteps[s_step].name,
                kSteps[s_step].tft, kSteps[s_step].sd, kSteps[s_step].bl, kSteps[s_step].wifi);
}

void finishStep() {
  const StepDef& d = kSteps[s_step];
  DiagRow r;
  memset(&r, 0, sizeof(r));
  strncpy(r.step, d.name, sizeof(r.step) - 1);
  r.tft = d.tft; r.sd = d.sd; r.bl = d.bl; r.wifi = d.wifi; r.bl_switchable = CFG_TFT_BL_SWITCHABLE;
  r.n = s_incl;
  if (s_incl) {
    r.med_q4 = (uint16_t)(s_sumMed / s_incl);
    r.mean_q4 = (uint16_t)(s_sumMean / s_incl);
    r.active_x10 = (uint16_t)(s_sumAct * 10U / s_incl);
    r.max_lvl = s_maxLvl;
    uint16_t avg[RF_N];
    for (uint16_t i = 0; i < RF_N; i++) avg[i] = (uint16_t)(s_binSum[i] * 16U / s_incl);
    if (s_step == 0) { memcpy(s_base, avg, sizeof(avg)); s_haveBase = true; }
    else if (s_haveBase) {
      int16_t best = -32768; uint16_t bin = 0, up = 0;
      for (uint16_t i = 0; i < RF_N; i++) {
        const int16_t dl = (int16_t)((int)avg[i] - (int)s_base[i]);
        if (dl > best) { best = dl; bin = i; }
        if (dl >= (CFG_DIAG_BIN_DELTA_LVL << 4)) up++;
      }
      r.dmax_q4 = best; r.dmax_bin = bin; r.bins_up = up;
    }
  }
  r.t_ms = timeMonoMs();
  r.epoch_s = timeEpoch();
  xQueueSend(g_diagQueue, &r, 0);
  Serial.printf("DIAG %-9s n=%u median=%.1f dBm mean=%.1f dBm active=%.1f max=%.1f dBm dBinMax=%+.2f dB @%u Hz binsUp=%u\n",
                r.step, r.n, (r.med_q4 / 16.0f - 255) * 0.5f, (r.mean_q4 / 16.0f - 255) * 0.5f, r.active_x10 / 10.0f,
                ((int)r.max_lvl - 255) * 0.5f, r.dmax_q4 / 32.0f, (unsigned)rfBinFreqHz(r.dmax_bin), r.bins_up);
}
}  // namespace

bool diagActive() { return s_active; }

void diagRestoreDefaults() {
  g_ctl.quiet_gate = CFG_QUIET_ACQUISITION;
  g_ctl.tft_enable = g_ctl.allow_display;
  g_ctl.sd_stress = false;
  g_ctl.bl_enable = true;
  g_ctl.wifi_enable = (CFG_ENABLE_WIFI_DASHBOARD || CFG_ENABLE_NTP_AT_BOOT) && g_ctl.allow_wifi;
  g_ctl.diag_active = 0;
}

void diagStart() {
  s_active = true; s_step = 0; s_haveBase = false;
  g_ctl.diag_active = 1;
  Serial.println("DIAG start (6 steps max); compare each step with QUIET");
  beginStep();
}

void diagAbort() {
  if (!s_active) return;
  s_active = false;
  diagRestoreDefaults();
  Serial.println("DIAG aborted");
}

void diagOnSweep(const SweepSnapshot& s) {
  if (!s_active) return;
  s_attempts++;
  const StepDef& d = kSteps[s_step];
  const bool match = ((s.flags & SF_TFT_ACTIVE) != 0) == d.tft && ((s.flags & SF_SD_ACTIVE) != 0) == d.sd &&
                     ((s.flags & SF_WIFI_ON) != 0) == d.wifi &&
                     (!CFG_TFT_BL_SWITCHABLE || ((s.flags & SF_BL_ON) != 0) == d.bl) &&
                     !(s.flags & (SF_PARTIAL | SF_LORA_WINDOW | SF_RADIO_RECOVERED));
  if (s_skip) { if (s_skip) s_skip--; }
  else if (match && s.g.n_valid == RF_N) {
    s_incl++;
    s_sumMed += s.g.med_q8 >> 4;
    uint32_t sum = 0, cnt = 0;
    for (uint16_t i = 0; i < RF_N; i++) {
      if (s.raw[i] == RF_RAW_INVALID) continue;
      const uint8_t lv = rfRawToLvl(s.raw[i]);
      s_binSum[i] += lv;
      sum += lv; cnt++;
    }
    s_sumMean += cnt ? (sum * 16U / cnt) : 0;
    s_sumAct += s.g.n_active;
    if (s.g.peak_sweep_lvl > s_maxLvl) s_maxLvl = s.g.peak_sweep_lvl;
  }
  if (s_incl >= CFG_DIAG_STEP_SWEEPS || s_attempts > 4U * CFG_DIAG_STEP_SWEEPS) {
    finishStep();
    if (++s_step >= kNSteps) {
      s_active = false;
      diagRestoreDefaults();
      Serial.println("DIAG complete: see diag_XXXX.csv in the session folder");
    } else {
      beginStep();
    }
  }
}
