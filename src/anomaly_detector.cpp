#include "anomaly_detector.h"
#include <math.h>

void StatisticalDetector::reset() {
  memset(det_, 0, sizeof(det_));
  memset(&gNoise_, 0, sizeof(gNoise_));
  memset(&gWide_, 0, sizeof(gWide_));
  memset(&gAct_, 0, sizeof(gAct_));
  sweeps_ = 0;
  openCount_ = 0;
  baseMed_q8_ = 0;
  occFast_q8_ = occSlow_q8_ = 0;
}

void StatisticalDetector::emit(EventSink& sink, const DetectCtx& c, uint8_t type, uint8_t phase,
                               uint32_t freq, uint8_t peak, uint8_t noise, uint8_t thr, uint32_t dur) {
  RfEvent e;
  memset(&e, 0, sizeof(e));
  e.t_ms = c.t_ms;
  e.epoch_s = c.epoch_s;
  e.synced = c.synced ? 1 : 0;
  e.freq_hz = freq;
  e.dur_ms = dur;
  e.type = type;
  e.phase = phase;
  e.peak_lvl = peak;
  e.noise_lvl = noise;
  e.thr_lvl = thr;
  sink.push(e);
}

uint8_t StatisticalDetector::thrLvl(const BinStat& b) {
  const uint32_t var = b.var_q4sq < CFG_STAT_MIN_VAR ? CFG_STAT_MIN_VAR : b.var_q4sq;
  const float sigma = sqrtf((float)var) / 16.0f;                 // in level units
  float m = CFG_DET_Z_OPEN * sigma;
  if (m < CFG_STAT_OPEN_MARGIN) m = CFG_STAT_OPEN_MARGIN;
  const int t = (int)RfStats::noiseLvl(b) + (int)(m + 0.5f);
  return (uint8_t)(t > 255 ? 255 : t);
}

bool StatisticalDetector::neighbourOpen(uint16_t i) const {
  const int lo = (int)i - CFG_DET_MERGE_BINS, hi = (int)i + CFG_DET_MERGE_BINS;
  for (int j = lo; j <= hi; j++) {
    if (j < 0 || j >= (int)RF_N || j == (int)i) continue;
    if (det_[j].open) return true;
  }
  return false;
}

void StatisticalDetector::globalEvent(GlobEv& ev, bool condOpen, bool condKeep, uint8_t persist, uint8_t type,
                                      uint8_t peak, uint8_t noise, uint8_t thr, uint32_t freq,
                                      const DetectCtx& c, EventSink& sink) {
  if (!ev.open) {
    if (condOpen) {
      if (ev.cnt < 255) ev.cnt++;
      if (ev.cnt >= persist) {
        ev.open = 1; ev.quiet = 0; ev.peak = peak; ev.t_open = c.t_ms; ev.t_last = c.t_ms;
        openCount_++;
        emit(sink, c, type, PH_START, freq, peak, noise, thr, 0);
      }
    } else {
      ev.cnt = 0;
    }
  } else {
    if (peak > ev.peak) ev.peak = peak;
    if (condKeep) { ev.quiet = 0; ev.t_last = c.t_ms; }
    else if (++ev.quiet >= CFG_DET_CLOSE_QUIET) {
      ev.open = 0; ev.cnt = 0;
      if (openCount_) openCount_--;
      emit(sink, c, type, PH_END, freq, ev.peak, noise, thr, (uint32_t)(ev.t_last - ev.t_open));
    }
  }
}

void StatisticalDetector::process(const RfStats& st, const DetectCtx& c, EventSink& sink) {
  const bool learn = sweeps_ < CFG_BASELINE_SWEEPS;
  const GlobalStats& g = st.global();
  sweeps_++;

  for (uint16_t i = 0; i < RF_N; i++) {
    BinDet& d = det_[i];
    const BinStat& b = st.bin(i);
    const bool act = (b.flags & BF_ACTIVE) != 0;
    d.hist = (uint16_t)((d.hist << 1) | (act ? 1 : 0));
    if (act) {
      if (d.run == 0) d.t_first = c.t_ms;
      if (d.run < 255) d.run++;
      d.t_last = c.t_ms;
      d.lastpk = b.cur;
    } else {
      d.run = 0;
    }
    if (learn) continue;

    if (!d.open) {
      uint8_t type = 0;
      if (act) {
        if ((b.flags & BF_JUMP) && (b.flags & BF_ZHIT)) type = EV_RSSI_JUMP;
        else if (d.run >= CFG_DET_PERSIST_SWEEPS) type = EV_PERSISTENT_NB;
      }
      if (!type) {
        const uint16_t trans = (uint16_t)__builtin_popcount((unsigned)((d.hist ^ (d.hist >> 1)) & 0x7FFF));
        if (trans >= CFG_DET_BURST_MIN_TRANS && __builtin_popcount(d.hist) >= 2) type = EV_BURSTS;
      }
      // while a wideband episode is in progress, per-bin events would only duplicate it
      if (type && g.n_active < CFG_DET_WIDE_BINS && !neighbourOpen(i)) {
        d.open = 1; d.type = type; d.quiet = 0;
        d.peak = act ? b.cur : d.lastpk;
        if (d.run == 0) d.t_first = d.t_last;     // bursts: start at the last observed activity
        openCount_++;
        emit(sink, c, type, PH_START, rfBinFreqHz(i), d.peak, RfStats::noiseLvl(b), thrLvl(b), 0);
      }
    } else {
      if (b.cur > d.peak) d.peak = b.cur;
      if (act) d.quiet = 0;
      else if (++d.quiet >= CFG_DET_CLOSE_QUIET) {
        d.open = 0;
        if (openCount_) openCount_--;
        const uint32_t dur = d.t_last >= d.t_first ? (uint32_t)(d.t_last - d.t_first) : 0;
        emit(sink, c, d.type, PH_END, rfBinFreqHz(i), d.peak, RfStats::noiseLvl(b), thrLvl(b), dur);
        d.quiet = 0;
      }
    }
  }

  if (sweeps_ == CFG_BASELINE_SWEEPS) {            // end of baseline learning
    baseMed_q8_ = g.med_q8;
    occFast_q8_ = occSlow_q8_ = (uint16_t)((uint16_t)g.occ_inst_pct << 8);
  }
  if (learn) return;

  const uint32_t center = (RF_START_HZ + RF_STOP_HZ) / 2;
  const uint8_t medLvl = (uint8_t)(g.med_q8 >> 8);

  // sustained noise-floor rise (robust sweep median vs baseline)
  const int32_t diff = (int32_t)g.med_q8 - (int32_t)baseMed_q8_;
  const uint8_t baseLvl = (uint8_t)(baseMed_q8_ >> 8);
  globalEvent(gNoise_, diff >= ((int32_t)CFG_DET_NF_RISE_OPEN << 8), diff >= ((int32_t)CFG_DET_NF_RISE_CLOSE << 8),
              CFG_DET_NF_PERSIST, EV_NOISE_FLOOR_RISE, medLvl, baseLvl,
              (uint8_t)(baseLvl + CFG_DET_NF_RISE_OPEN), center, c, sink);

  // sustained wideband (interference-like) activity
  globalEvent(gWide_, g.n_active >= CFG_DET_WIDE_BINS, g.n_active >= (CFG_DET_WIDE_BINS / 2), CFG_DET_WIDE_PERSIST,
              EV_WIDEBAND_SUSTAINED, g.peak_sweep_lvl, medLvl, 0, rfBinFreqHz(g.peak_sweep_bin), c, sink);

  // occupancy shift: fast vs slow EMA (slow one frozen while the event is open)
  occFast_q8_ = (uint16_t)((int32_t)occFast_q8_ + ((((int32_t)g.occ_inst_pct << 8) - (int32_t)occFast_q8_) >> 2));
  int32_t od = (int32_t)occFast_q8_ - (int32_t)occSlow_q8_;
  if (od < 0) od = -od;
  globalEvent(gAct_, od >= ((int32_t)CFG_DET_ACT_SHIFT_PCT << 8), od >= ((int32_t)(CFG_DET_ACT_SHIFT_PCT / 2) << 8),
              CFG_DET_ACT_PERSIST, EV_ACTIVITY_SHIFT, g.peak_sweep_lvl, medLvl, 0, center, c, sink);
  if (!gAct_.open)
    occSlow_q8_ = (uint16_t)((int32_t)occSlow_q8_ + ((((int32_t)g.occ_inst_pct << 8) - (int32_t)occSlow_q8_) >> 8));
}
