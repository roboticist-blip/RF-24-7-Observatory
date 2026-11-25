#include "rf_statistics.h"

void RfStats::reset() {
  memset(bins_, 0, sizeof(bins_));
  memset(&g_, 0, sizeof(g_));
  for (uint16_t i = 0; i < RF_N; i++) bins_[i].minv = 255;
  sweeps_ = 0;
}

void RfStats::ingest(const uint8_t* raw, bool learning) {
  uint8_t hist[256];
  memset(hist, 0, sizeof(hist));            // RF_N <= 255, so uint8 counters cannot overflow
  uint16_t nValid = 0, nAct = 0;
  uint8_t pk = 0;
  uint16_t pkBin = 0;
  uint32_t occSum = 0;
  const bool first = (sweeps_ == 0);

  for (uint16_t i = 0; i < RF_N; i++) {
    BinStat& b = bins_[i];
    if (raw[i] == RF_RAW_INVALID) {         // missing point: hold state, no update
      b.flags &= (uint8_t)~(BF_ZHIT | BF_JUMP | BF_RISE);
      occSum += b.occ_q16;
      continue;
    }
    const uint8_t x = (uint8_t)(255 - raw[i]);
    const int32_t xq8 = (int32_t)x << 8;
    if (first || b.noise_q8 == 0) {
      b.noise_q8 = b.avg_q8 = (uint16_t)xq8;
      b.var_q4sq = CFG_STAT_INIT_VAR;
      b.cur = b.peak = b.minv = x;
      b.flags = 0;
    }
    const int32_t devq4 = (xq8 - (int32_t)b.noise_q8) >> 4;
    const uint32_t dev2 = (uint32_t)(devq4 * devq4);
    const uint32_t var = b.var_q4sq < CFG_STAT_MIN_VAR ? CFG_STAT_MIN_VAR : b.var_q4sq;
    const bool zhit = devq4 > 0 && dev2 >= (uint32_t)CFG_DET_Z_OPEN_SQ * var;
    const bool was = (b.flags & BF_ACTIVE) != 0;
    const bool active = was ? (devq4 >= ((int32_t)CFG_STAT_KEEP_MARGIN << 4))
                            : (zhit && devq4 >= ((int32_t)CFG_STAT_OPEN_MARGIN << 4));
    uint8_t f = 0;
    if (active) f |= BF_ACTIVE;
    if (zhit) f |= BF_ZHIT;
    if (active && !was) { f |= BF_RISE; b.act_count++; }
    if (!first && ((int)x - (int)b.cur) >= CFG_DET_JUMP_LVL) f |= BF_JUMP;
    b.flags = f;
    b.cur = x;
    if (x > b.peak) b.peak = x;
    if (x < b.minv) b.minv = x;

    b.avg_q8 = (uint16_t)((int32_t)b.avg_q8 + ((xq8 - (int32_t)b.avg_q8) >> CFG_STAT_AVG_SHIFT));

    if (learning || !active) {              // background estimate only learns from non-active samples
      const int shift = learning ? CFG_STAT_LEARN_SHIFT : CFG_STAT_NOISE_SHIFT;
      int32_t dn = xq8 - (int32_t)b.noise_q8;
      int32_t dc = devq4;
      if (learning) {
        const int32_t c8 = (int32_t)CFG_STAT_LEARN_CLIP << 8, c4 = (int32_t)CFG_STAT_LEARN_CLIP << 4;
        if (dn > c8) dn = c8; else if (dn < -c8) dn = -c8;
        if (dc > c4) dc = c4; else if (dc < -c4) dc = -c4;
      }
      b.noise_q8 = (uint16_t)((int32_t)b.noise_q8 + (dn >> shift));
      int32_t v = (int32_t)b.var_q4sq;
      v += ((dc * dc) - v) >> shift;
      b.var_q4sq = v < 0 ? 0 : (uint32_t)v;
    }
    b.occ_q16 = (uint16_t)((int32_t)b.occ_q16 + ((((active) ? 65535 : 0) - (int32_t)b.occ_q16) >> CFG_STAT_OCC_SHIFT));

    nValid++;
    hist[x]++;
    if (active) nAct++;
    occSum += b.occ_q16;
    if (x >= pk) { pk = x; pkBin = i; }
    if (x > g_.peak_session_lvl) g_.peak_session_lvl = x;
  }

  if (nValid) {                              // robust sweep floor: median level
    const uint16_t target = (uint16_t)((nValid + 1) / 2);
    uint16_t cum = 0;
    uint8_t med = 0;
    for (uint16_t v = 0; v < 256; v++) { cum += hist[v]; if (cum >= target) { med = (uint8_t)v; break; } }
    const int32_t m8 = (int32_t)med << 8;
    if (g_.med_q8 == 0) g_.med_q8 = (uint16_t)m8;
    else g_.med_q8 = (uint16_t)((int32_t)g_.med_q8 + ((m8 - (int32_t)g_.med_q8) >> CFG_STAT_MED_SHIFT));
  }
  g_.n_valid = nValid;
  g_.peak_sweep_lvl = pk;
  g_.peak_sweep_bin = pkBin;
  g_.n_active = nAct;
  g_.occ_inst_pct = nValid ? (uint8_t)((uint32_t)nAct * 100U / nValid) : 0;
  g_.occ_avg_pct = (uint8_t)(occSum * 100U / (65535UL * RF_N));
  sweeps_++;
}

void RfStats::exportNoiseLvl(uint8_t* out) const {
  for (uint16_t i = 0; i < RF_N; i++) out[i] = noiseLvl(bins_[i]);
}
