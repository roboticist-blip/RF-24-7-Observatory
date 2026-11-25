// Per-frequency running statistics (fixed point, no heap, no history storage).
#pragma once
#include "rf_types.h"

enum : uint8_t { BF_ACTIVE = 0x01, BF_ZHIT = 0x02, BF_JUMP = 0x04, BF_RISE = 0x08 };

struct BinStat {
  uint8_t  cur, peak, minv, flags;   // levels (see rf_types.h)
  uint16_t avg_q8;                   // ungated EMA of level (Q8)
  uint16_t noise_q8;                 // EMA of level over non-active samples (Q8) = background noise estimate
  uint16_t occ_q16;                  // EMA of the active flag (duty cycle, 0..65535)
  uint16_t pad;
  uint32_t var_q4sq;                 // EMA of squared deviation, deviation in Q4 -> (Q4)^2
  uint32_t act_count;                // rising edges of "active"
};

class RfStats {
 public:
  void reset();
  // raw: RSSI register values (0xFF = missing). learning: fast, ungated noise adaptation.
  void ingest(const uint8_t* raw, bool learning);
  const BinStat& bin(uint16_t i) const { return bins_[i]; }
  const GlobalStats& global() const { return g_; }
  uint32_t sweeps() const { return sweeps_; }
  void exportNoiseLvl(uint8_t* out) const;
  static uint8_t noiseLvl(const BinStat& b) { uint16_t v = (uint16_t)((b.noise_q8 + 128) >> 8); return (uint8_t)(v > 255 ? 255 : v); }
 private:
  BinStat bins_[RF_N];
  GlobalStats g_;
  uint32_t sweeps_;
};
