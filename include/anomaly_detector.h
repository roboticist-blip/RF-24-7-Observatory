// Lightweight statistical RF event detector (no ML framework).
// Modular: implement IRfEventDetector to plug in an embedded ML model later.
#pragma once
#include "rf_types.h"
#include "rf_statistics.h"

class EventSink {
 public:
  virtual ~EventSink() {}
  virtual bool push(const RfEvent& e) = 0;
};

struct DetectCtx {
  uint64_t t_ms;
  uint32_t epoch_s;
  bool synced;
};

class IRfEventDetector {
 public:
  virtual ~IRfEventDetector() {}
  virtual void reset() = 0;
  virtual bool learning() const = 0;
  virtual void process(const RfStats& st, const DetectCtx& ctx, EventSink& sink) = 0;
  virtual uint16_t openEvents() const = 0;
};

class StatisticalDetector : public IRfEventDetector {
 public:
  void reset() override;
  bool learning() const override { return sweeps_ < CFG_BASELINE_SWEEPS; }
  void process(const RfStats& st, const DetectCtx& ctx, EventSink& sink) override;
  uint16_t openEvents() const override { return openCount_; }
  uint16_t baselineMedQ8() const { return baseMed_q8_; }

 private:
  struct BinDet {
    uint64_t t_first;      // start of the current activity run
    uint64_t t_last;       // last sweep the bin was active
    uint16_t hist;         // active flag history (bit0 = latest)
    uint8_t  run, quiet, open, type, peak;
    uint8_t  lastpk;       // level at the last active sweep
  };
  struct GlobEv {
    uint64_t t_open, t_last;
    uint8_t  open, cnt, quiet, peak;
  };

  void emit(EventSink& sink, const DetectCtx& c, uint8_t type, uint8_t phase, uint32_t freq,
            uint8_t peak, uint8_t noise, uint8_t thr, uint32_t dur);
  static uint8_t thrLvl(const BinStat& b);
  bool neighbourOpen(uint16_t i) const;
  void globalEvent(GlobEv& ev, bool condOpen, bool condKeep, uint8_t persist, uint8_t type,
                   uint8_t peak, uint8_t noise, uint8_t thr, uint32_t freq,
                   const DetectCtx& c, EventSink& sink);

  BinDet det_[RF_N];
  GlobEv gNoise_, gWide_, gAct_;
  uint32_t sweeps_;
  uint16_t openCount_;
  uint16_t baseMed_q8_;
  uint16_t occFast_q8_, occSlow_q8_;   // occupancy percent in Q8
};
