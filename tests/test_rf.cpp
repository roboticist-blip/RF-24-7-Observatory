#include <cassert>
#include <cstdio>
#include <vector>
#include "rf_statistics.h"
#include "anomaly_detector.h"
#include "sweep_ring.h"
class Sink : public EventSink { public: std::vector<RfEvent> events; bool push(const RfEvent& e) override {events.push_back(e); return true;} };
int main() {
  static_assert(RF_N == 175, "default grid");
  assert(rfBinFreqHz(RF_N-1)==RF_STOP_HZ);
  assert(rfCrc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29B1);
  RfStats stats; StatisticalDetector det; Sink sink; stats.reset(); det.reset();
  uint8_t raw[RF_N]; memset(raw, 200, sizeof(raw)); DetectCtx c{};
  auto step = [&]() { c.t_ms += 1500; stats.ingest(raw, det.learning()); det.process(stats,c,sink); };
  for (unsigned i=0;i<CFG_BASELINE_SWEEPS;++i) step();
  assert(!det.learning() && sink.events.empty()); assert(stats.global().n_valid==RF_N);
  assert(stats.global().med_q8==(55u<<8));
  raw[80]=150; step(); assert(stats.global().n_active==1); assert(!sink.events.empty());
  assert(sink.events.back().type==EV_RSSI_JUMP && sink.events.back().phase==PH_START);
  raw[80]=RF_RAW_INVALID; step(); assert(stats.global().n_valid==RF_N-1); assert(stats.global().n_active==0);
  raw[80]=200; for (unsigned i=0;i<CFG_DET_CLOSE_QUIET+1;++i) step();
  bool ended=false; for(auto &e:sink.events) if(e.type==EV_RSSI_JUMP && e.phase==PH_END) ended=true; assert(ended);
  SweepRing ring; SweepRecord rec{}; for(unsigned i=0;i<CFG_LOG_RING_RECORDS-1;i++){rec.sweep_id=i; assert(ring.push(rec));}
  assert(!ring.push(rec) && ring.dropped()==1); for(unsigned i=0;i<CFG_LOG_RING_RECORDS-1;i++){assert(ring.peek()->sweep_id==i);ring.pop();} assert(!ring.peek());
  printf("PASS: frequency grid, CRC, baseline, anomaly start/end, invalid-bin occupancy, ring capacity/order\n");
}
