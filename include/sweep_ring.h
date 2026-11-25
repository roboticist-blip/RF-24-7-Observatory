// Single-producer (RF task) / single-consumer (service task) ring of binary sweep
// records. Lock-free, statically allocated; producer never blocks (drops + counts).
#pragma once
#include "rf_types.h"

class SweepRing {
 public:
  bool push(const SweepRecord& r) {
    const uint16_t h = head_, n = (uint16_t)((h + 1) % CFG_LOG_RING_RECORDS);
    if (n == tail_) { dropped_++; return false; }
    buf_[h] = r;
    __sync_synchronize();
    head_ = n;
    return true;
  }
  const SweepRecord* peek() const {
    if (tail_ == head_) return nullptr;
    __sync_synchronize();
    return &buf_[tail_];
  }
  void pop() {
    if (tail_ != head_) { __sync_synchronize(); tail_ = (uint16_t)((tail_ + 1) % CFG_LOG_RING_RECORDS); }
  }
  uint32_t dropped() const { return dropped_; }
  uint16_t depth() const {
    int d = (int)head_ - (int)tail_;
    if (d < 0) d += CFG_LOG_RING_RECORDS;
    return (uint16_t)d;
  }
 private:
  SweepRecord buf_[CFG_LOG_RING_RECORDS];
  volatile uint16_t head_ = 0, tail_ = 0;
  volatile uint32_t dropped_ = 0;
};
