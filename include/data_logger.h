// Buffered, rotating SD logger. Owned by the service task (single owner of the HSPI bus).
// Streams: binary sweeps, CSV events / environment / health / time-sync / diagnostics.
#pragma once
#include "shared_state.h"
#include "environment.h"
#include "system_health.h"

class DataLogger {
 public:
  void begin(SPIClass& bus, bool enabled);
  void service(uint64_t nowMs, bool forceFlush);      // drain queues, batch-write, rotate, recover
  void logEnv(const EnvData& e);
  void logHealth(const HealthSnapshot& h);
  void stress(bool active);                           // diagnostic SD write load
  void flushAll();
  SdState state() const;
  const char* dir() const;
};
