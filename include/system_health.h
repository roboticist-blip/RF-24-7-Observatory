// Boot policy (reset-reason tracking, reboot-loop protection), watchdog setup,
// and runtime health metrics (heap, stacks, task heartbeats).
#pragma once
#include <Arduino.h>
#include "esp_system.h"

enum HealthState : uint8_t { HEALTH_OK = 0, HEALTH_WARN = 1, HEALTH_FAULT = 2 };

struct BootInfo {
  esp_reset_reason_t reason;
  const char* reasonName;
  uint8_t  streak;        // consecutive abnormal resets (panic / WDT / brownout)
  uint32_t bootCount;
  bool     brownout;
};

struct HealthSnapshot {
  uint32_t uptime_s, heap_free, heap_min, heap_largest;
  uint32_t stack_rf, stack_svc, stack_net;     // high-water marks (bytes free)
  uint8_t  tasks_alive;                        // bit0 RF, bit1 service, bit2 network
  HealthState state;
};

void  healthBootInit();                        // FIRST call in setup(): applies profile + back-off delay
const BootInfo& healthBoot();
void  healthWdtInit();
void  healthMarkStable();                      // clears the abnormal-reset streak
void  healthCollect(HealthSnapshot& h);
HealthState healthEvaluate(HealthSnapshot& h); // also detects RF stalls / heap exhaustion
HealthState healthLast();                      // last evaluated state (for the UI)
bool  healthRestartRequested();
void  healthRequestRestart();
