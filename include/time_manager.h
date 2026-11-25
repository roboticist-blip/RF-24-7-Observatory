// Time management without an RTC. Monotonic time is always available; wall-clock
// time exists only after an explicit synchronization (NTP, serial command or web
// browser). Unsynchronized time is NEVER presented as a valid date.
#pragma once
#include <Arduino.h>

enum TimeSource : uint8_t { TS_NONE = 0, TS_NTP = 1, TS_SERIAL = 2, TS_WEB = 3 };

struct TimeSyncEvent { uint64_t t_ms; uint32_t epoch_s; uint8_t source; };

void     timeInit();
uint64_t timeMonoMs();                          // ms since boot (64-bit, no wrap in practice)
bool     timeSynced();
uint32_t timeEpoch();                           // 0 when not synchronized
void     timeSetEpoch(uint32_t epoch, TimeSource src);
void     timeStartNtp();                        // requires an active STA connection
void     timePoll();                            // detects NTP completion; call from the service task
bool     timePeekSyncEvent(TimeSyncEvent& e);
void     timePopSyncEvent();
const char* timeSourceName(uint8_t s);
void     timeFormatUtc(char* out, size_t n, uint32_t epoch);       // "YYYY-MM-DDTHH:MM:SSZ"
void     timeFormatUptime(char* out, size_t n, uint64_t monoMs);   // "12d03h", "03h12m", "12m05s"
