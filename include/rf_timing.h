// Microsecond-resolution sleeping for the RF task (esp_timer wake-up, no long busy-waits).
// Single-caller only: use exclusively from the RF acquisition task.
#pragma once
#include <stdint.h>

void    rfTimingInit();                       // call once from the RF task
int64_t rfNowUs();
void    rfSleepUntilUs(int64_t deadlineUs);   // sleeps via scheduler / esp_timer, spins only for the last ~CFG_RF_WAKE_LATENCY_US
void    rfSleepUs(uint32_t us);
