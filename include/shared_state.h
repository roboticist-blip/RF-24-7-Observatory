// Cross-task shared state: metrics, runtime control flags, latest sweep snapshot,
// environment data, recent-event ring, queues. Single-writer fields are volatile;
// multi-word structures are copied inside short critical sections.
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "rf_types.h"
#include "sweep_ring.h"

#if CFG_DEBUG
#define DBG(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(...) ((void)0)
#endif

enum RfState : uint8_t { RF_INIT = 0, RF_OK = 1, RF_RECOVERING = 2, RF_FAULT = 3 };
enum SdState : uint8_t { SD_DISABLED = 0, SD_ABSENT = 1, SD_OK = 2, SD_ERROR = 3, SD_FULL = 4 };

struct Metrics {
  volatile uint32_t sweeps_done, points_dropped, pll_timeouts, rf_recoveries, rf_comm_errors, rf_tx_blocked;
  volatile uint32_t sweep_ms_last, sweep_ms_max, sweep_ms_avg_x10, proc_us_last, proc_us_max;
  volatile uint32_t quiet_overruns, evq_drops, events_started;
  volatile uint32_t lora_windows, lora_cad_runs, lora_cad_hits, lora_packets;
  volatile uint32_t rf_heartbeat, svc_heartbeat, net_heartbeat;
  volatile uint32_t svc_busy_us_last, svc_busy_us_max;
  volatile uint8_t  rf_state;
  volatile uint16_t events_open;
  volatile uint8_t  sd_state;
  volatile uint32_t sd_errors, sd_bytes_written, sd_free_mb, log_bytes_lost, log_integrity_errors;
  volatile uint32_t sweeps_logged, sweeps_not_logged;
  volatile uint32_t env_failures;
  volatile uint32_t session_id, session_uid;
};

struct Control {
  volatile bool quiet_gate, tft_enable, sd_stress, bl_enable, wifi_enable, req_settle, req_diag, req_diag_abort;
  volatile bool wifi_active, bl_state, tft_busy, sd_busy;
  volatile uint8_t diag_active;
  volatile uint32_t tft_ops, sd_ops;
  volatile uint32_t window_seq, window_done;  // RF window generation / service acknowledgement
  // boot profile (set once at startup)
  volatile bool allow_display, allow_sd, allow_wifi;
};

struct EnvData {
  bool     valid;
  uint8_t  status;       // 0 ok, 1 not detected, 2 read error, 3 invalid values
  int16_t  t_c100;       // 0.01 C
  uint32_t p_pa;         // Pa
  uint64_t t_ms;
  uint32_t epoch_s;
};

struct DiagRow {
  char     step[12];
  uint8_t  tft, sd, bl, wifi, bl_switchable;
  uint16_t n;
  uint16_t med_q4, mean_q4, active_x10;
  uint8_t  max_lvl;
  int16_t  dmax_q4;      // max per-bin increase vs QUIET baseline (level units x16)
  uint16_t dmax_bin;
  uint16_t bins_up;
  uint64_t t_ms;
  uint32_t epoch_s;
};

extern Metrics g_metrics;
extern Control g_ctl;
extern SweepRing g_sweepRing;
extern QueueHandle_t g_eventQueue, g_diagQueue;
extern TaskHandle_t g_rfTask, g_svcTask, g_netTask;
extern SPIClass g_hspi;           // TFT + SD bus; owned by the service task only

void     sharedInit();
void     sharedPublishSweep(const SweepSnapshot& s);
uint32_t sharedSweepSeq();
uint32_t sharedGetSweep(SweepSnapshot& out);
void     sharedSetEnv(const EnvData& e);
void     sharedGetEnv(EnvData& e);
void     sharedPushRecentEvent(const RfEvent& e);
uint8_t  sharedGetRecentEvents(RfEvent* out, uint8_t maxN);   // newest first
