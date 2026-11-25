// Shared data types. No Arduino dependencies (host-testable).
//
// LEVEL CONVENTION: the SX1278 RegRssiValue ("raw") gives dBm = -raw/2.
// Internally the statistics use  lvl = 255 - raw  (0..255, higher = stronger,
// 0.5 dB per step, dBm = (lvl - 255) / 2). Binary logs store RAW register values.
// raw == 0xFF is the "no data" sentinel (point not measured).
#pragma once
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "config.h"

#define RF_N (RF_POINTS)
static_assert(RF_POINTS >= 8 && RF_POINTS <= 255, "RF_POINTS must fit in uint8 and be >= 8");
static_assert(RF_START_HZ < RF_STOP_HZ, "invalid sweep range");

#define RF_RAW_INVALID 0xFF

inline uint32_t rfBinFreqHz(uint16_t i) { return RF_START_HZ + (uint32_t)i * RF_STEP_HZ; }
inline uint8_t  rfRawToLvl(uint8_t raw) { return (uint8_t)(255 - raw); }
inline float    rfLvlToDbm(uint8_t lvl) { return ((int)lvl - 255) * 0.5f; }
inline uint8_t  rfDbmToLvl(float dbm) { int v = (int)(dbm * 2.0f) + 255; return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); }

inline uint16_t rfCrc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {   // CRC-16/CCITT-FALSE
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// ---------------------------------------------------------------- events
enum RfEvType : uint8_t {
  EV_NONE = 0,
  EV_RSSI_JUMP = 1,
  EV_PERSISTENT_NB = 2,
  EV_BURSTS = 3,
  EV_NOISE_FLOOR_RISE = 4,
  EV_ACTIVITY_SHIFT = 5,
  EV_WIDEBAND_SUSTAINED = 6,
  EV_LORA_CAD = 10,
  EV_LORA_PACKET = 11
};
enum RfEvPhase : uint8_t { PH_START = 0, PH_END = 1, PH_INSTANT = 2 };

inline const char* rfEvName(uint8_t t) {
  switch (t) {
    case EV_RSSI_JUMP: return "RSSI_JUMP";
    case EV_PERSISTENT_NB: return "PERSISTENT_NARROWBAND";
    case EV_BURSTS: return "INTERMITTENT_BURSTS";
    case EV_NOISE_FLOOR_RISE: return "NOISE_FLOOR_RISE";
    case EV_ACTIVITY_SHIFT: return "ACTIVITY_SHIFT";
    case EV_WIDEBAND_SUSTAINED: return "SUSTAINED_WIDEBAND_ACTIVITY";
    case EV_LORA_CAD: return "LORA_CAD_HIT";
    case EV_LORA_PACKET: return "LORA_PACKET";
    default: return "UNKNOWN";
  }
}
inline const char* rfPhaseName(uint8_t p) { return p == PH_START ? "START" : (p == PH_END ? "END" : "INSTANT"); }

struct RfEvent {
  uint64_t t_ms;       // monotonic ms since boot
  uint32_t epoch_s;    // 0 when time is not synchronized
  uint32_t freq_hz;
  uint32_t dur_ms;     // valid for PH_END
  uint8_t  type;
  uint8_t  phase;
  uint8_t  peak_lvl;
  uint8_t  noise_lvl;
  uint8_t  thr_lvl;    // 0 = not applicable
  uint8_t  synced;
  char     detail[24];
};

// ---------------------------------------------------------------- statistics summary
struct GlobalStats {
  uint16_t med_q8;           // smoothed sweep median level (Q8)
  uint16_t peak_sweep_bin;
  uint8_t  peak_sweep_lvl;
  uint8_t  peak_session_lvl;
  uint8_t  occ_inst_pct;     // % of bins active in this sweep
  uint8_t  occ_avg_pct;      // mean long-term duty over all bins
  uint16_t n_active;
  uint16_t n_valid;
};

// ---------------------------------------------------------------- sweep flags (record + snapshot)
enum : uint16_t {
  SF_TIME_SYNCED    = 1 << 0,
  SF_QUIET_GATE     = 1 << 1,
  SF_WIFI_ON        = 1 << 2,
  SF_TFT_ACTIVE     = 1 << 3,   // TFT bus activity overlapped this sweep
  SF_SD_ACTIVE      = 1 << 4,   // SD activity overlapped this sweep
  SF_BL_ON          = 1 << 5,
  SF_LORA_WINDOW    = 1 << 6,   // a LoRa window ran since the previous sweep
  SF_PARTIAL        = 1 << 7,   // some points missing
  SF_RADIO_RECOVERED= 1 << 8,
  SF_DIAG           = 1 << 9
};

struct SweepSnapshot {
  uint32_t id;
  uint64_t t_ms;
  uint32_t epoch_s;
  uint16_t dur_ms;
  uint16_t flags;
  uint8_t  radio_state;
  uint8_t  raw[RF_N];
  uint8_t  noise_lvl[RF_N];
  GlobalStats g;
  uint32_t events_started;
  uint16_t events_open;
};

// ---------------------------------------------------------------- binary log format (little endian)
struct __attribute__((packed)) SweepRecord {
  uint16_t magic;       // 0xA55A
  uint32_t sweep_id;
  uint64_t t_ms;
  uint32_t epoch_s;
  uint16_t sweep_ms;
  uint16_t flags;
  uint8_t  cfg_id;
  uint8_t  n;
  uint8_t  raw[RF_N];
  uint16_t crc;         // CRC-16/CCITT-FALSE over all preceding bytes
};
static_assert(sizeof(SweepRecord) == 24 + RF_POINTS + 2, "SweepRecord layout");
#define SWEEP_RECORD_MAGIC 0xA55A

struct __attribute__((packed)) LogFileHeader {
  char     magic[8];        // "RF247BIN"
  uint16_t version;
  uint16_t header_size;     // 64
  uint32_t session_id;
  uint32_t session_uid;
  uint16_t part_index;
  uint16_t n_points;
  uint32_t start_freq_hz;
  uint32_t step_hz;
  uint32_t rx_bw_hz;
  uint16_t dwell_us;
  uint16_t settle_us;
  uint8_t  rssi_samples;
  uint8_t  rssi_smoothing_exp;
  uint8_t  quiet_mode;
  uint8_t  reserved0;
  uint64_t file_start_t_ms;
  uint32_t file_start_epoch_s;
  uint16_t record_size;
  uint8_t  reset_reason;
  uint8_t  reserved1[5];
};
static_assert(sizeof(LogFileHeader) == 64, "LogFileHeader must be 64 bytes");
