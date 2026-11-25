// RF-24/7 Observatory - centralized configuration.
// Every tunable lives here. Plain #defines only, so this header is also usable
// from the host-side unit tests (no Arduino includes).
#pragma once
#include <stdint.h>

// ============================================================ identity
#define CFG_FW_NAME                 "RF-24/7 Observatory"
#define CFG_FW_VERSION              "1.0.5"
#define CFG_LOG_FORMAT_VERSION      1

// ============================================================ feature switches
#define CFG_ENABLE_DISPLAY          1
#define CFG_ENABLE_SD               1
#define CFG_ENABLE_BMP280           1
#define CFG_ENABLE_LORA_MONITOR     1   // periodic LoRa CAD / packet windows
#define CFG_ENABLE_WIFI_DASHBOARD   0   // Wi-Fi is OFF by default (initial RF characterization)
#define CFG_ENABLE_NTP_AT_BOOT      0   // STA mode only; needs CFG_WIFI_SSID
#define CFG_DIAG_WIFI_STEP          1   // let the self-interference diagnostic switch Wi-Fi on temporarily
#define CFG_QUIET_ACQUISITION       1   // TFT/SD/I2C work only between sweeps
#define CFG_DIAG_AUTORUN            0   // 1 = run self-interference diagnostic after baseline learning
#define CFG_DEBUG                   0   // compile-time debug logging
#define CFG_SERIAL_CONSOLE          1   // minimal command console on UART0

#define CFG_WIFI_SUPPORT  (CFG_ENABLE_WIFI_DASHBOARD || CFG_ENABLE_NTP_AT_BOOT || CFG_DIAG_WIFI_STEP)

// ============================================================ pins (FIXED by hardware)
#define PIN_SX_SCK      12
#define PIN_SX_MISO     13
#define PIN_SX_MOSI     11
#define PIN_SX_NSS      10
#define PIN_SX_RST       9
#define PIN_SX_DIO0     14
#define PIN_SX_DIO1     15
#define PIN_SX_DIO2     16

#define PIN_TFT_MOSI    35
#define PIN_TFT_SCK     36
#define PIN_TFT_CS      39
#define PIN_TFT_RST     40
#define PIN_TFT_DC      41
#define PIN_TFT_BL      42   // optional

#define PIN_SD_MOSI     35   // shared HSPI bus
#define PIN_SD_SCK      36
#define PIN_SD_MISO     37
#define PIN_SD_CS       38

#define PIN_I2C_SDA      4
#define PIN_I2C_SCL      5
#define BMP280_ADDR_PRIMARY    0x76
#define BMP280_ADDR_SECONDARY  0x77

// ============================================================ RF sweep
#define RF_START_HZ     433050000UL
#define RF_STOP_HZ      434790000UL
#define RF_STEP_HZ      10000UL
#define RF_POINTS       ((uint16_t)(((RF_STOP_HZ - RF_START_HZ) / RF_STEP_HZ) + 1UL))   // 175

// FSK receiver bandwidth: RxBw = 32 MHz / (mant * 2^(exp+2)); mant = 16/20/24 for idx 0/1/2
#define CFG_RF_RXBW_MANT_IDX    1      // 20
#define CFG_RF_RXBW_EXP         5      // -> 12.5 kHz (supported steps: 10.4, 12.5, 15.6, 20.8 ... kHz)
#define CFG_RF_RXBW_HZ  (32000000UL / ((16UL + 4UL * CFG_RF_RXBW_MANT_IDX) * (1UL << (CFG_RF_RXBW_EXP + 2))))

#define CFG_RF_DWELL_US         5000   // total time per frequency point (initial value)
#define CFG_RF_SETTLE_US        2500   // PLL + AGC + RSSI settling before the first RSSI read (MUST be validated, see README)
#define CFG_RF_RSSI_SAMPLES     4      // software averaging: samples spread over (dwell - settle)
#define CFG_RF_RSSI_SMOOTHING_EXP 1    // hardware RSSI smoothing: 2^(n+1) samples (n = 0..7)
#define CFG_RF_PLL_TIMEOUT_US   1000   // extra wait for RxReady/PllLock after the settle time
#define CFG_RF_WAKE_LATENCY_US  40     // esp_timer wake-up latency compensation
#define CFG_RF_SWEEP_PERIOD_MS  1500   // start-to-start sweep interval (sweep itself takes ~0.9 s)
#define CFG_RF_INIT_RETRIES     5
#define CFG_RF_FAIL_RECOVER_POINTS 6   // consecutive failed points that abort a sweep and trigger recovery
#define CFG_RF_RECOVER_BACKOFF_MIN_MS 500
#define CFG_RF_RECOVER_BACKOFF_MAX_MS 30000
#define CFG_RF_RETUNE_VIA_STANDBY 0    // 1 = conservative retune (standby -> write Frf -> RX); slower, see README
#define CFG_RF_SPI_HZ           8000000UL

// ============================================================ LoRa monitor (receive-only)
#define CFG_LORA_WINDOW_EVERY_SWEEPS 20
#define CFG_LORA_FREQS_HZ       { 433175000UL, 433375000UL, 433575000UL }   // EU433 LoRaWAN default channels
#define CFG_LORA_SF_LIST        { 7, 9, 12 }
#define CFG_LORA_BW_IDX         7      // 7 = 125 kHz (0:7.8k 1:10.4k 2:15.6k 3:20.8k 4:31.25k 5:41.7k 6:62.5k 7:125k 8:250k 9:500k)
#define CFG_LORA_CR_IDX         1      // 1 = 4/5
#define CFG_LORA_SYNC_WORD      0x12   // 0x12 private, 0x34 public LoRaWAN
#define CFG_LORA_CAD_TIMEOUT_MS 150
#define CFG_LORA_RX_LISTEN_MS   400    // listen time after a CAD hit

// ============================================================ statistics (levels are 0.5 dB units, 255 = 0 dBm)
#define CFG_STAT_AVG_SHIFT        5
#define CFG_STAT_NOISE_SHIFT      5
#define CFG_STAT_LEARN_SHIFT      3
#define CFG_STAT_OCC_SHIFT        6
#define CFG_STAT_MED_SHIFT        2
#define CFG_STAT_OPEN_MARGIN      10   // 5 dB over noise to become "active"
#define CFG_STAT_KEEP_MARGIN       6   // 3 dB hysteresis to stay active
#define CFG_STAT_LEARN_CLIP       24   // 12 dB: clip applied to noise updates during learning
#define CFG_STAT_MIN_VAR        1024   // (2 levels * 16)^2 : minimum sigma 1 dB
#define CFG_STAT_INIT_VAR       2304   // (3 levels * 16)^2

// ============================================================ anomaly detector
#define CFG_BASELINE_SWEEPS       120  // baseline learning period (sweeps)
#define CFG_DET_Z_OPEN              4
#define CFG_DET_Z_OPEN_SQ          (CFG_DET_Z_OPEN * CFG_DET_Z_OPEN)
#define CFG_DET_JUMP_LVL           16  // 8 dB single-sweep increase
#define CFG_DET_PERSIST_SWEEPS     20
#define CFG_DET_BURST_MIN_TRANS     4  // transitions in the last 16 sweeps
#define CFG_DET_CLOSE_QUIET         4
#define CFG_DET_MERGE_BINS          2
#define CFG_DET_NF_RISE_OPEN       12  // 6 dB median rise vs baseline
#define CFG_DET_NF_RISE_CLOSE       6
#define CFG_DET_NF_PERSIST          6
#define CFG_DET_WIDE_BINS          15  // 150 kHz of simultaneously active bins
#define CFG_DET_WIDE_PERSIST        5
#define CFG_DET_ACT_SHIFT_PCT       8  // percentage points of occupancy change
#define CFG_DET_ACT_PERSIST         4

// ============================================================ TFT UI
#ifndef CFG_UI_SPI_HZ
#define CFG_UI_SPI_HZ           4000000UL  // validated-version drawing clock; retain until hardware timing is characterized
#endif
#define CFG_UI_ROTATION         2      // +90 degrees from rotation 1; portrait 240x320 (0 = opposite portrait)
#define CFG_UI_REFRESH_MS       200    // max 5 FPS service-loop cadence (non-quiet mode)
#define CFG_UI_MIN_GRAPH_MS     200
#define CFG_UI_TEXT_MS          1000
#define CFG_UI_SCALE_ADAPTIVE   1
#define CFG_UI_RSSI_MIN_DBM     (-120)
#define CFG_UI_RSSI_MAX_DBM     (-60)
#define CFG_UI_HOLD_DECAY_LVL   1      // peak-hold decay per sweep (0.5 dB units)
#define CFG_UI_WATERFALL_SCROLL 1      // 1 = latest row on top, region redrawn per sweep; 0 = in-place raster + marker
#define CFG_UI_WF_DEAD_LVL      4
#define CFG_UI_WF_STEP_LVL      4
#define CFG_TFT_BL_ENABLE       1      // drive PIN_TFT_BL high at boot (harmless if not wired)
#define CFG_TFT_BL_SWITCHABLE   0      // 1 only if the backlight is really switched by PIN_TFT_BL

// ============================================================ environment
#define CFG_ENV_SAMPLE_INTERVAL_S  10
#define CFG_ENV_REINIT_INTERVAL_S  30
#define CFG_I2C_HZ                 100000UL
#define CFG_I2C_TIMEOUT_MS         50

// ============================================================ logging
#define CFG_SD_SPI_HZ              10000000UL
#define CFG_LOG_DIR_ROOT           "/RF247"
#define CFG_LOG_FLUSH_INTERVAL_S   10
#define CFG_LOG_RING_RECORDS       48
#define CFG_LOG_SWEEP_EVERY_N      1
#define CFG_LOG_BIN_BUF_BYTES      4096
#define CFG_LOG_EVT_BUF_BYTES      1536
#define CFG_LOG_ENV_BUF_BYTES      512
#define CFG_LOG_HLTH_BUF_BYTES     1024
#define CFG_LOG_TS_BUF_BYTES       256
#define CFG_LOG_DIAG_BUF_BYTES     512
#define CFG_LOG_MAX_BIN_FILE_BYTES 33554432UL   // 32 MiB per binary part
#define CFG_LOG_MAX_CSV_FILE_BYTES 4194304UL    // 4 MiB per CSV part
#define CFG_LOG_MAX_SESSION_BYTES  0UL          // 0 = unlimited
#define CFG_LOG_MIN_FREE_BYTES     67108864ULL  // stop logging below 64 MiB free (never delete data)
#define CFG_LOG_SD_RETRY_S         30
#define CFG_LOG_HEALTH_INTERVAL_S  60
#define CFG_RECENT_EVENTS          16

// ============================================================ time
#define CFG_TIME_MIN_VALID_EPOCH   1735689600UL   // 2025-01-01: anything earlier is "not synchronized"
#define CFG_NTP_SERVER1            "pool.ntp.org"
#define CFG_NTP_SERVER2            "time.google.com"
#define CFG_NTP_TIMEOUT_S          40

// ============================================================ Wi-Fi / dashboard
#define CFG_WIFI_USE_STA           0
#define CFG_WIFI_SSID              ""
#define CFG_WIFI_PASS              ""
#define CFG_WIFI_AP_SSID           "RF247-OBS"
#define CFG_WIFI_AP_PASS           "rf247obs1"
#define CFG_WIFI_TX_POWER          WIFI_POWER_8_5dBm
#define CFG_WEB_PORT               80
#define CFG_WEB_MIN_UPDATE_MS      1000

// ============================================================ quiet acquisition / diagnostics
#define CFG_QUIET_MAX_WAIT_MS      400
#define CFG_SVC_IDLE_MS_QUIET      3000
#define CFG_DIAG_STEP_SWEEPS       40
#define CFG_DIAG_SKIP_SWEEPS       4
#define CFG_DIAG_BIN_DELTA_LVL     3      // 1.5 dB per-bin change counted as "bin up"

// ============================================================ tasks
#define CFG_TASK_RF_PRIO           18
#define CFG_TASK_RF_STACK          6144
#define CFG_TASK_RF_CORE           1
#define CFG_TASK_SVC_PRIO          3
#define CFG_TASK_SVC_STACK         9216
#define CFG_TASK_SVC_CORE          0
#define CFG_TASK_NET_PRIO          4
#define CFG_TASK_NET_STACK         6144
#define CFG_TASK_NET_CORE          0

// ============================================================ reliability
#define CFG_WDT_TIMEOUT_S          20
#define CFG_RF_STALL_LIMIT_S       15
#define CFG_HEAP_WARN_BYTES        40000UL
#define CFG_HEAP_CRIT_BYTES        20000UL
#define CFG_STABLE_UPTIME_S        300
#define CFG_SAFE_STAGE1_STREAK     3     // abnormal resets in a row: disable display + Wi-Fi
#define CFG_SAFE_STAGE2_STREAK     5     // ... and SD
#define CFG_BOOT_BACKOFF_STEP_MS   2000
#define CFG_BOOT_BACKOFF_MAX_MS    20000
