# RF-24/7 Observatory

ESP32-S3 DevKitM-1 / SX1278 receive-only RF activity monitor, with ST7789V TFT, microSD and BMP280. **Version 1.0.5** is a cleaned release of the v1.0.4 ST7789 firmware. The user reported that the previous version worked; the exact test/build environment was not specified. This revision has not yet been independently compiled for ESP32 or tested on hardware. It is not a calibrated RF instrument.

For the full architecture, algorithms, measurements and experimental procedures, see [PROJECT_DOCUMENTATION.md](PROJECT_DOCUMENTATION.md).

## Build and upload

Open this directory (the one containing `platformio.ini`) in VS Code with PlatformIO. Or, with PlatformIO Core installed:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

The target is Espressif32 6.12.0 (Arduino-ESP32 2.0.17). The library dependencies are pinned in `platformio.ini`. No ESP-IDF application or dummy `setup()`/`loop()` is needed. If using the existing project, replace the complete `src/`, `include/`, and `platformio.ini` with these versions; do not keep duplicate old sources. If upload fails, identify the UART port with `pio device list`, set `upload_port`, and lower `upload_speed` if necessary. For a USB-UART board, use its UART USB connection for the monitor.

## Wiring (preserved from the original config.h)

| Module | Signals / ESP32-S3 GPIO |
|---|---|
| SX1278 (dedicated FSPI) | SCK 12, MISO 13, MOSI 11, NSS 10, RESET 9, DIO0 14, DIO1 15, DIO2 16 |
| TFT (shared HSPI) | SCK 36, MOSI 35, CS 39, RST 40, DC 41, BL 42 (optional control) |
| microSD (shared HSPI) | SCK 36, MOSI 35, MISO 37, CS 38 |
| BMP280 | SDA 4, SCL 5, address 0x76 or 0x77 |

All modules must share ground. Use 3.3 V logic. Check the power and logic requirements of each specific breakout, particularly SD and TFT modules, and provide adequate decoupling. GPIO42 must drive a suitable backlight control input/transistor, not the LED backlight load itself; set `CFG_TFT_BL_ENABLE=0` if not wired to a safe control input. Verify GPIO35–37 are exposed and not reserved for octal flash/PSRAM on the actual board/module; remap in `config.h` if necessary. GPIO39–42 may conflict with external JTAG use. Connect the correct 433 MHz antenna. Do not use 5 V signals on the radio. Use FAT32 microSD; SD is never auto-formatted.

## Operation

- RF task on core 1: 175 sequential RSSI samples, 433.050–434.790 MHz, 10 kHz grid, nominal 12.5 kHz receiver bandwidth. Four readings per point, nominal 5 ms point dwell, 1.5 s sweep period. Actual cadence depends on timing, PLL readiness, service windows and LoRa monitor windows.
- Service task on core 0: shared TFT/SD bus, BMP280 I2C, logger, console, time synchronization and health. Arduino loop sleeps. Quiet mode uses generation-tagged notifications; service waits for RF windows, but the RF task resumes after a bounded 400 ms acknowledgement timeout. Check `quiet_overruns` and recorded flags before treating results as quiet.
- Startup learns the baseline for 120 sweeps (roughly three minutes at the nominal period). Anomaly detection then reports jumps, narrowband persistence, bursts, median rises, occupancy shifts and wide activity. These are statistical indicators, not classified emitter identities.
- LoRa CAD windows are receive-only, periodic and sequential across the configured frequencies/SFs; no payload logging. A CAD hit is not a decoded packet, and the short receive window can miss long packets. The SX1278 cannot receive the whole band simultaneously; intermittent transmissions may be missed between visits.
- Wi-Fi is off by default except when explicitly exercising the Wi-Fi diagnostic. `CFG_ENABLE_WIFI_DASHBOARD=1` enables the optional basic status page at the AP address (typically 192.168.4.1) or the STA IP. Change the default AP password before enabling. The dashboard is local, unauthenticated beyond network access, and can disrupt quiet measurements; it shares the service task rather than using a permanent network task. For NTP set `CFG_ENABLE_NTP_AT_BOOT=1`, `CFG_WIFI_SSID` and `CFG_WIFI_PASS`; network activity is not RF-quiet. No RTC means UTC is not available until synchronized. Serial time synchronization works without Wi-Fi.

## Serial commands (115200 baud, newline)

| Command | Action |
|---|---|
| `help` | List commands |
| `status` | RF/SD state, RSSI, valid points, occupancy, environment, heap, stacks, logging |
| `c` | Receiver step-response characterization; use only with a stable RF test source |
| `diag` | Self-interference test after baseline; exercises quiet/TFT/SD/Wi-Fi/all configurations |
| `abort` | Abort diagnostic |
| `flush` | Request draining/flushing SD buffers |
| `tftredraw` | Restore the full normal UI from its latest snapshot; no new waterfall sample |
| `time <epoch>` | Set current UTC epoch seconds (2025 to 2038 supported) |
| `reboot` | Controlled restart, attempts to flush logs first |

Generate the current UTC epoch on Linux with `date +%s`, then send `time` followed by the returned value. Diagnostic rows with insufficient matched sweeps are not valid comparative results. Missing peripherals can make a diagnostic step time out. The backlight isolation step requires a real switchable backlight.

## Log data

Files live under `/RF247/Sxxxxxx/` on the SD card. `sweeps_XXXX.bin` holds a 64-byte v1 header and fixed-size 201-byte sweep records at the default grid, with CRC-16/CCITT-FALSE. CSV streams contain events, environment, health, diagnostics and time synchronization. `session.txt` captures boot/config metadata. Logging stops on low free space; the firmware does not delete past observations. Logs may be lost during SD failures, removal or sudden power loss; consult error counters and CRCs.

Decode a sweep file on a computer:

```sh
python3 tools/decode_sweeps.py sweeps_0001.bin sweeps_0001.csv
```

The CSV has one row per frequency point. Invalid RSSI readings are blank, not zero. The decoder reports corrupt/truncated records and exits nonzero if any are found. Raw RSSI is register-derived and uncalibrated; no absolute measurement accuracy is claimed. Occupancy denotes fraction of sampled bins considered active, not regulatory channel utilization or measured transmitter duty cycle.

## Validation and bring-up

Host tests (no Arduino SDK needed):

```sh
g++ -std=c++11 -Wall -Wextra -Werror -Iinclude tests/test_rf.cpp src/rf_statistics.cpp src/anomaly_detector.cpp -o /tmp/rf247_test
/tmp/rf247_test
python3 tests/test_decoder.py
g++ -std=c++11 -Wall -Wextra -Werror -Itests/mocks -Iinclude tests/test_bmp280.cpp src/environment.cpp -o /tmp/rf247_bmp
/tmp/rf247_bmp
```

All three host tests passed for this release in the packaging environment, including BMP280 compensation and mocked I2C error handling. The ESP32 cross-build could not be run: PlatformIO and the Xtensa SDK are unavailable. First run `pio run` locally; the hardware drivers, task scheduling, library/API compatibility and memory high-water marks remain unverified on the target.

Bring-up checklist:
1. Verify the 3.3 V rail and all pin mappings. Start with SX1278 and UART; temporarily set display/SD/BMP feature switches to 0 if necessary.
2. Confirm the radio version check and that sweeps increment (`status`); confirm `valid=175/175` under a healthy link. If flat or PLL failures persist, inspect wiring and try `CFG_RF_RETUNE_VIA_STANDBY=1`.
3. Add BMP280, TFT, SD individually. Confirm realistic temperature/pressure, correctly oriented display, session directory, and nonzero logging.
4. Use `c` with a stable test signal to characterize settling. Adjust `CFG_RF_SETTLE_US`, dwell and RBW from measurements, not assumptions. Use a known frequency and reference RF level to check tuning and RSSI offsets.
5. Run `diag` after learning; inspect actual matched counts and quiet-overrun metrics. A 400 ms service deadline is not a hard electrical-isolation guarantee; slow SD operations can overlap RF.
6. Run 24–72 h on a stable supply before rooftop use. Inspect WDT resets, minimum heap, stack headroom, dropped records, CRCs, timestamp continuity and environmental enclosure/condensation. Do not deploy until validated.

## Display and release configuration

The sole display driver is `Adafruit_ST7789`, initialized with `init(240, 320, SPI_MODE0)` on the shared hardware SPI bus after SD initialization. The drawing clock remains 4 MHz and the default rotation remains 2 (240 × 320). Select 0 for opposite portrait, or 1/3 for landscape; both layouts are included. Use `tftredraw` if a manual full repaint is needed. If touch is not used, hold T_CS high at 3.3 V; the touch controller is not driven by this firmware.

## Release notes

- v1.0.5: Removed the ILI9341 dependency, alternate build environments, standalone TFT test source, and TFT test-pattern console command. Preserved the ST7789 hardware SPI initialization, display layout and manual redraw recovery. The RF diagnostics, BMP280, SD formats, Wi-Fi feature switches, pin assignments, task architecture and data analysis tools remain unchanged.
- v1.0.4: Switched default driver to ST7789 and supplied A/B diagnostic builds; user reported success with this version.
- v1.0.1–1.0.3: BMP280 substitution, portrait support and display address-window safeguards.

When migrating, extract into a new folder rather than copying over the old source tree. In particular, the obsolete `src/tft_only.cpp` must not remain in `src/`, because it defines its own Arduino entry points.
