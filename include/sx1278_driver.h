// Lightweight register-level SX1278 driver. RECEIVE-ONLY: every mode write passes
// through setOpMode(), which refuses FSTX/TX. PaConfig is never written.
// Register addresses/bit masks follow the Semtech SX1276/77/78/79 datasheet
// (Rev. 7) - verify against your datasheet revision during bring-up (README).
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "config.h"

namespace sxreg {
// common
constexpr uint8_t OPMODE = 0x01, FRF_MSB = 0x06, FRF_MID = 0x07, FRF_LSB = 0x08, VERSION = 0x42;
// FSK/OOK
constexpr uint8_t LNA = 0x0C, RX_CONFIG = 0x0D, RSSI_CONFIG = 0x0E, RSSI_VALUE = 0x11, RX_BW = 0x12,
                  IRQ_FLAGS1 = 0x3E;
// LoRa
constexpr uint8_t LR_FIFO_ADDR_PTR = 0x0D, LR_FIFO_RX_BASE = 0x0F, LR_IRQ_FLAGS = 0x12, LR_RX_NB_BYTES = 0x13,
                  LR_PKT_SNR = 0x19, LR_PKT_RSSI = 0x1A, LR_RSSI_VALUE = 0x1B, LR_MODEM_CFG1 = 0x1D,
                  LR_MODEM_CFG2 = 0x1E, LR_MODEM_CFG3 = 0x26, LR_FEI_MSB = 0x28, LR_DETECT_OPT = 0x31,
                  LR_DETECT_THR = 0x37, LR_SYNC_WORD = 0x39;
// OpMode values (LowFrequencyModeOn = 0x08 for the 433 MHz band)
constexpr uint8_t LF = 0x08, MODE_SLEEP = 0x00, MODE_STDBY = 0x01, MODE_RX_FSK = 0x05, MODE_CAD = 0x07, LORA = 0x80;
constexpr uint8_t OPMODE_FSK_RX = LF | MODE_RX_FSK;                 // 0x0D
constexpr uint8_t RXCFG_BASE = 0x08;                                // AgcAutoOn, AFC off, no trigger
constexpr uint8_t RXCFG_RESTART_PLL = 0x20;                         // RestartRxWithPllLock
constexpr uint8_t VERSION_EXPECTED = 0x12;
}  // namespace sxreg

class Sx1278 {
 public:
  struct LoraPacket { bool crcError; bool feiValid; uint8_t len; float rssi_dbm; float snr_db; float fei_hz; };

  bool begin(SPIClass& spi);                 // pins, hardware reset, RegVersion check
  bool reset();
  bool probe();                              // RegVersion == 0x12
  bool configureFsk(uint32_t frf);           // FSK receiver for narrowband RSSI sweeping, enters RX
  bool enterFskRx();
  bool retune(uint32_t frf);                 // fast hop while in RX
  bool waitRxReady(uint32_t timeoutUs);
  uint8_t readRssiRaw();                     // RegRssiValue: dBm = -raw/2
  bool verifyFskRx();                        // RegVersion + RegOpMode readback
  void standby();
  bool recover(uint32_t frf);

  // LoRa mode (CAD + packet metadata only; payload is never read)
  bool enterLora();
  bool loraConfigure(uint32_t hz, uint8_t bwIdx, uint8_t sf, uint8_t crIdx, uint8_t sync);
  bool loraCad(bool& detected, uint32_t timeoutMs);
  bool loraStartRx();
  bool loraPollPacket(LoraPacket& p);
  float loraRssiNow();

  static uint32_t hzToFrf(uint32_t hz) { return (uint32_t)((((uint64_t)hz << 19) + 16000000ULL) / 32000000ULL); }
  uint32_t commErrors() const { return commErr_; }
  uint32_t txBlocked() const { return txBlocked_; }
  uint8_t readReg(uint8_t a);
  void writeReg(uint8_t a, uint8_t v);

 private:
  bool setOpMode(uint8_t v);
  void writeFrf(uint32_t frf);
  SPIClass* spi_ = nullptr;
  SPISettings set_;
  bool lora_ = false;
  uint32_t loraBwHz_ = 125000;
  uint32_t commErr_ = 0, txBlocked_ = 0;
};
