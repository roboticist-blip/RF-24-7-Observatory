#include "sx1278_driver.h"
#include "rf_timing.h"
#include "esp_timer.h"

using namespace sxreg;

static const uint32_t kLoraBwHz[10] = {7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};

uint8_t Sx1278::readReg(uint8_t a) {
  spi_->beginTransaction(set_);
  digitalWrite(PIN_SX_NSS, LOW);
  spi_->transfer(a & 0x7F);
  uint8_t v = spi_->transfer(0x00);
  digitalWrite(PIN_SX_NSS, HIGH);
  spi_->endTransaction();
  return v;
}

void Sx1278::writeReg(uint8_t a, uint8_t v) {
  if (a == OPMODE && ((v & 7) == 2 || (v & 7) == 3)) { txBlocked_++; return; }
  spi_->beginTransaction(set_);
  digitalWrite(PIN_SX_NSS, LOW);
  spi_->transfer(a | 0x80);
  spi_->transfer(v);
  digitalWrite(PIN_SX_NSS, HIGH);
  spi_->endTransaction();
}

void Sx1278::writeFrf(uint32_t frf) {              // burst write, address auto-increments MSB->MID->LSB
  uint8_t buf[4] = {(uint8_t)(FRF_MSB | 0x80), (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf};
  spi_->beginTransaction(set_);
  digitalWrite(PIN_SX_NSS, LOW);
  spi_->transferBytes(buf, nullptr, 4);
  digitalWrite(PIN_SX_NSS, HIGH);
  spi_->endTransaction();
}

bool Sx1278::setOpMode(uint8_t v) {                // RECEIVE-ONLY guard: refuse FSTX (010) and TX (011)
  const uint8_t m = v & 0x07;
  if (m == 0x02 || m == 0x03) { txBlocked_++; return false; }
  writeReg(OPMODE, v);
  return true;
}

bool Sx1278::begin(SPIClass& spi) {
  spi_ = &spi;
  set_ = SPISettings(CFG_RF_SPI_HZ, MSBFIRST, SPI_MODE0);
  pinMode(PIN_SX_NSS, OUTPUT);
  digitalWrite(PIN_SX_NSS, HIGH);
  pinMode(PIN_SX_RST, OUTPUT);
  digitalWrite(PIN_SX_RST, HIGH);
  pinMode(PIN_SX_DIO0, INPUT);
  pinMode(PIN_SX_DIO1, INPUT);
  pinMode(PIN_SX_DIO2, INPUT);
  return reset() && probe();
}

bool Sx1278::reset() {
  digitalWrite(PIN_SX_RST, LOW);
  delay(2);                                        // >= 100 us required
  digitalWrite(PIN_SX_RST, HIGH);
  delay(10);                                       // >= 5 ms before SPI access
  lora_ = false;
  return true;
}

bool Sx1278::probe() {
  const uint8_t v = readReg(VERSION);
  if (v != VERSION_EXPECTED) { commErr_++; return false; }
  return true;
}

bool Sx1278::configureFsk(uint32_t frf) {
  if (!probe()) return false;
  // LongRangeMode can only be changed in Sleep: go to sleep (keeping the current mode bit), then select FSK.
  setOpMode((uint8_t)((readReg(OPMODE) & LORA) | LF | MODE_SLEEP));
  delay(1);
  setOpMode((uint8_t)(LF | MODE_SLEEP));    // FSK, low-frequency port, sleep
  delay(1);
  setOpMode(LF | MODE_STDBY);
  delay(1);
  lora_ = false;

  writeReg(RX_CONFIG, RXCFG_BASE);
  writeReg(RSSI_CONFIG, (uint8_t)(CFG_RF_RSSI_SMOOTHING_EXP & 0x07));        // RssiOffset = 0
  const uint8_t bw = (uint8_t)(((CFG_RF_RXBW_MANT_IDX & 0x03) << 3) | (CFG_RF_RXBW_EXP & 0x07));
  writeReg(RX_BW, bw);
  writeFrf(frf);
  if (readReg(RX_BW) != bw || (readReg(RSSI_CONFIG) & 0x07) != (CFG_RF_RSSI_SMOOTHING_EXP & 0x07)) {
    commErr_++;                                    // register readback mismatch: SPI wiring/clock problem
    return false;
  }
  return enterFskRx();
}

bool Sx1278::enterFskRx() {
  if (!setOpMode(OPMODE_FSK_RX)) return false;
  return waitRxReady(5000);
}

bool Sx1278::waitRxReady(uint32_t timeoutUs) {
  const int64_t dl = rfNowUs() + timeoutUs;
  for (;;) {
    const uint8_t f = readReg(IRQ_FLAGS1);         // bit7 ModeReady, bit6 RxReady, bit4 PllLock
    if ((f & 0x50) == 0x50) return true;
    if (rfNowUs() >= dl) return false;
    rfSleepUs(100);
  }
}

bool Sx1278::retune(uint32_t frf) {
#if CFG_RF_RETUNE_VIA_STANDBY
  if (!setOpMode(LF | MODE_STDBY)) return false;
  writeFrf(frf);
  return setOpMode(OPMODE_FSK_RX);
#else
  writeFrf(frf);
  writeReg(RX_CONFIG, RXCFG_BASE);                 // 0 -> 1 transition on RestartRxWithPllLock
  writeReg(RX_CONFIG, (uint8_t)(RXCFG_BASE | RXCFG_RESTART_PLL));
  return true;
#endif
}

uint8_t Sx1278::readRssiRaw() { return readReg(RSSI_VALUE); }

bool Sx1278::verifyFskRx() {
  if (!probe()) return false;
  const uint8_t m = readReg(OPMODE);
  if ((m & 0x87) != MODE_RX_FSK) { commErr_++; return false; }
  return true;
}

void Sx1278::standby() { setOpMode((uint8_t)((lora_ ? LORA : 0) | LF | MODE_STDBY)); }

bool Sx1278::recover(uint32_t frf) {
  reset();
  if (!probe()) return false;
  return configureFsk(frf);
}

// ------------------------------------------------------------------ LoRa
bool Sx1278::enterLora() {
  setOpMode(LF | MODE_SLEEP);                      // leave FSK RX through sleep
  delay(1);
  setOpMode((uint8_t)(LORA | LF | MODE_SLEEP));
  delay(1);
  setOpMode((uint8_t)(LORA | LF | MODE_STDBY));
  delay(1);
  lora_ = true;
  return (readReg(OPMODE) & LORA) != 0;
}

bool Sx1278::loraConfigure(uint32_t hz, uint8_t bwIdx, uint8_t sf, uint8_t crIdx, uint8_t sync) {
  if (sf < 7) sf = 7;
  if (sf > 12) sf = 12;
  if (bwIdx > 9) bwIdx = 9;
  if (crIdx < 1 || crIdx > 4) crIdx = 1;
  setOpMode((uint8_t)(LORA | LF | MODE_STDBY));
  writeFrf(hzToFrf(hz));
  writeReg(LR_MODEM_CFG1, (uint8_t)((bwIdx << 4) | (crIdx << 1)));            // explicit header
  writeReg(LR_MODEM_CFG2, (uint8_t)((sf << 4) | 0x04));                       // payload CRC on
  const bool ldro = ((1UL << sf) * 1000UL / kLoraBwHz[bwIdx]) >= 16;          // symbol time >= 16 ms
  writeReg(LR_MODEM_CFG3, (uint8_t)((ldro ? 0x08 : 0x00) | 0x04));           // LDRO, AgcAutoOn
  writeReg(LR_DETECT_OPT, (uint8_t)((readReg(LR_DETECT_OPT) & 0xF8) | 0x03)); // SF7..12
  writeReg(LR_DETECT_THR, 0x0A);
  writeReg(LR_SYNC_WORD, sync);
  writeReg(LR_FIFO_RX_BASE, 0x00);
  writeReg(LR_FIFO_ADDR_PTR, 0x00);
  loraBwHz_ = kLoraBwHz[bwIdx];
  return (readReg(LR_MODEM_CFG2) >> 4) == sf;
}

bool Sx1278::loraCad(bool& detected, uint32_t timeoutMs) {
  detected = false;
  writeReg(LR_IRQ_FLAGS, 0xFF);
  if (!setOpMode((uint8_t)(LORA | LF | MODE_CAD))) return false;
  const int64_t dl = rfNowUs() + (int64_t)timeoutMs * 1000;
  for (;;) {
    const uint8_t f = readReg(LR_IRQ_FLAGS);       // bit2 CadDone, bit0 CadDetected
    if (f & 0x04) {
      detected = (f & 0x01) != 0;
      writeReg(LR_IRQ_FLAGS, 0x05);
      standby();
      return true;
    }
    if (rfNowUs() >= dl) { standby(); return false; }
    rfSleepUs(500);
  }
}

bool Sx1278::loraStartRx() {
  writeReg(LR_IRQ_FLAGS, 0xFF);
  writeReg(LR_FIFO_ADDR_PTR, 0x00);
  return setOpMode((uint8_t)(LORA | LF | 0x05));   // RXCONTINUOUS
}

bool Sx1278::loraPollPacket(LoraPacket& p) {
  const uint8_t f = readReg(LR_IRQ_FLAGS);
  if (!(f & 0x40)) return false;                   // RxDone
  p.crcError = (f & 0x20) != 0;
  p.len = readReg(LR_RX_NB_BYTES);
  const int8_t snr4 = (int8_t)readReg(LR_PKT_SNR);
  p.snr_db = snr4 * 0.25f;
  float rssi = -164.0f + readReg(LR_PKT_RSSI);     // LF port (< 525 MHz)
  if (p.snr_db < 0) rssi += p.snr_db;
  p.rssi_dbm = rssi;
  int32_t fei = ((int32_t)(readReg(LR_FEI_MSB) & 0x0F) << 16) | ((int32_t)readReg(LR_FEI_MSB + 1) << 8) | readReg(LR_FEI_MSB + 2);
  if (fei & 0x80000) fei -= 0x100000;              // 20-bit two's complement
  p.fei_hz = (float)fei * (16777216.0f / 32000000.0f) * ((float)loraBwHz_ / 500000.0f);
  p.feiValid = !p.crcError;
  writeReg(LR_IRQ_FLAGS, 0xFF);
  return true;
}

float Sx1278::loraRssiNow() { return -164.0f + readReg(LR_RSSI_VALUE); }
