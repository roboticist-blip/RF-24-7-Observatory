#include "environment.h"
#include <Wire.h>
#include "time_manager.h"

namespace {
bool s_ok = false;
bool s_wireStarted = false;
uint8_t s_addr = 0;
uint16_t dig_T1, dig_P1; int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

bool rd(uint8_t reg, uint8_t* b, uint8_t n) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)s_addr, (int)n) != (int)n) return false;
  for (uint8_t i = 0; i < n; i++) b[i] = (uint8_t)Wire.read();
  return true;
}
bool wr(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

bool probeAddr(uint8_t a) {
  s_addr = a;
  uint8_t id = 0;
  if (!rd(0xD0, &id, 1)) return false;
  if (id != 0x58) return false;      // BMP280 production chip ID
  return true;
}

int32_t compT(int32_t adc) {
  int32_t v1 = ((((adc >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
  int32_t v2 = (((((adc >> 4) - (int32_t)dig_T1) * ((adc >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
  t_fine = v1 + v2;
  return (t_fine * 5 + 128) >> 8;                  // 0.01 C
}

uint32_t compP(int32_t adc) {
  int32_t v1 = (t_fine >> 1) - 64000;
  int32_t v2 = (((v1 >> 2) * (v1 >> 2)) >> 11) * (int32_t)dig_P6;
  v2 = v2 + ((v1 * (int32_t)dig_P5) * 2);
  v2 = (v2 >> 2) + ((int32_t)dig_P4 * 65536);
  v1 = (((dig_P3 * (((v1 >> 2) * (v1 >> 2)) >> 13)) >> 3) + (((int32_t)dig_P2 * v1) >> 1)) >> 18;
  v1 = ((32768 + v1) * (int32_t)dig_P1) >> 15;
  if (v1 == 0) return 0;
  uint32_t p = (((uint32_t)(((int32_t)1048576) - adc) - (uint32_t)(v2 >> 12))) * 3125U;
  if (p < 0x80000000UL) p = (p << 1) / (uint32_t)v1; else p = (p / (uint32_t)v1) * 2;
  v1 = ((int32_t)dig_P9 * (int32_t)(((p >> 3) * (p >> 3)) >> 13)) >> 12;
  v2 = ((int32_t)(p >> 2) * (int32_t)dig_P8) >> 13;
  return (uint32_t)((int32_t)p + ((v1 + v2 + dig_P7) >> 4));       // Pa
}

}  // namespace

bool envPresent() { return s_ok; }

bool envInit() {
  s_ok = false;
  if (!s_wireStarted) {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, (uint32_t)CFG_I2C_HZ);
    Wire.setTimeOut(CFG_I2C_TIMEOUT_MS);
    s_wireStarted = true;
  }
  if (!probeAddr(BMP280_ADDR_PRIMARY) && !probeAddr(BMP280_ADDR_SECONDARY)) return false;
  if (!wr(0xE0, 0xB6)) return false;                                   // soft reset
  delay(5);
  uint8_t st = 1;
  for (uint8_t i = 0; i < 10 && (st & 0x01); i++) { if (!rd(0xF3, &st, 1)) return false; if (st & 0x01) delay(2); }
  if (st & 0x01) return false;
  uint8_t c1[24];
  if (!rd(0x88, c1, sizeof(c1))) return false;
  dig_T1 = (uint16_t)(c1[0] | (c1[1] << 8)); dig_T2 = (int16_t)(c1[2] | (c1[3] << 8)); dig_T3 = (int16_t)(c1[4] | (c1[5] << 8));
  dig_P1 = (uint16_t)(c1[6] | (c1[7] << 8)); dig_P2 = (int16_t)(c1[8] | (c1[9] << 8)); dig_P3 = (int16_t)(c1[10] | (c1[11] << 8));
  dig_P4 = (int16_t)(c1[12] | (c1[13] << 8)); dig_P5 = (int16_t)(c1[14] | (c1[15] << 8)); dig_P6 = (int16_t)(c1[16] | (c1[17] << 8));
  dig_P7 = (int16_t)(c1[18] | (c1[19] << 8)); dig_P8 = (int16_t)(c1[20] | (c1[21] << 8)); dig_P9 = (int16_t)(c1[22] | (c1[23] << 8));
  if (!dig_T1 || !dig_P1 || dig_T1 == 0xFFFF || dig_P1 == 0xFFFF) return false;             // calibration all-zero: bus problem
  s_ok = true;
  return true;
}

bool envSample(EnvData& o) {
  memset(&o, 0, sizeof(o));
  o.t_ms = timeMonoMs();
  o.epoch_s = timeEpoch();
  if (!s_ok) { o.status = 1; return false; }
  if (!wr(0xF4, 0x25)) { o.status = 2; s_ok = false; return false; }               // osrs_t x1, osrs_p x1, forced
  delay(10); // x1 temperature + x1 pressure: allow conversion to start and finish
  uint8_t st = 0x08;
  for (uint8_t i = 0; i < 20 && (st & 0x08); i++) { vTaskDelay(pdMS_TO_TICKS(2)); if (!rd(0xF3, &st, 1)) { o.status = 2; s_ok = false; return false; } }
  if (st & 0x08) { o.status = 2; s_ok = false; return false; }
  uint8_t d[6];
  if (!rd(0xF7, d, sizeof(d))) { o.status = 2; s_ok = false; return false; }
  const int32_t aP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  const int32_t aT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
  if (aT == 0x80000 || aP == 0x80000) { o.status = 3; return false; }
  const int32_t t = compT(aT);
  const uint32_t p = compP(aP);
  const bool tOk = (t >= -4000 && t <= 8500);
  const bool pOk = (p >= 30000 && p <= 110000);
  if (!tOk || !pOk) { o.status = 3; return false; }
  o.t_c100 = (int16_t)t;
  o.p_pa = p;
  o.valid = true;
  o.status = 0;
  return true;
}
