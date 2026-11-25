#pragma once
#include <stdint.h>
#include <cassert>
#include <vector>
class MockWire {
public:
  uint8_t address = 0x77, reg[256] = {}, addr = 0, pos = 0;
  bool online = true;
  std::vector<uint8_t> tx;
  void begin(int, int, uint32_t) {}
  void setTimeOut(unsigned) {}
  void beginTransmission(uint8_t a) { addr = a; tx.clear(); }
  void write(uint8_t v) { tx.push_back(v); }
  int endTransmission(bool = true) {
    if (!online || addr != address) return 2;
    assert(tx.size() == 1 || tx.size() == 2);
    pos = tx[0];
    assert(pos == 0xD0 || pos == 0xE0 || pos == 0xF3 || pos == 0x88 || pos == 0xF4 || pos == 0xF7);
    if (tx.size() == 2) reg[pos] = tx[1];
    return 0;
  }
  int requestFrom(int a, int n) {
    if (!online || a != address) return 0;
    assert((pos == 0x88 && n == 24) || (pos == 0xF7 && n == 6) || ((pos == 0xD0 || pos == 0xF3) && n == 1));
    return n;
  }
  int read() { return reg[pos++]; }
};
extern MockWire Wire;
