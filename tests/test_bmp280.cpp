#include <cassert>
#include <cstdio>
#include <cstdlib>
#include "environment.h"
#include "Wire.h"
MockWire Wire;
void delay(unsigned long) {}
uint64_t timeMonoMs() { return 1234; }
uint32_t timeEpoch() { return 0; }
static void raw(uint8_t reg, uint32_t value) {
  Wire.reg[reg] = value >> 12;
  Wire.reg[reg+1] = value >> 4;
  Wire.reg[reg+2] = value << 4;
}
int main() {
  // Bosch example calibration and ADC values; expected 25.08 C, approx. 100653 Pa.
  const int c[] = {27504,26435,-1000,36477,-10685,3024,2855,140,-7,15500,-14600,6000};
  for (unsigned i=0;i<12;i++) { uint16_t v=static_cast<uint16_t>(c[i]); Wire.reg[0x88+2*i]=v; Wire.reg[0x89+2*i]=v>>8; }
  Wire.reg[0xD0]=0x60; assert(!envInit());
  Wire.reg[0xD0]=0x58; assert(envInit()); // secondary address fallback
  raw(0xF7,415148); raw(0xFA,519888);
  EnvData e{}; assert(envSample(e)); assert(e.valid && e.status==0 && e.t_c100==2508 && std::abs((int)e.p_pa-100653)<=4);
  assert(Wire.reg[0xF4]==0x25);
  raw(0xFA,0x80000); assert(!envSample(e) && !e.valid && e.status==3);
  Wire.online=false; assert(!envSample(e) && e.status==2 && !envPresent());
  assert(!envSample(e) && e.status==1);
  Wire.online=true; Wire.address=0x76; assert(envInit());
  raw(0xFA,519888); Wire.reg[0xF3]=8; assert(!envSample(e) && e.status==2);
  printf("PASS: BMP280 chip ID, both addresses, calibration, compensation, invalid ADC, bus error, conversion timeout\n");
}
