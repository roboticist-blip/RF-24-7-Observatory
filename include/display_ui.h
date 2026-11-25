// Instrument-style TFT UI (portrait or landscape). Service task only.
// Dirty-region rendering: static chrome once, text fields only when their text changes,
// spectrum/waterfall only when a new sweep arrived. No full-screen framebuffer.
#pragma once
#include <Arduino.h>
#include <SPI.h>

bool uiInit(SPIClass& bus);                 // returns false if disabled
void uiUpdate(uint64_t nowMs);              // cheap when nothing changed
void uiSetBacklight(bool on);
void uiForceRedraw();
