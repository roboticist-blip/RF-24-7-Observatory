// RF acquisition task: sequential FSK RSSI sweep, statistics, event detection,
// LoRa windows, recovery. Highest priority; never waits on SD, TFT or Wi-Fi
// (except the bounded quiet-window handshake).
#pragma once
#include "shared_state.h"

void rfAcquisitionTask(void* arg);
