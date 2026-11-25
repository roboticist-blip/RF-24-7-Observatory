// Minimal BMP280 driver (forced mode, Bosch integer compensation). Owned by the service task.
#pragma once
#include "shared_state.h"

bool envInit();                    // probes 0x76 then 0x77, reads calibration
bool envSample(EnvData& out);      // one forced measurement; false on failure (out.status says why)
bool envPresent();
