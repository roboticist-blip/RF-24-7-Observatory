// Self-interference characterization: steps through subsystem on/off combinations and
// compares RF statistics against a QUIET baseline. Runs inside the RF task (per-sweep
// hook); it only manipulates the runtime control flags in g_ctl.
//
// A sweep counts toward a step ONLY if the activity flags recorded for that sweep
// (what really happened on the buses) match the step definition.
#pragma once
#include "shared_state.h"

void diagStart();
void diagAbort();
bool diagActive();
void diagOnSweep(const SweepSnapshot& s);       // RF task, after each published sweep
void diagRestoreDefaults();
