// Boot diagnostics: why the chip last reset, a crash-loop counter that drops
// into safe mode (no Bluetooth) after repeated crashes, and the stored core
// dump summary from the most recent crash, printed to the log.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>

// Call first thing in app_main. Returns true if the firmware should start in
// safe mode (USB + log only) because it crashed several times in a row.
bool diag_boot(void);

// Starts a task that repeats the crash report in the log for a while (so it
// is seen even if the log port is opened late) and clears the crash-loop
// counter once the firmware has run for a minute.
void diag_start(void);

// Erases the stored crash report.
void diag_clear_crash(void);
