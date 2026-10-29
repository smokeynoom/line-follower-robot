#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <Arduino.h>
#include "navigation.h"

// Serial port + boot banner + the reason the ESP32 (re)started. Call once, first thing in setup().
void diagBegin();

// Why the chip started this time (esp_reset_reason()), as text and as a verdict.
const char* diagResetReasonName();
// true for a reset the chip did not ask for (panic, watchdogs, brownout): the run was cut short by the controller, NOT by line loss.
bool diagResetWasUnexpected();

// printf-style log line with a millis() timestamp. Never blocks: if the serial TX buffer has no room the line is dropped (and counted).
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
uint32_t diagDroppedLogLines();

// ---- Memory that survives an unexpected reset (ESP32 RTC memory; lost on a real power-up) -------------------------------------------------------
struct RetainedState {
  NavMemory nav;
  bool wasRunning = false;          // a run was active when the controller last wrote this
  uint8_t lastState = 0;            // robot state at the last state change
  uint32_t lastUptimeMs = 0;        // millis() at that moment
  uint16_t unexpectedResets = 0;    // unexpected resets since the last power-up
};

// Loads the retained memory. Returns true if it is valid AND this start followed an unexpected reset (i.e. it should be used).
// On a normal power-up it is cleared and false is returned.
bool diagLoadRetained(RetainedState& out);
void diagSaveRetained(const RetainedState& in);

#endif
