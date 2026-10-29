#include "diagnostics.h"

#include <esp_attr.h>
#include <esp_system.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "config.h"

namespace {

// RTC_NOINIT memory is NOT cleared by a reset (panic, watchdog, brownout, software) and is garbage after a power-up, so it carries a magic number
// and a checksum and is only trusted when both match.
//
// The block is deliberately a plain struct of uint32_t with NO default member initialisers and no padding: anything with a constructor would be
// re-initialised by the C++ start-up code on every boot, which would wipe exactly the data this block exists to keep.
const uint32_t RETAINED_MAGIC = 0x4C46524Cu;   // "LFRL"

struct RetainedBlock {
  uint32_t magic;
  uint32_t pendingTurn, lastValidMarker, lastLineErrorSide, lastTurnExecuted;
  uint32_t markerCount, intersectionsCompleted;
  uint32_t wasRunning, lastState, unexpectedResets, lastUptimeMs;
  uint32_t checksum;
};
RTC_NOINIT_ATTR RetainedBlock retained;

esp_reset_reason_t bootReason = ESP_RST_UNKNOWN;
uint32_t droppedLines = 0;

uint32_t checksumOf(const RetainedBlock& b) {
  const uint32_t* words = (const uint32_t*)&b;
  const size_t count = (sizeof(RetainedBlock) - sizeof(uint32_t)) / sizeof(uint32_t);   // every word except the checksum itself
  uint32_t h = 2166136261u;                                                              // FNV-1a
  for (size_t i = 0; i < count; i++) h = (h ^ words[i]) * 16777619u;
  return h;
}

bool retainedIsValid() {
  return retained.magic == RETAINED_MAGIC && retained.checksum == checksumOf(retained);
}

Direction directionFromWord(uint32_t w) { return w == DIR_LEFT ? DIR_LEFT : (w == DIR_RIGHT ? DIR_RIGHT : DIR_NONE); }

}  // namespace

const char* diagResetReasonName() {
  switch (bootReason) {
    case ESP_RST_POWERON:   return "POWERON (normal power-up)";
    case ESP_RST_EXT:       return "EXT (reset pin)";
    case ESP_RST_SW:        return "SW (esp_restart)";
    case ESP_RST_PANIC:     return "PANIC (crash / exception)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP (wake-up)";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply voltage dipped)";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB";
    case ESP_RST_JTAG:      return "JTAG";
    case ESP_RST_EFUSE:     return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
    default:                return "UNKNOWN";
  }
}

bool diagResetWasUnexpected() {
  switch (bootReason) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_BROWNOUT:
    case ESP_RST_PWR_GLITCH:
    case ESP_RST_CPU_LOCKUP:
      return true;
    default:
      return false;
  }
}

void diagBegin() {
  Serial.setTxBufferSize(2048);          // room for a burst of log lines, so logging never stalls the control loop
  Serial.begin(SERIAL_BAUD);
  bootReason = esp_reset_reason();
  droppedLines = 0;
  Serial.println();
  Serial.println("=== MECC 2026 line follower: state-machine firmware ===");
  Serial.print("Reset reason: ");
  Serial.print(diagResetReasonName());
  Serial.println(diagResetWasUnexpected() ? "   <-- UNEXPECTED: the controller reset, this was not line loss" : "");
}

void logf(const char* fmt, ...) {
  char line[176];
  int n = snprintf(line, sizeof(line), "[%lu] ", (unsigned long)millis());
  va_list ap;
  va_start(ap, fmt);
  const int m = vsnprintf(line + n, sizeof(line) - n, fmt, ap);
  va_end(ap);
  if (m > 0) n += m;
  if (n > (int)sizeof(line) - 2) n = (int)sizeof(line) - 2;
  line[n++] = '\n';
  if ((int)Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t*)line, n);
  } else {
    droppedLines++;
  }
}

uint32_t diagDroppedLogLines() { return droppedLines; }

bool diagLoadRetained(RetainedState& out) {
  const bool valid = retainedIsValid();
  const bool unexpected = diagResetWasUnexpected();

  if (valid && unexpected) {
    out = RetainedState();
    out.nav.pendingTurn = directionFromWord(retained.pendingTurn);
    out.nav.lastValidMarkerDirection = directionFromWord(retained.lastValidMarker);
    out.nav.lastLineErrorSide = directionFromWord(retained.lastLineErrorSide);
    out.nav.lastTurnExecuted = directionFromWord(retained.lastTurnExecuted);
    out.nav.markerCount = (uint16_t)retained.markerCount;
    out.nav.intersectionsCompleted = (uint16_t)retained.intersectionsCompleted;
    out.wasRunning = retained.wasRunning != 0;
    out.lastState = (uint8_t)retained.lastState;
    out.lastUptimeMs = retained.lastUptimeMs;
    out.unexpectedResets = (uint16_t)(retained.unexpectedResets < 0xFFFF ? retained.unexpectedResets + 1 : 0xFFFF);
    return true;
  }
  out = RetainedState();                   // normal power-up (or unusable memory): start clean
  return false;
}

void diagSaveRetained(const RetainedState& in) {
  retained.magic = RETAINED_MAGIC;
  retained.pendingTurn = in.nav.pendingTurn;
  retained.lastValidMarker = in.nav.lastValidMarkerDirection;
  retained.lastLineErrorSide = in.nav.lastLineErrorSide;
  retained.lastTurnExecuted = in.nav.lastTurnExecuted;
  retained.markerCount = in.nav.markerCount;
  retained.intersectionsCompleted = in.nav.intersectionsCompleted;
  retained.wasRunning = in.wasRunning ? 1 : 0;
  retained.lastState = in.lastState;
  retained.unexpectedResets = in.unexpectedResets;
  retained.lastUptimeMs = in.lastUptimeMs;
  retained.checksum = checksumOf(retained);
}
