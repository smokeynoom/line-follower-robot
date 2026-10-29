#include "sensors.h"
#include "config.h"

// ---- Workshop API (unchanged) ----------------------------------------------------------------------------------------------------------------

void sensorsDigitalInit(const uint8_t IR_PINS[]) {
  for (int i = 0; i < 5; i++) {
    pinMode(IR_PINS[i], INPUT_PULLUP);
  }
}

uint16_t sensorsDigitalRead(const uint8_t IR_PINS[], uint8_t index) {
  if (index >= 5) {
  return -1;
  }
  uint16_t status = digitalRead(IR_PINS[index]);
  return status;
}


void sensorsAnalogInit(const uint8_t IR_PINS[]) {
  for (int i = 0; i < 5; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
}

uint16_t sensorsAnalogRead(const uint8_t IR_PINS[], uint8_t index) {
  if (index >= 5) {
  return -1;
  }
  uint16_t status = analogRead(IR_PINS[index]);
  return status;
}

// ---- Logical sensors ---------------------------------------------------------------------------------------------------------------------------

void sensorsInit() {
  for (int i = 0; i < 5; i++) {
    pinMode(IR_GPIO[i], SENSOR_USE_PULLUP ? INPUT_PULLUP : INPUT);
  }
}

uint8_t sensorsReadBits() {
  uint8_t bits = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(IR_GPIO[i]) == SENSOR_BLACK_LEVEL) bits |= (uint8_t)(1 << i);   // bit i = IR(i+1), independent of the GPIO number
  }
  return bits;
}

void sensorBitsToString(uint8_t bits, char* buf) {
  for (int i = 0; i < 5; i++) buf[i] = (bits & (1 << i)) ? '1' : '0';
  buf[5] = '\0';
}

// ---- Marker detector -----------------------------------------------------------------------------------------------------------------------------

void MarkerDetector::reset() {
  left_ = Side();
  right_ = Side();
}

void MarkerDetector::updateSide(Side& s, bool black, bool suppressed, uint32_t now) {
  s.black.update(black, now);
  s.white.update(!black, now);

  if (suppressed) {          // the intersection, a turn or a search is in progress: forget the box, require a fresh release afterwards
    s.armed = false;
    s.inBox = false;
    s.done = false;
    return;
  }
  if (s.inBox) {
    if (s.black.heldMs(now) > MARKER_MAX_MS) {           // far too long for a box: it is a line, not a marker
      s.inBox = false;
    } else if (s.white.heldFor(MARKER_RELEASE_MS, now)) {  // the box has passed: this is the moment it counts
      s.inBox = false;
      s.done = true;
      s.doneAt = now;
      s.armed = true;                                    // already released, ready for the next box
    }
    return;
  }
  if (!s.armed) {            // wait for the sensor to read white for long enough (start-up, after suppression, after a line)
    if (s.white.heldFor(MARKER_RELEASE_MS, now)) s.armed = true;
    return;
  }
  if (s.black.heldFor(MARKER_CONFIRM_MS, now)) {         // black for long enough: a box is passing
    s.inBox = true;
    s.armed = false;
  }
}

MarkerEvent MarkerDetector::update(uint8_t bits, bool suppressed, uint32_t now) {
  updateSide(left_, (bits & SENSOR_IR1) != 0, suppressed, now);
  updateSide(right_, (bits & SENSOR_IR5) != 0, suppressed, now);

  if (suppressed) return MARKER_NONE;
  if (left_.done && right_.done) {                       // boxes on both sides at once
    left_.done = false;
    right_.done = false;
    return MARKER_BOTH;
  }
  if (left_.done && now - left_.doneAt >= MARKER_PAIR_WINDOW_MS) {
    left_.done = false;
    return MARKER_LEFT;
  }
  if (right_.done && now - right_.doneAt >= MARKER_PAIR_WINDOW_MS) {
    right_.done = false;
    return MARKER_RIGHT;
  }
  return MARKER_NONE;
}
