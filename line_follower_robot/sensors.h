#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "common.h"

// ================================================================================================================================================
// Workshop API (Task 1 / Task 2 skeleton) — unchanged. The line follower itself uses the logical API further down.
// ================================================================================================================================================
void sensorsDigitalInit(const uint8_t IR_PINS[]);
uint16_t sensorsDigitalRead(const uint8_t IR_PINS[], uint8_t index);
void sensorsAnalogInit(const uint8_t IR_PINS[]);
uint16_t sensorsAnalogRead(const uint8_t IR_PINS[], uint8_t index);

// ================================================================================================================================================
// Logical sensor bits, kept separate from the GPIO numbers (IR_GPIO[] in config.h maps one to the other)
//
//   bit            0         1          2         3          4
//   sensor        IR1       IR2        IR3       IR4        IR5
//   position   leftmost  middle-left  centre  middle-right  rightmost
//   job        LEFT box     line       line      line      RIGHT box
//
// A bit is 1 when that sensor is over BLACK (SENSOR_BLACK_LEVEL already applied).
// ================================================================================================================================================
const uint8_t SENSOR_IR1 = 1 << 0;
const uint8_t SENSOR_IR2 = 1 << 1;
const uint8_t SENSOR_IR3 = 1 << 2;
const uint8_t SENSOR_IR4 = 1 << 3;
const uint8_t SENSOR_IR5 = 1 << 4;
const uint8_t SENSOR_BOX_MASK = SENSOR_IR1 | SENSOR_IR5;                 // the two marker sensors
const uint8_t SENSOR_MID_MASK = SENSOR_IR2 | SENSOR_IR3 | SENSOR_IR4;    // the three line sensors

// The middle sensors as a 3-bit pattern, written left to right: bit2 = IR2, bit1 = IR3, bit0 = IR4  (so 0b110 reads "IR2 IR3 black, IR4 white")
const uint8_t MID_NONE = 0b000;          // no line under the middle sensors
const uint8_t MID_LEFT = 0b100;          // line under IR2 only
const uint8_t MID_LEFT_CENTRE = 0b110;
const uint8_t MID_CENTRE = 0b010;        // line centred under IR3
const uint8_t MID_CENTRE_RIGHT = 0b011;
const uint8_t MID_RIGHT = 0b001;         // line under IR4 only
const uint8_t MID_OUTER = 0b101;         // IR2 and IR4 but not IR3: not a single line
const uint8_t MID_ALL = 0b111;           // thick box

void sensorsInit();                      // pinMode for the five logical sensors (SENSOR_USE_PULLUP)
uint8_t sensorsReadBits();               // logical bits, bit i = IR(i+1) sees black

inline uint8_t middlePattern(uint8_t bits) {
  return (uint8_t)(((bits & SENSOR_IR2) ? 0b100 : 0) | ((bits & SENSOR_IR3) ? 0b010 : 0) | ((bits & SENSOR_IR4) ? 0b001 : 0));
}
// "A plausible line": one line under one or two adjacent middle sensors. Never the thick box, never nothing, never the IR2+IR4 split.
inline bool isAnyLinePattern(uint8_t mid) {
  return mid == MID_CENTRE || mid == MID_LEFT_CENTRE || mid == MID_CENTRE_RIGHT || mid == MID_LEFT || mid == MID_RIGHT;
}
// A line that is already under IR3 (used when a search decides whether to stop sweeping).
inline bool isCentredLinePattern(uint8_t mid) {
  return mid == MID_CENTRE || mid == MID_LEFT_CENTRE || mid == MID_CENTRE_RIGHT;
}
// Writes the five bits IR1..IR5 as characters, e.g. "01110". buf must hold 6 bytes.
void sensorBitsToString(uint8_t bits, char* buf);

// ================================================================================================================================================
// Directional-marker detector (IR1 / IR5)
// One physical box produces exactly one event. See the marker section of config.h for the rules.
//
// A box is registered when its sensor was black for MARKER_CONFIRM_MS..MARKER_MAX_MS and then white again for MARKER_RELEASE_MS, with no
// suppression in between. Committing at the END of the box (not the start) is what rejects a line that runs into an intersection: such a line
// stays black into the thick box, the run is cut off by the suppression, and nothing is registered.
// ================================================================================================================================================
enum MarkerEvent : uint8_t { MARKER_NONE = 0, MARKER_LEFT, MARKER_RIGHT, MARKER_BOTH };

class MarkerDetector {
 public:
  MarkerDetector() { reset(); }
  void reset();   // forget partial boxes; each side must read white for MARKER_RELEASE_MS before it can register again

  // Call once per control tick with the logical sensor bits. While suppressed no event is produced, a box in progress is thrown away and both
  // sides must re-release (read white) afterwards. Returns the event completed on this tick, if any.
  MarkerEvent update(uint8_t bits, bool suppressed, uint32_t now);

 private:
  struct Side {
    bool armed = false;        // released (white long enough) and ready to see a box
    bool inBox = false;        // black for long enough: a box is passing
    bool done = false;         // the box ended cleanly; waiting out the pairing window
    uint32_t doneAt = 0;
    HoldTimer black;
    HoldTimer white;
  };
  Side left_;
  Side right_;
  static void updateSide(Side& s, bool black, bool suppressed, uint32_t now);
};

#endif
