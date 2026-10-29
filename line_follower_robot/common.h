#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

// A side of the robot, or a turn direction. DIR_NONE means "no information".
enum Direction : uint8_t { DIR_NONE = 0, DIR_LEFT = 1, DIR_RIGHT = 2 };

inline Direction oppositeDirection(Direction d) {
  return d == DIR_LEFT ? DIR_RIGHT : (d == DIR_RIGHT ? DIR_LEFT : DIR_NONE);
}

inline const char* directionName(Direction d) {
  return d == DIR_LEFT ? "LEFT" : (d == DIR_RIGHT ? "RIGHT" : "NONE");
}

inline int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline int absInt(int v) { return v < 0 ? -v : v; }

// "Has this condition been true continuously for N ms?" Every debounce / stable-reading rule in the
// program is built from this one little helper, so they all behave the same way.
struct HoldTimer {
  bool active = false;
  uint32_t since = 0;

  void update(bool condition, uint32_t now) {
    if (!condition) { active = false; return; }
    if (!active) { active = true; since = now; }
  }
  void reset() { active = false; }

  // How long the condition has been true, counting only time at or after notBefore (so that a reading
  // taken before a state started cannot count towards that state's stability requirement).
  uint32_t heldMs(uint32_t now, uint32_t notBefore) const {
    if (!active) return 0;
    const uint32_t start = (int32_t)(since - notBefore) > 0 ? since : notBefore;
    return now - start;
  }
  uint32_t heldMs(uint32_t now) const { return active ? now - since : 0; }
  bool heldFor(uint32_t ms, uint32_t now) const { return active && (now - since >= ms); }
  bool heldFor(uint32_t ms, uint32_t now, uint32_t notBefore) const {
    return active && heldMs(now, notBefore) >= ms;
  }
};

#endif
