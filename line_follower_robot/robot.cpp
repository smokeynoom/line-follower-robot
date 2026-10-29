#include "robot.h"

#include <math.h>
#include <stdio.h>

#include "common.h"
#include "config.h"
#include "diagnostics.h"
#include "motor.h"
#include "navigation.h"
#include "sensors.h"

namespace {

const char* const STATE_NAMES[STATE_COUNT] = {
  "STOPPED", "START_DELAY", "LINE_FOLLOWING", "INTERSECTION_DETECTED", "STRAIGHT_PATH_CHECK", "TURNING_LEFT", "TURNING_RIGHT",
  "LINE_REACQUISITION", "LOCAL_LINE_SEARCH", "WIDE_LINE_SEARCH", "RETRACE_RECOVERY", "FINISHED", "MOVEMENT_TEST",
};

// ================================================================================================================================================
// PD controller. Error: negative = line to the LEFT, positive = line to the RIGHT. The derivative is the error change over DERIVATIVE_WINDOW_MS
// (digital sensors change in steps, so a one-tick difference would be a single-tick spike that the motors never feel).
// ================================================================================================================================================
const uint8_t PD_SLOTS = 32;
const uint32_t PD_TICKS_RAW = DERIVATIVE_WINDOW_MS / CONTROL_PERIOD_MS;
const uint8_t PD_TICKS = (uint8_t)(PD_TICKS_RAW < 1 ? 1 : (PD_TICKS_RAW > PD_SLOTS ? PD_SLOTS : PD_TICKS_RAW));

struct PdController {
  float history[PD_SLOTS];
  uint8_t head = 0;
  float lastError = 0;

  PdController() { reset(0); }
  void reset(float error) {
    for (uint8_t i = 0; i < PD_SLOTS; i++) history[i] = error;
    head = 0;
    lastError = error;
  }
  float step(float error) {                       // returns (error now) - (error PD_TICKS ago)
    const float old = history[head];
    history[head] = error;
    head = (uint8_t)(head + 1 >= PD_TICKS ? 0 : head + 1);
    lastError = error;
    return error - old;
  }
};

enum TurnPhase : uint8_t { TURN_REPOSITION, TURN_BLIND, TURN_SEEK };

struct TestStep {
  const char* name;
  const char* expect;
  void (*apply)();
};

// ================================================================================================================================================
// All mutable state of the robot, in one place. robotSetup() starts from a fresh copy, so the program (and the host tests) are repeatable.
// ================================================================================================================================================
struct RobotCtx {
  RobotState state = STATE_STOPPED;
  uint32_t stateSince = 0;
  uint32_t lastTickUs = 0;
  uint32_t lastMotorUpdateMs = 0;
  uint32_t lastStatusLogMs = 0;
  uint16_t unexpectedResets = 0;

  // sensors of this tick
  uint8_t bits = 0;
  uint8_t mid = 0;
  HoldTimer thick;          // IR2+IR3+IR4 all black
  HoldTimer notThick;
  HoldTimer none;           // IR2+IR3+IR4 all white
  HoldTimer anyLine;        // a plausible single line under the middle sensors
  HoldTimer centredLine;    // ... and it is under IR3
  HoldTimer badPattern;     // 000 or 111
  bool thickRunActive = false;
  uint32_t thickRunStart = 0;

  // markers and navigation memory
  MarkerDetector markers;
  NavMemory nav;

  // run control
  uint32_t startDelayMs = 0;
  bool buttonRaw = false;
  bool buttonStable = false;
  uint32_t buttonChangedAt = 0;
  bool sensorMonitor = false;
  uint32_t lastMonitorMs = 0;

  // line following
  PdController pd;
  bool intersectionArmed = true;
  uint32_t intersectionDoneAt = 0;

  // intersection handling
  uint32_t intersectionDetectedAt = 0;
  uint16_t intersectionNumber = 0;
  Direction latchedTurn = DIR_NONE;
  bool straightProbing = false;
  uint32_t straightPhaseSince = 0;
  bool oversizeLogged = false;

  // turning
  Direction turnDir = DIR_NONE;
  TurnPhase turnPhase = TURN_BLIND;
  uint32_t turnPhaseSince = 0;
  uint32_t pivotStartedAt = 0;
  int32_t repositionMs = 0;
  bool turnSawClear = false;

  // reacquisition and recovery
  RobotState reacquireReturn = STATE_LOCAL_LINE_SEARCH;
  uint8_t recoveryCycle = 0;        // completed local->wide->retrace rounds in this episode
  uint8_t recoveryEpisodes = 0;     // line losses since the robot last made real progress
  Direction searchFirstSide = DIR_LEFT;
  uint8_t sweepIndex = 0;
  Direction sweepDir = DIR_LEFT;
  uint32_t sweepSince = 0;
  uint32_t sweepDurationMs = 0;
  uint32_t wideAmpMs = 0;
  uint32_t widePrevAmpMs = 0;
  bool retraceReversing = false;
  uint32_t retraceSince = 0;

  // movement test
  uint8_t testStep = 0;
  bool testStarted = false;
  bool testPaused = false;
  uint32_t testPhaseSince = 0;
};

RobotCtx rb;

// ---- forward declarations ---------------------------------------------------------------------------------------------------------------------
void enterState(RobotState next, uint32_t now);
void beginTurn(Direction dir, int32_t repositionMs, uint32_t now);
void beginReacquisition(RobotState returnTo, uint32_t now);
void beginRecovery(Direction firstSide, uint32_t now);
void beginLocalSearch(uint32_t now);
void beginWideSearch(uint32_t now);
void beginRetrace(uint32_t now);
void startRun(uint32_t delayMs, bool resetMemory, const char* why, uint32_t now);
void stopRun(const char* why, uint32_t now);

// ================================================================================================================================================
// Small helpers
// ================================================================================================================================================
bool runIsActive(RobotState s) {
  return s != STATE_STOPPED && s != STATE_MOVEMENT_TEST && s != STATE_FINISHED;
}

void saveRetained(uint32_t now) {
  RetainedState s;
  s.nav = rb.nav;
  s.wasRunning = runIsActive(rb.state);
  s.lastState = (uint8_t)rb.state;
  s.lastUptimeMs = now;
  s.unexpectedResets = rb.unexpectedResets;
  diagSaveRetained(s);
}

void enterState(RobotState next, uint32_t now) {
  char b[6];
  sensorBitsToString(rb.bits, b);
  logf("STATE %s -> %s | sensors=%s pending=%s lastMarker=%s", STATE_NAMES[rb.state], STATE_NAMES[next], b,
       directionName(rb.nav.pendingTurn), directionName(rb.nav.lastValidMarkerDirection));
  rb.state = next;
  rb.stateSince = now;
  saveRetained(now);
}

void pivot(Direction dir, int speed) {
  if (dir == DIR_LEFT) pivotLeft(speed);
  else pivotRight(speed);
}

// A pivot with some drive added to both wheels (a little reverse in the wide search), so it also explores ground instead of only rotating on the spot.
void sweepArc(Direction dir, int pivotPwm, int forwardBias) {
  if (dir == DIR_LEFT) setMotorSpeeds(forwardBias - pivotPwm, forwardBias + pivotPwm);
  else setMotorSpeeds(forwardBias + pivotPwm, forwardBias - pivotPwm);
}

bool lineErrorOf(uint8_t mid, float& error) {
  switch (mid) {
    case MID_CENTRE:       error = 0.0f;  return true;
    case MID_LEFT_CENTRE:  error = -0.5f; return true;
    case MID_LEFT:         error = -1.0f; return true;
    case MID_CENTRE_RIGHT: error = 0.5f;  return true;
    case MID_RIGHT:        error = 1.0f;  return true;
    default:               return false;     // 000, 111 and 101 carry no usable error: the caller keeps the previous one
  }
}

// PD steering. cruise = speed on a centred line. Never reverses a wheel: ordinary following is a gradual curve.
void followLine(int cruise) {
  float error = 0;
  if (lineErrorOf(rb.mid, error)) navNoteLineError(rb.nav, error);
  else error = rb.pd.lastError;

  const float derivative = rb.pd.step(error);
  float steer = PD_KP * error + PD_KD * derivative;
  if (steer > MAX_STEER_PWM) steer = MAX_STEER_PWM;
  if (steer < -MAX_STEER_PWM) steer = -MAX_STEER_PWM;

  int speed = cruise - (int)(CORNER_SLOWDOWN * fabsf(error));
  const int floorSpeed = cruise < MIN_FOLLOW_SPEED ? cruise : MIN_FOLLOW_SPEED;
  if (speed < floorSpeed) speed = floorSpeed;

  setMotorSpeeds(clampInt(speed + (int)steer, PD_MIN_WHEEL_PWM, MAX_PWM), clampInt(speed - (int)steer, PD_MIN_WHEEL_PWM, MAX_PWM));
}

// Recovery durations grow with every failed round and with repeated line losses at the same place, up to a cap.
uint32_t scaledMs(uint32_t base) {
  uint32_t k = rb.recoveryCycle + (rb.recoveryEpisodes > 0 ? rb.recoveryEpisodes - 1 : 0);
  if (k > SEARCH_MAX_GROWTH_CYCLES) k = SEARCH_MAX_GROWTH_CYCLES;
  return base + base * k * SEARCH_CYCLE_GROWTH_PERCENT / 100;
}

// ================================================================================================================================================
// Stop / start
// ================================================================================================================================================
void startRun(uint32_t delayMs, bool resetMemory, const char* why, uint32_t now) {
  logf("RUN START (%s): delay %lu ms%s", why, (unsigned long)delayMs, resetMemory ? ", marker memory cleared" : ", marker memory kept");
  if (resetMemory) navReset(rb.nav);
  rb.markers.reset();
  rb.intersectionArmed = true;
  rb.recoveryCycle = 0;
  rb.recoveryEpisodes = 0;
  rb.intersectionNumber = 0;
  rb.pd.reset(0);
  if (delayMs == 0) {
    enterState(STATE_LINE_FOLLOWING, now);
  } else {
    rb.startDelayMs = delayMs;
    motorStop();
    enterState(STATE_START_DELAY, now);
  }
}

void stopRun(const char* why, uint32_t now) {
  motorStop();
  logf("RUN STOP (%s): motors off, the control loop keeps running; BOOT or 'g' starts again", why);
  enterState(STATE_STOPPED, now);
}

void onButtonPressed(uint32_t now) {
  if (rb.state == STATE_STOPPED) startRun(MANUAL_START_DELAY_MS, RESET_NAV_MEMORY_ON_MANUAL_START, "BOOT button", now);
  else stopRun("BOOT button", now);
}

void handleButton(uint32_t now) {
  const bool down = digitalRead(PIN_BOOT_BUTTON) == LOW;
  if (down != rb.buttonRaw) {
    rb.buttonRaw = down;
    rb.buttonChangedAt = now;
  }
  if (rb.buttonStable != rb.buttonRaw && now - rb.buttonChangedAt >= BUTTON_DEBOUNCE_MS) {
    rb.buttonStable = rb.buttonRaw;
    if (rb.buttonStable) onButtonPressed(now);
  }
}

// ================================================================================================================================================
// Line following
// ================================================================================================================================================
void finishIntersection(Direction turnExecuted, uint32_t now) {
  navCompleteIntersection(rb.nav, turnExecuted);
  rb.intersectionArmed = false;                    // re-armed in updateLineFollowing() once the box is behind us
  rb.intersectionDoneAt = now;
  rb.recoveryEpisodes = 0;                         // an intersection was completed: that is real progress
}

void updateLineFollowing(uint32_t now) {
  if (rb.recoveryEpisodes > 0 && now - rb.stateSince >= PROGRESS_FOLLOW_MS) rb.recoveryEpisodes = 0;

  // Re-arm intersection detection once the previous box is behind us.
  if (!rb.intersectionArmed && rb.notThick.heldFor(INTERSECTION_REARM_CLEAR_MS, now) &&
      now - rb.intersectionDoneAt >= INTERSECTION_REARM_HOLDOFF_MS) {
    rb.intersectionArmed = true;
  }

  if (rb.mid == MID_ALL) {                         // all three middle sensors black: a possible intersection
    if (rb.intersectionArmed && rb.thick.heldFor(INTERSECTION_CONFIRM_MS, now)) {
      rb.intersectionDetectedAt = now;
      rb.intersectionNumber++;
      rb.latchedTurn = rb.nav.pendingTurn;         // latched now; pendingTurn itself is only consumed when the intersection completes
      rb.oversizeLogged = false;
      moveForward(INTERSECTION_PWM);
      enterState(STATE_INTERSECTION_DETECTED, now);
      return;
    }
    // Not confirmed yet (a box edge, or just a bend in the line) or already handled: do not steer on a thick pattern, hold the heading. (Holding
    // the LAST steering command instead was tried: at a box edge the last command comes from a transient partial pattern and yawed the robot.)
    moveForward(INTERSECTION_PWM);
    return;
  }

  if (rb.mid == MID_NONE) {                        // no line: ride out a brief dropout, then search
    if (rb.none.heldFor(LINE_LOST_CONFIRM_MS, now)) {
      char b[6];
      sensorBitsToString(rb.bits, b);
      logf("LINE LOST (sensors=%s), last line side=%s -> local search", b, directionName(rb.nav.lastLineErrorSide));
      beginRecovery(DIR_NONE, now);
    }
    return;                                        // the wheels keep their last command meanwhile
  }

  followLine(BASE_SPEED);
}

// ================================================================================================================================================
// Intersection: decide, then either test for a straight line or turn
// ================================================================================================================================================
void updateIntersectionDetected(uint32_t now) {
  moveForward(INTERSECTION_PWM);
  if (rb.latchedTurn != DIR_NONE) {
    logf("DECISION #%u TURN %s (marker)", rb.intersectionNumber, directionName(rb.latchedTurn));
    beginTurn(rb.latchedTurn, INTERSECTION_ADVANCE_MS - (int32_t)(now - rb.intersectionDetectedAt), now);
  } else {
    logf("INTERSECTION #%u: no pending marker -> testing for a straight line (last marker=%s)", rb.intersectionNumber,
         directionName(rb.nav.lastValidMarkerDirection));
    rb.straightProbing = false;
    enterState(STATE_STRAIGHT_PATH_CHECK, now);
  }
}

void straightFound(uint32_t now) {
  char b[6];
  sensorBitsToString(rb.bits, b);
  logf("DECISION #%u STRAIGHT (line found ahead, sensors=%s)", rb.intersectionNumber, b);
  finishIntersection(DIR_NONE, now);
  rb.pd.reset(0);
  enterState(STATE_LINE_FOLLOWING, now);
}

void straightMissing(uint32_t now) {
  const Direction turn = navFallbackTurn(rb.nav);
  logf("DECISION #%u FALLBACK %s (no marker, no straight line; last marker=%s, default=%s)", rb.intersectionNumber, directionName(turn),
       directionName(rb.nav.lastValidMarkerDirection), directionName(ROUTE_FALLBACK_NO_HISTORY));
  beginTurn(turn, INTERSECTION_ADVANCE_MS - (int32_t)(now - rb.intersectionDetectedAt), now);
}

void handleOversize(uint32_t now) {
  const bool outerBlack = (rb.bits & SENSOR_BOX_MASK) == SENSOR_BOX_MASK;
  if (!rb.oversizeLogged) {
    rb.oversizeLogged = true;
    logf("OVERSIZE black area: middle sensors black for %lu ms, outer sensors %s", (unsigned long)rb.thick.heldMs(now),
         outerBlack ? "black too (finish-box like)" : "not both black");
  }
  if (FINISH_STOP_ENABLED && (outerBlack || !FINISH_REQUIRE_OUTER_SENSORS)) {
    motorStop();
    logf("FINISH zone reached: stopping (FINISH_STOP_ENABLED). Resumes by itself once the sensors leave all-black for %lu ms.",
         (unsigned long)FINISH_RESUME_MS);
    enterState(STATE_FINISHED, now);
    return;
  }
  straightMissing(now);                            // not stopping: a black area this big is "no straight line"
}

// STRAIGHT-PATH TEST. Phase CLEAR: drive over the box. Phase PROBE: look for the outgoing line. The line-lost handler is not involved here.
void updateStraightCheck(uint32_t now) {
  moveForward(INTERSECTION_PWM);

  if (!rb.straightProbing) {
    if (rb.thick.heldMs(now) >= OVERSIZE_BLACK_MS) {
      handleOversize(now);
      return;
    }
    if (now - rb.intersectionDetectedAt >= STRAIGHT_CLEAR_MS && rb.notThick.heldFor(THICK_CLEAR_MS, now)) {
      rb.straightProbing = true;
      rb.straightPhaseSince = now;
    }
    return;
  }
  if (rb.anyLine.heldFor(STRAIGHT_STABLE_MS, now, rb.straightPhaseSince)) {
    straightFound(now);
    return;
  }
  if (now - rb.straightPhaseSince >= STRAIGHT_PROBE_MS) straightMissing(now);
}

// ================================================================================================================================================
// Turning: REPOSITION -> BLIND pivot -> SEEK pivot -> LINE_REACQUISITION. No encoders or gyro: durations are approximations, the sensors finish it.
// ================================================================================================================================================
void beginTurn(Direction dir, int32_t repositionMs, uint32_t now) {
  rb.turnDir = dir;
  rb.latchedTurn = dir;
  rb.repositionMs = (int32_t)clampInt((int)repositionMs, (int)-MAX_REPOSITION_MS, (int)MAX_REPOSITION_MS);
  rb.turnPhaseSince = now;
  if (rb.repositionMs != 0) {
    rb.turnPhase = TURN_REPOSITION;
  } else {
    rb.turnPhase = TURN_BLIND;
    rb.pivotStartedAt = now;
    rb.turnSawClear = !TURN_REQUIRE_CLEAR_GAP;
  }
  enterState(dir == DIR_LEFT ? STATE_TURNING_LEFT : STATE_TURNING_RIGHT, now);
}

void updateTurning(uint32_t now) {
  if (rb.turnPhase == TURN_REPOSITION) {
    if (rb.repositionMs > 0) moveForward(INTERSECTION_PWM);
    else moveBackward(INTERSECTION_PWM);
    if (now - rb.turnPhaseSince >= (uint32_t)absInt((int)rb.repositionMs)) {
      rb.turnPhase = TURN_BLIND;
      rb.turnPhaseSince = now;
      rb.pivotStartedAt = now;
      rb.turnSawClear = !TURN_REQUIRE_CLEAR_GAP;
    }
    return;
  }

  // The sensors must have seen "no line" at some point since the pivot began: that proves they left the box and the line we arrived on.
  if (!rb.turnSawClear && rb.none.heldFor(TURN_CLEAR_MS, now, rb.pivotStartedAt)) rb.turnSawClear = true;

  if (rb.turnPhase == TURN_BLIND) {
    pivot(rb.turnDir, TURN_PWM);
    if (now - rb.turnPhaseSince >= TURN_BLIND_MS) {
      rb.turnPhase = TURN_SEEK;
      rb.turnPhaseSince = now;
    }
    return;
  }

  pivot(rb.turnDir, TURN_SEEK_PWM);
  if (rb.turnSawClear && rb.centredLine.heldFor(TURN_CAPTURE_MS, now, rb.turnPhaseSince)) {
    beginReacquisition(rb.state, now);
    return;
  }
  if (now - rb.pivotStartedAt >= TURN_SEEK_PATIENCE_MS) {          // never stop: hand over to the search, keeping the turn side as the first guess
    logf("TURN %s: no line found after %lu ms of pivoting -> recovery search", directionName(rb.turnDir), (unsigned long)(now - rb.pivotStartedAt));
    beginRecovery(rb.turnDir, now);
  }
}

// ================================================================================================================================================
// Line reacquisition: a candidate line was seen; creep forward and believe it only when a plausible pattern stays stable.
// ================================================================================================================================================
void beginReacquisition(RobotState returnTo, uint32_t now) {
  rb.reacquireReturn = returnTo;
  rb.pd.reset(0);
  enterState(STATE_LINE_REACQUISITION, now);
}

void updateReacquisition(uint32_t now) {
  followLine(REACQUIRE_PWM);

  if (rb.anyLine.heldFor(REACQUIRE_STABLE_MS, now, rb.stateSince)) {
    const bool fromTurn = rb.reacquireReturn == STATE_TURNING_LEFT || rb.reacquireReturn == STATE_TURNING_RIGHT;
    char b[6];
    sensorBitsToString(rb.bits, b);
    if (fromTurn) {
      logf("TURN %s complete: line reacquired (sensors=%s)", directionName(rb.turnDir), b);
      finishIntersection(rb.turnDir, now);
    } else {
      logf("LINE reacquired after search (sensors=%s)", b);
    }
    rb.pd.reset(0);
    enterState(STATE_LINE_FOLLOWING, now);
    return;
  }
  if (rb.badPattern.heldFor(REACQUIRE_DROP_MS, now, rb.stateSince)) {   // false alarm: carry on exactly where we were
    logf("REACQUISITION dropped (line not stable), resuming %s", STATE_NAMES[rb.reacquireReturn]);
    if (rb.reacquireReturn == STATE_TURNING_LEFT || rb.reacquireReturn == STATE_TURNING_RIGHT) rb.turnPhaseSince = now;
    rb.sweepSince = now;
    enterState(rb.reacquireReturn, now);
  }
}

// ================================================================================================================================================
// Recovery. Never ends, never stops the wheels: LOCAL -> WIDE -> RETRACE -> LOCAL (longer, other side first) -> ...
// ================================================================================================================================================
void beginRecovery(Direction firstSide, uint32_t now) {
  if (firstSide == DIR_NONE) firstSide = rb.nav.lastLineErrorSide != DIR_NONE ? rb.nav.lastLineErrorSide : DEFAULT_SEARCH_SIDE;
  rb.searchFirstSide = firstSide;
  rb.recoveryCycle = 0;
  if (rb.recoveryEpisodes < 255) rb.recoveryEpisodes++;
  beginLocalSearch(now);
}

bool searchCaptured(uint32_t now) {
  if (rb.centredLine.heldFor(SEARCH_CAPTURE_MS, now, rb.stateSince)) {
    beginReacquisition(rb.state, now);
    return true;
  }
  return false;
}

void beginLocalSearch(uint32_t now) {
  rb.sweepIndex = 0;
  rb.sweepDir = (rb.recoveryCycle % 2 == 0) ? rb.searchFirstSide : oppositeDirection(rb.searchFirstSide);
  rb.sweepSince = now;
  rb.sweepDurationMs = scaledMs(LOCAL_SWEEP_MS);
  enterState(STATE_LOCAL_LINE_SEARCH, now);
}

void updateLocalSearch(uint32_t now) {
  pivot(rb.sweepDir, SEARCH_PIVOT_PWM);
  if (searchCaptured(now)) return;
  if (now - rb.sweepSince < rb.sweepDurationMs) return;
  if (++rb.sweepIndex >= LOCAL_SEARCH_SWEEPS) {
    beginWideSearch(now);
    return;
  }
  rb.sweepDir = oppositeDirection(rb.sweepDir);    // sweep to the other side, twice as far, so the whole neighbourhood is covered
  rb.sweepSince = now;
  rb.sweepDurationMs = 2 * scaledMs(LOCAL_SWEEP_MS);
}

void beginWideSearch(uint32_t now) {
  rb.sweepIndex = 0;
  rb.sweepDir = oppositeDirection(rb.sweepDir);
  rb.wideAmpMs = scaledMs(WIDE_SWEEP_START_MS);
  rb.widePrevAmpMs = 0;
  rb.sweepSince = now;
  rb.sweepDurationMs = rb.wideAmpMs;
  enterState(STATE_WIDE_LINE_SEARCH, now);
}

void updateWideSearch(uint32_t now) {
  sweepArc(rb.sweepDir, SEARCH_PIVOT_PWM, WIDE_BIAS_PWM);
  if (searchCaptured(now)) return;
  if (now - rb.sweepSince < rb.sweepDurationMs) return;
  if (++rb.sweepIndex >= WIDE_SEARCH_SWEEPS) {
    beginRetrace(now);
    return;
  }
  rb.sweepDir = oppositeDirection(rb.sweepDir);
  rb.widePrevAmpMs = rb.wideAmpMs;
  const uint32_t cap = scaledMs(WIDE_SWEEP_MAX_MS);
  const uint32_t grown = rb.wideAmpMs * WIDE_GROWTH_PERCENT / 100;
  rb.wideAmpMs = grown > cap ? cap : grown;
  rb.sweepSince = now;
  rb.sweepDurationMs = rb.widePrevAmpMs + rb.wideAmpMs;   // swing back across the middle and out to the new, larger extreme
}

// APPROXIMATE retrace: back up for a calibrated time, then search wide. The robot cannot know its real path or guarantee to return along it.
void beginRetrace(uint32_t now) {
  rb.retraceReversing = true;
  rb.retraceSince = now;
  logf("RETRACE: reversing %lu ms, then a wide search (approximate; no encoders or gyro)", (unsigned long)scaledMs(RETRACE_REVERSE_MS));
  enterState(STATE_RETRACE_RECOVERY, now);
}

void updateRetrace(uint32_t now) {
  if (rb.retraceReversing) {
    moveBackward(RETRACE_PWM);
    if (searchCaptured(now)) return;
    if (now - rb.retraceSince < scaledMs(RETRACE_REVERSE_MS)) return;
    rb.retraceReversing = false;
    rb.sweepIndex = 0;
    // The history decides which side to try first: the last intersection turn if the last decision was a turn, else the last line error.
    rb.sweepDir = rb.nav.lastTurnExecuted != DIR_NONE ? rb.nav.lastTurnExecuted
                  : (rb.nav.lastLineErrorSide != DIR_NONE ? rb.nav.lastLineErrorSide : DEFAULT_SEARCH_SIDE);
    rb.wideAmpMs = scaledMs(WIDE_SWEEP_MAX_MS);
    rb.sweepSince = now;
    rb.sweepDurationMs = rb.wideAmpMs / 2;
    return;
  }

  sweepArc(rb.sweepDir, SEARCH_PIVOT_PWM, 0);
  if (searchCaptured(now)) return;
  if (now - rb.sweepSince < rb.sweepDurationMs) return;
  if (++rb.sweepIndex >= RETRACE_SEARCH_SWEEPS) {
    if (rb.recoveryCycle < 255) rb.recoveryCycle++;
    logf("RECOVERY round %u failed -> local search again, wider, other side first", rb.recoveryCycle);
    beginLocalSearch(now);
    return;
  }
  rb.sweepDir = oppositeDirection(rb.sweepDir);
  rb.sweepSince = now;
  rb.sweepDurationMs = rb.wideAmpMs;
}

// ================================================================================================================================================
// FINISHED (only with FINISH_STOP_ENABLED): motors off, but not waiting for BOOT — it resumes when the sensors change.
// ================================================================================================================================================
void updateFinished(uint32_t now) {
  if (rb.notThick.heldFor(FINISH_RESUME_MS, now)) {
    logf("FINISH: sensors changed (robot moved or lifted) -> resuming autonomous following");
    rb.pd.reset(0);
    rb.intersectionArmed = false;
    rb.intersectionDoneAt = now;
    enterState(STATE_LINE_FOLLOWING, now);
  }
}

// ================================================================================================================================================
// Manual movement test (serial 'w'): every movement function in turn, so each wheel direction can be checked with the robot lifted.
// ================================================================================================================================================
void testLeftOnly() { setMotorSpeeds(MOVEMENT_TEST_PWM, 0); }
void testRightOnly() { setMotorSpeeds(0, MOVEMENT_TEST_PWM); }
void testForward() { moveForward(MOVEMENT_TEST_PWM); }
void testBackward() { moveBackward(MOVEMENT_TEST_PWM); }
void testPivotLeft() { pivotLeft(MOVEMENT_TEST_PWM); }
void testPivotRight() { pivotRight(MOVEMENT_TEST_PWM); }
void testCurveLeft() { curveLeft(MOVEMENT_TEST_PWM, MOVEMENT_TEST_PWM / 2); }
void testCurveRight() { curveRight(MOVEMENT_TEST_PWM, MOVEMENT_TEST_PWM / 2); }

const TestStep TEST_STEPS[] = {
  {"LEFT wheel only, forward", "left wheel forward, right wheel still", testLeftOnly},
  {"RIGHT wheel only, forward", "right wheel forward, left wheel still", testRightOnly},
  {"moveForward", "both wheels forward, robot drives straight ahead", testForward},
  {"moveBackward", "both wheels backward", testBackward},
  {"pivotLeft", "left wheel backward + right wheel forward: robot spins ANTICLOCKWISE seen from above", testPivotLeft},
  {"pivotRight", "left wheel forward + right wheel backward: robot spins CLOCKWISE seen from above", testPivotRight},
  {"curveLeft", "both forward, left wheel slower: robot curves LEFT", testCurveLeft},
  {"curveRight", "both forward, right wheel slower: robot curves RIGHT", testCurveRight},
};
const uint8_t TEST_STEP_COUNT = sizeof(TEST_STEPS) / sizeof(TEST_STEPS[0]);

void updateMovementTest(uint32_t now) {
  if (rb.testStep >= TEST_STEP_COUNT) {
    motorStop();
    logf("Movement test finished");
    enterState(STATE_STOPPED, now);
    return;
  }
  if (rb.testPaused) {
    if (now - rb.testPhaseSince >= MOVEMENT_TEST_PAUSE_MS) {
      rb.testStep++;
      rb.testPaused = false;
      rb.testStarted = false;
    }
    return;
  }
  if (!rb.testStarted) {
    logf("TEST %u/%u %s -> expect: %s", rb.testStep + 1, TEST_STEP_COUNT, TEST_STEPS[rb.testStep].name, TEST_STEPS[rb.testStep].expect);
    TEST_STEPS[rb.testStep].apply();
    rb.testStarted = true;
    rb.testPhaseSince = now;
  } else if (now - rb.testPhaseSince >= MOVEMENT_TEST_STEP_MS) {
    motorStop();
    rb.testPaused = true;
    rb.testPhaseSince = now;
  }
}

// ================================================================================================================================================
// Serial console and periodic diagnostics
// ================================================================================================================================================
void logStatus(uint32_t now) {
  char b[6];
  sensorBitsToString(rb.bits, b);
  logf("STATUS %s sensors=%s mid=%d%d%d pending=%s lastMarker=%s lineSide=%s wheels L=%d R=%d crossings=%u markers=%u logDropped=%lu",
       STATE_NAMES[rb.state], b, (rb.mid >> 2) & 1, (rb.mid >> 1) & 1, rb.mid & 1, directionName(rb.nav.pendingTurn),
       directionName(rb.nav.lastValidMarkerDirection), directionName(rb.nav.lastLineErrorSide), motorLeftApplied(), motorRightApplied(),
       rb.nav.intersectionsCompleted, rb.nav.markerCount, (unsigned long)diagDroppedLogLines());
  saveRetained(now);
}

void printHelp() {
  logf("Commands: g = start   x = stop motors   w = wheel movement test (only when stopped, robot lifted)   m = sensor monitor on/off   ? = this help");
}

void pollConsole(uint32_t now) {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    switch (c) {
      case 'g': case 'G':
        if (rb.state == STATE_STOPPED) startRun(MANUAL_START_DELAY_MS, RESET_NAV_MEMORY_ON_MANUAL_START, "serial g", now);
        else logf("'g' ignored: not stopped (state %s)", STATE_NAMES[rb.state]);
        break;
      case 'x': case 'X':
        if (rb.state != STATE_STOPPED) stopRun("serial x", now);
        break;
      case 'w': case 'W':
        if (rb.state == STATE_STOPPED) {
          rb.testStep = 0;
          rb.testStarted = false;
          rb.testPaused = false;
          enterState(STATE_MOVEMENT_TEST, now);
        } else {
          logf("'w' ignored: stop the robot first with 'x' (state %s)", STATE_NAMES[rb.state]);
        }
        break;
      case 'm': case 'M':
        rb.sensorMonitor = !rb.sensorMonitor;
        logf("Sensor monitor %s (IR1..IR5 = leftmost..rightmost, 1 = black)", rb.sensorMonitor ? "ON" : "OFF");
        break;
      case '?': case 'h': case 'H':
        printHelp();
        logStatus(now);
        break;
      default:
        break;
    }
  }
}

void periodicLogs(uint32_t now) {
  if (LOG_PERIODIC_STATUS && now - rb.lastStatusLogMs >= STATUS_LOG_PERIOD_MS) {
    rb.lastStatusLogMs = now;
    logStatus(now);
  }
  if (rb.sensorMonitor && now - rb.lastMonitorMs >= SENSOR_MONITOR_PERIOD_MS) {
    rb.lastMonitorMs = now;
    char b[6];
    sensorBitsToString(rb.bits, b);
    logf("SENSORS IR1..IR5 = %c %c %c %c %c   (GPIO %u %u %u %u %u)", b[0], b[1], b[2], b[3], b[4], IR_GPIO[0], IR_GPIO[1], IR_GPIO[2],
         IR_GPIO[3], IR_GPIO[4]);
  }
}

// ================================================================================================================================================
// One control tick
// ================================================================================================================================================
void controlTick(uint32_t now) {
  rb.bits = sensorsReadBits();
  rb.mid = middlePattern(rb.bits);
  rb.thick.update(rb.mid == MID_ALL, now);
  rb.notThick.update(rb.mid != MID_ALL, now);
  rb.none.update(rb.mid == MID_NONE, now);
  rb.anyLine.update(isAnyLinePattern(rb.mid), now);
  rb.centredLine.update(isCentredLinePattern(rb.mid), now);
  rb.badPattern.update(rb.mid == MID_NONE || rb.mid == MID_ALL, now);

  if (LOG_THICK_RUNS) {      // how long each all-black run lasts: the data for calibrating INTERSECTION_CONFIRM_MS
    if (rb.mid == MID_ALL && !rb.thickRunActive) {
      rb.thickRunActive = true;
      rb.thickRunStart = now;
    } else if (rb.mid != MID_ALL && rb.thickRunActive) {
      rb.thickRunActive = false;
      const uint32_t ms = now - rb.thickRunStart;
      if (ms >= THICK_RUN_LOG_MIN_MS) {
        logf("THICK run ended after %lu ms in %s: %s (a box needs >= %lu ms)", (unsigned long)ms, STATE_NAMES[rb.state],
             ms >= INTERSECTION_CONFIRM_MS ? "long enough for an intersection" : "too short, treated as a bend / noise",
             (unsigned long)INTERSECTION_CONFIRM_MS);
      }
    }
  }

  handleButton(now);

  // Markers count only while following the line, and never while IR2+IR3+IR4 are all black. A thick pattern shorter than MARKER_SUPPRESS_AFTER_MS
  // (the robot veering so that IR4 touches the marker box, or a bend) only PAUSES the detector; a longer one (a real box) suppresses it and
  // throws away whatever box was in progress, e.g. a line that runs into the intersection.
  const bool hardSuppress = rb.state != STATE_LINE_FOLLOWING || rb.thick.heldFor(MARKER_SUPPRESS_AFTER_MS, now);
  const bool pausedByBlip = rb.mid == MID_ALL && !hardSuppress;
  const MarkerEvent ev = pausedByBlip ? MARKER_NONE : rb.markers.update(rb.bits, hardSuppress, now);
  if (ev != MARKER_NONE) {
    navOnMarkerEvent(rb.nav, ev);
    logf("MARKER %s -> pending=%s lastMarker=%s", ev == MARKER_LEFT ? "LEFT" : (ev == MARKER_RIGHT ? "RIGHT" : "BOTH (cancels pending)"),
         directionName(rb.nav.pendingTurn), directionName(rb.nav.lastValidMarkerDirection));
    saveRetained(now);
  }

  switch (rb.state) {
    case STATE_STOPPED:                                      break;   // motors already off; sensors, button and console stay alive
    case STATE_START_DELAY:
      if (now - rb.stateSince >= rb.startDelayMs) enterState(STATE_LINE_FOLLOWING, now);
      break;
    case STATE_LINE_FOLLOWING:        updateLineFollowing(now);      break;
    case STATE_INTERSECTION_DETECTED: updateIntersectionDetected(now); break;
    case STATE_STRAIGHT_PATH_CHECK:   updateStraightCheck(now);      break;
    case STATE_TURNING_LEFT:
    case STATE_TURNING_RIGHT:         updateTurning(now);            break;
    case STATE_LINE_REACQUISITION:    updateReacquisition(now);      break;
    case STATE_LOCAL_LINE_SEARCH:     updateLocalSearch(now);        break;
    case STATE_WIDE_LINE_SEARCH:      updateWideSearch(now);         break;
    case STATE_RETRACE_RECOVERY:      updateRetrace(now);            break;
    case STATE_FINISHED:              updateFinished(now);           break;
    case STATE_MOVEMENT_TEST:         updateMovementTest(now);       break;
    default:                          break;
  }

  motorUpdate(rb.lastMotorUpdateMs == 0 ? CONTROL_PERIOD_MS : now - rb.lastMotorUpdateMs);
  rb.lastMotorUpdateMs = now;
  periodicLogs(now);
}

}  // namespace

// ================================================================================================================================================
// Public API
// ================================================================================================================================================
void robotSetup() {
  rb = RobotCtx();
  diagBegin();
  motorInit(PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB);
  sensorsInit();
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  rb.lastTickUs = micros();
  const uint32_t now = millis();

  logf("Wheels: LEFT = Motor %c, RIGHT = Motor %c, Motor A %s, Motor B %s. Sensors: black = %s, IR1..IR5 = GPIO %u %u %u %u %u",
       LEFT_WHEEL_IS_MOTOR_A ? 'A' : 'B', LEFT_WHEEL_IS_MOTOR_A ? 'B' : 'A', MOTOR_A_INVERT ? "inverted" : "normal",
       MOTOR_B_INVERT ? "inverted" : "normal", SENSOR_BLACK_LEVEL == HIGH ? "HIGH" : "LOW", IR_GPIO[0], IR_GPIO[1], IR_GPIO[2], IR_GPIO[3],
       IR_GPIO[4]);
  logf("Send '?' for commands. Auto-start %s.", AUTO_START_ON_BOOT ? "ON (competition mode)" : "OFF (bench mode: press BOOT or send g)");

  RetainedState kept;
  const bool restored = diagLoadRetained(kept);
  rb.unexpectedResets = kept.unexpectedResets;
  if (restored) {
    rb.nav = kept.nav;
    logf("UNEXPECTED RESET #%u. Before it: state=%s, uptime %lu ms, run %s. Marker memory restored: pending=%s lastMarker=%s.",
         kept.unexpectedResets, kept.lastState < STATE_COUNT ? STATE_NAMES[kept.lastState] : "?", (unsigned long)kept.lastUptimeMs,
         kept.wasRunning ? "was ACTIVE" : "was stopped", directionName(kept.nav.pendingTurn), directionName(kept.nav.lastValidMarkerDirection));
    if (RESUME_RUN_AFTER_UNEXPECTED_RESET && kept.wasRunning) {
      startRun(AUTO_RESTART_DELAY_MS, false, "resuming after unexpected reset", now);
      return;
    }
    if (!kept.wasRunning) {                      // the robot had been stopped on purpose: a reset must not make it drive off
      motorStop();
      saveRetained(now);
      logf("STATE STOPPED (the unexpected reset happened while stopped). Press BOOT or send g to start.");
      return;
    }
    navReset(rb.nav);                            // resume is disabled: treat it like a fresh power-up below
  } else {
    navReset(rb.nav);
  }

  if (AUTO_START_ON_BOOT) {
    startRun(AUTO_START_DELAY_MS, true, "auto-start", now);
  } else {
    motorStop();
    saveRetained(now);
    logf("STATE STOPPED (bench mode). Press BOOT or send g to start.");
  }
}

void robotLoop() {
  const uint32_t nowUs = micros();
  const uint32_t periodUs = CONTROL_PERIOD_MS * 1000UL;
  if ((uint32_t)(nowUs - rb.lastTickUs) < periodUs) return;          // no tick due: return at once, nothing here ever waits
  rb.lastTickUs = ((uint32_t)(nowUs - rb.lastTickUs) > 5 * periodUs) ? nowUs : rb.lastTickUs + periodUs;
  const uint32_t now = millis();
  controlTick(now);
  pollConsole(now);
}

void robotRequestStart() {
  const uint32_t now = millis();
  if (rb.state == STATE_STOPPED) startRun(MANUAL_START_DELAY_MS, RESET_NAV_MEMORY_ON_MANUAL_START, "request", now);
}

void robotRequestStop() {
  const uint32_t now = millis();
  if (rb.state != STATE_STOPPED) stopRun("request", now);
}

RobotState robotState() { return rb.state; }
const char* robotStateName(RobotState s) { return s < STATE_COUNT ? STATE_NAMES[s] : "?"; }
const NavMemory& robotNavMemory() { return rb.nav; }
