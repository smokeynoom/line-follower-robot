// Host tests for the line follower firmware. The real firmware sources are compiled against test/mock/ and driven by the simulator in sim.h.
// Usage:  line_follower_tests [test-name ...]      (no arguments = run everything)
//         LF_TRACE=trace.csv line_follower_tests arena     (also write the robot's path for plotting / debugging)
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <functional>
#include <string>
#include <vector>

#include "esp_system.h"
#include "sim.h"
#include "config.h"
#include "diagnostics.h"
#include "motor.h"
#include "navigation.h"
#include "robot.h"
#include "sensors.h"

static int g_checks = 0;
static int g_failures = 0;
static bool g_skipped = false;     // set by a test that does not apply to this build configuration

#define CHECK(cond)                                                              \
  do {                                                                           \
    g_checks++;                                                                  \
    if (!(cond)) { g_failures++; printf("    FAIL line %d: %s\n", __LINE__, #cond); } \
  } while (0)
#define CHECK_MSG(cond, ...)                                                     \
  do {                                                                           \
    g_checks++;                                                                  \
    if (!(cond)) { g_failures++; printf("    FAIL line %d: %s -- ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static void powerOn(int reason = ESP_RST_POWERON) {
  mock::reset();
  mock::resetReason = reason;
  robotSetup();
}

// the three middle sensors as IR2 IR3 IR4 bits
static uint8_t bitsMid(uint8_t mid) {
  return (uint8_t)(((mid & 0b100) ? SENSOR_IR2 : 0) | ((mid & 0b010) ? SENSOR_IR3 : 0) | ((mid & 0b001) ? SENSOR_IR4 : 0));
}

static int countLines(const std::string& needle) { return (int)sim::logLinesContaining(mock::serialOut, needle).size(); }

// ================================================================================================================================================
// Marker detector: one box = one event, double markers cancel, suppression, release / re-arm
// ================================================================================================================================================
struct MarkerRig {
  MarkerDetector d;
  uint32_t t = 0;
  int left = 0, right = 0, both = 0;
  bool suppressed = false;
  void run(uint8_t bits, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
      t++;
      switch (d.update(bits, suppressed, t)) {
        case MARKER_LEFT: left++; break;
        case MARKER_RIGHT: right++; break;
        case MARKER_BOTH: both++; break;
        default: break;
      }
    }
  }
};

static void testMarkerDetector() {
  const uint8_t L = SENSOR_IR1, R = SENSOR_IR5;
  // An event is committed at the END of a box (after the release time) plus the pairing window, so every case finishes with a long white stretch.
  {  // a sensor that is already black at start-up registers nothing until it has been released (white) once
    MarkerRig m; m.run(L, 300);
    CHECK(m.left == 0);
    m.run(0, 50); m.run(L, 100); m.run(0, 250);
    CHECK_MSG(m.left == 1 && m.right == 0 && m.both == 0, "left=%d right=%d both=%d", m.left, m.right, m.both);
  }
  {  // a single box on each side
    MarkerRig m; m.run(0, 100); m.run(L, 80); m.run(0, 250); m.run(R, 80); m.run(0, 250);
    CHECK(m.left == 1 && m.right == 1 && m.both == 0);
  }
  {  // both sides at once -> exactly one BOTH event, no single events
    MarkerRig m; m.run(0, 100); m.run(L | R, 80); m.run(0, 250);
    CHECK(m.both == 1 && m.left == 0 && m.right == 0);
  }
  {  // both sides, one box starting 25 ms later than the other, ending together -> still BOTH
    MarkerRig m; m.run(0, 100); m.run(L, 25); m.run(L | R, 80); m.run(0, 250);
    CHECK(m.both == 1 && m.left == 0 && m.right == 0);
  }
  {  // both sides but 150 ms apart (two different boxes) -> LEFT then RIGHT
    MarkerRig m; m.run(0, 100); m.run(L, 60); m.run(0, 150); m.run(R, 60); m.run(0, 250);
    CHECK_MSG(m.left == 1 && m.right == 1 && m.both == 0, "left=%d right=%d both=%d", m.left, m.right, m.both);
  }
  {  // a 5 ms blip is not a box
    MarkerRig m; m.run(0, 100); m.run(L, 5); m.run(0, 250);
    CHECK(m.left == 0 && m.right == 0 && m.both == 0);
  }
  {  // one long black stretch is one event; a stretch longer than MARKER_MAX_MS is a line, not a marker
    MarkerRig m; m.run(0, 100); m.run(R, 500); m.run(0, 250);
    CHECK(m.right == 1 && m.left == 0 && m.both == 0);
    MarkerRig n; n.run(0, 100); n.run(R, MARKER_MAX_MS + 200); n.run(0, 250);
    CHECK(n.right == 0 && n.left == 0 && n.both == 0);
  }
  {  // flicker inside one box (10 ms of white, shorter than the release time) does not make a second event
    MarkerRig m; m.run(0, 100); m.run(L, 40); m.run(0, 10); m.run(L, 40); m.run(0, 250);
    CHECK_MSG(m.left == 1, "left=%d", m.left);
  }
  {  // two boxes separated by enough white make two events
    MarkerRig m; m.run(0, 100); m.run(L, 60); m.run(0, 100); m.run(L, 60); m.run(0, 250);
    CHECK(m.left == 2);
  }
  {  // suppressed (e.g. all three middle sensors black) while a box passes: no event, and nothing leaks out when suppression ends
    MarkerRig m; m.run(0, 100);
    m.suppressed = true; m.run(L, 100); m.suppressed = false; m.run(L, 100);
    CHECK(m.left == 0);
    m.run(0, 60); m.run(L, 60); m.run(0, 250);
    CHECK(m.left == 1);
  }
  {  // a LINE that runs into the thick box (seen by IR1 just before the box, the real failure at the top of a diagonal): the black run is
     // cut off by the suppression, so it is never counted, even after the sensor goes white again
    MarkerRig m; m.run(0, 100); m.run(L, 80);
    m.suppressed = true; m.run(L, 300); m.suppressed = false; m.run(0, 300);
    CHECK_MSG(m.left == 0 && m.right == 0 && m.both == 0, "left=%d right=%d both=%d", m.left, m.right, m.both);
  }
}

// The marker rules as the whole firmware applies them (suppression comes from the state machine, not from the detector alone).
static void testMarkerSuppressionInRobot() {
  {  // IR4 touching the marker box for a few ms (all-black blip) must not cancel the marker
    powerOn();
    sim::Sim s; s.scripted = bitsMid(0b010); s.run(3.3);
    CHECK(robotState() == STATE_LINE_FOLLOWING);
    s.scripted = bitsMid(0b010) | SENSOR_IR1; s.run(0.05);
    s.scripted = bitsMid(0b111) | SENSOR_IR1; s.run(0.015);                  // 15 ms blip, shorter than MARKER_SUPPRESS_AFTER_MS
    s.scripted = bitsMid(0b010) | SENSOR_IR1; s.run(0.05);
    s.scripted = bitsMid(0b010); s.run(0.3);
    CHECK_MSG(robotNavMemory().pendingTurn == DIR_LEFT, "pending=%s", directionName(robotNavMemory().pendingTurn));
  }
  {  // a line that runs into a real box: the leftmost sensor stays black into the thick pattern, so nothing may be registered
    powerOn();
    sim::Sim s; s.scripted = bitsMid(0b010); s.run(3.3);
    s.scripted = bitsMid(0b010) | SENSOR_IR1; s.run(0.08);
    s.scripted = bitsMid(0b111) | SENSOR_IR1; s.run(0.4);                    // a real thick box
    s.scripted = bitsMid(0b010); s.run(0.4);
    CHECK_MSG(robotNavMemory().markerCount == 0 && robotNavMemory().pendingTurn == DIR_NONE, "markerCount=%u pending=%s",
              robotNavMemory().markerCount, directionName(robotNavMemory().pendingTurn));
  }
  {  // a marker, then the intersection: the pending direction is latched, used by the turn, and consumed when the turn completes
    powerOn();
    sim::Sim s; s.scripted = bitsMid(0b010); s.run(3.3);
    s.scripted = bitsMid(0b010) | SENSOR_IR5; s.run(0.08);
    s.scripted = bitsMid(0b010); s.run(0.3);
    CHECK(robotNavMemory().pendingTurn == DIR_RIGHT);
    s.scripted = bitsMid(0b111); s.run(0.25);                                // the box is confirmed after INTERSECTION_CONFIRM_MS
    CHECK(robotState() == STATE_TURNING_RIGHT || robotState() == STATE_INTERSECTION_DETECTED);
    CHECK(robotNavMemory().pendingTurn == DIR_RIGHT);                        // not consumed merely by entering the intersection
    s.scripted = 0; s.run(0.3);                                              // the box is behind us: the pivot's blind phase runs
    s.scripted = bitsMid(0b010); s.run(1.0);                                 // a line appears under IR3 while pivoting right: turn completes
    CHECK_MSG(robotState() == STATE_LINE_FOLLOWING, "state=%s", robotStateName(robotState()));
    CHECK(robotNavMemory().pendingTurn == DIR_NONE && robotNavMemory().lastValidMarkerDirection == DIR_RIGHT);
    CHECK(robotNavMemory().lastTurnExecuted == DIR_RIGHT);
  }
}

// ================================================================================================================================================
// Navigation memory: pendingTurn / lastValidMarkerDirection / fallback
// ================================================================================================================================================
static void testNavigationRules() {
  NavMemory n;
  CHECK(n.pendingTurn == DIR_NONE && n.lastValidMarkerDirection == DIR_NONE);
  CHECK(navFallbackTurn(n) == ROUTE_FALLBACK_NO_HISTORY && ROUTE_FALLBACK_NO_HISTORY == DIR_LEFT);   // the arena's required left turn

  navOnMarkerEvent(n, MARKER_LEFT);
  CHECK(n.pendingTurn == DIR_LEFT && n.lastValidMarkerDirection == DIR_LEFT);
  navOnMarkerEvent(n, MARKER_RIGHT);                                         // the most recent marker overrides
  CHECK(n.pendingTurn == DIR_RIGHT && n.lastValidMarkerDirection == DIR_RIGHT);
  navOnMarkerEvent(n, MARKER_BOTH);                                          // double marker: bias cancelled, history untouched
  CHECK(n.pendingTurn == DIR_NONE && n.lastValidMarkerDirection == DIR_RIGHT);
  CHECK(navFallbackTurn(n) == DIR_RIGHT);

  navOnMarkerEvent(n, MARKER_LEFT);
  navCompleteIntersection(n, DIR_LEFT);                                      // used: pending consumed, history kept (this is the fallback for the next, unmarked box)
  CHECK(n.pendingTurn == DIR_NONE && n.lastValidMarkerDirection == DIR_LEFT && n.lastTurnExecuted == DIR_LEFT);
  CHECK(navFallbackTurn(n) == DIR_LEFT);

  navNoteLineError(n, -0.5f); CHECK(n.lastLineErrorSide == DIR_LEFT);
  navNoteLineError(n, 0.0f);  CHECK(n.lastLineErrorSide == DIR_LEFT);        // zero error does not erase the side
  navNoteLineError(n, 1.0f);  CHECK(n.lastLineErrorSide == DIR_RIGHT);
}

// ================================================================================================================================================
// Motor layer: every movement function, checked against the simulator's own (handout) wiring truth
// ================================================================================================================================================
static void testMotorMapping() {
  mock::reset();
  motorInit(PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB);
  sim::Sim s;
  auto settle = [&]() { motorUpdate(20); motorUpdate(20); motorUpdate(20); };

  moveForward(150); settle();
  CHECK(s.wheelTarget(true) > 0 && s.wheelTarget(false) > 0);
  CHECK(fabs(s.wheelTarget(true) - s.wheelTarget(false)) < 1e-6);

  moveBackward(150); settle();
  CHECK(s.wheelTarget(true) < 0 && s.wheelTarget(false) < 0);

  pivotLeft(150); settle();                                  // left wheel back, right wheel forward = anticlockwise
  CHECK(s.wheelTarget(true) < 0 && s.wheelTarget(false) > 0);
  pivotRight(150); settle();
  CHECK(s.wheelTarget(true) > 0 && s.wheelTarget(false) < 0);

  curveLeft(150, 60); settle();                              // both forward, left slower
  CHECK(s.wheelTarget(true) > 0 && s.wheelTarget(true) < s.wheelTarget(false));
  curveRight(150, 60); settle();
  CHECK(s.wheelTarget(false) > 0 && s.wheelTarget(false) < s.wheelTarget(true));

  leftForward(150); rightForward(0);                         // workshop API, one wheel at a time
  CHECK(s.wheelTarget(true) > 0 && s.wheelTarget(false) == 0);
  rightReverse(150); leftForward(0);
  CHECK(s.wheelTarget(false) < 0 && s.wheelTarget(true) == 0);

  moveForward(150); settle();
  motorStop();                                               // intentional stop: both off at once, direction pins low too ...
  CHECK(s.wheelTarget(true) == 0 && s.wheelTarget(false) == 0);
  CHECK(mock::pinLevel[PIN_IN1] == LOW && mock::pinLevel[PIN_IN2] == LOW && mock::pinLevel[PIN_IN3] == LOW && mock::pinLevel[PIN_IN4] == LOW);
  moveForward(150); settle();                                // ... and the motors run again when told to
  CHECK(s.wheelTarget(true) > 0 && s.wheelTarget(false) > 0);

  setMotorSpeeds(10, -10); settle();                         // dead zone: a small nonzero command is lifted to MIN_MOVE_PWM
  CHECK(mock::pwmDuty[PIN_ENA] == (uint32_t)MIN_MOVE_PWM && mock::pwmDuty[PIN_ENB] == (uint32_t)MIN_MOVE_PWM);

  motorStop();
  setMotorSpeeds(255, 255); motorUpdate(2);                  // slew limit: only MOTOR_SLEW_PWM_PER_MS * 2 ms on the first step
  CHECK(motorLeftApplied() == MOTOR_SLEW_PWM_PER_MS * 2);
}

// ================================================================================================================================================
// PD sign convention: line left -> steer left, line right -> steer right, centred -> straight
// ================================================================================================================================================
static void testPdSigns() {
  powerOn();
  sim::Sim s;
  s.scripted = bitsMid(0b010);
  s.run(3.3);
  CHECK(robotState() == STATE_LINE_FOLLOWING);
  s.run(0.2);
  CHECK(abs(motorLeftApplied() - motorRightApplied()) <= 2);                       // centred: straight
  CHECK(motorLeftApplied() >= BASE_SPEED - 2 && motorRightApplied() >= BASE_SPEED - 2);

  s.scripted = bitsMid(0b100); s.run(0.3);                                         // line under IR2 (left): left wheel slower -> curves left
  CHECK(motorLeftApplied() < motorRightApplied() && motorLeftApplied() >= 0);
  s.scripted = bitsMid(0b110); s.run(0.3);
  CHECK(motorLeftApplied() < motorRightApplied());
  s.scripted = bitsMid(0b001); s.run(0.3);                                         // line under IR4 (right): right wheel slower -> curves right
  CHECK(motorRightApplied() < motorLeftApplied() && motorRightApplied() >= 0);
  s.scripted = bitsMid(0b011); s.run(0.3);
  CHECK(motorRightApplied() < motorLeftApplied());
  s.scripted = bitsMid(0b010); s.run(0.3);
  CHECK(abs(motorLeftApplied() - motorRightApplied()) <= 2);
  CHECK(s.maxZeroRunMs < 20);
  CHECK(robotNavMemory().lastLineErrorSide == DIR_RIGHT);                          // last off-centre reading was on the right
}

// ================================================================================================================================================
// Never-stop soak tests: whatever the sensors say, a running robot keeps its wheels commanded
// ================================================================================================================================================
static std::vector<std::string> statesVisited() {
  std::vector<std::string> v;
  for (const std::string& line : sim::logLinesContaining(mock::serialOut, "STATE ")) {
    const size_t arrow = line.find(" -> ");
    if (arrow == std::string::npos) continue;
    const size_t end = line.find(" |", arrow);
    v.push_back(line.substr(arrow + 4, end == std::string::npos ? std::string::npos : end - arrow - 4));
  }
  return v;
}
static bool visited(const std::vector<std::string>& v, const char* name) {
  for (const std::string& s : v) if (s == name) return true;
  return false;
}

static void testSoakAllWhite() {         // the robot is lifted off the track onto white floor: the line is never found
  powerOn();
  sim::Sim s; s.scripted = 0; s.run(180);
  const auto v = statesVisited();
  CHECK(robotState() != STATE_STOPPED && robotState() != STATE_FINISHED);
  CHECK_MSG(s.maxZeroRunMs < 60, "both wheels were off for %u ms", s.maxZeroRunMs);
  CHECK(visited(v, "LOCAL_LINE_SEARCH") && visited(v, "WIDE_LINE_SEARCH") && visited(v, "RETRACE_RECOVERY"));
  CHECK_MSG(countLines("RECOVERY round") >= 5, "only %d recovery rounds in 180 s", countLines("RECOVERY round"));
  printf("    180 s with no line: %d recovery rounds, longest both-wheels-off run %u ms\n", countLines("RECOVERY round"), s.maxZeroRunMs);
}

static void testSoakAllBlack() {         // an unexpected pattern for ever: black everywhere (e.g. wrong sensor polarity, or a lifted robot)
  powerOn();
  sim::Sim s; s.scripted = 0b11111; s.run(120);
  CHECK(robotState() != STATE_STOPPED);
  CHECK_MSG(s.maxZeroRunMs < 60, "both wheels were off for %u ms", s.maxZeroRunMs);
  CHECK_MSG(countLines("DECISION #") == 1, "the same black area was treated as %d intersections", countLines("DECISION #"));
  printf("    120 s all-black: %d decision, longest both-wheels-off run %u ms\n", countLines("DECISION #"), s.maxZeroRunMs);
}

static void testSoakRandomNoise() {      // random sensor chatter, changing every few ms
  powerOn();
  sim::Sim s; s.scripted = bitsMid(0b010);
  uint32_t seed = 12345;
  auto rnd = [&]() { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed; };
  double elapsed = 0;
  while (elapsed < 120) {
    s.scripted = (int)(rnd() % 32);
    const double d = (3 + rnd() % 60) / 1000.0;
    s.run(d);
    elapsed += d;
  }
  CHECK(robotState() != STATE_STOPPED);
  CHECK_MSG(s.maxZeroRunMs < 60, "both wheels were off for %u ms", s.maxZeroRunMs);
  printf("    120 s of random sensor noise: longest both-wheels-off run %u ms\n", s.maxZeroRunMs);
}

// ================================================================================================================================================
// Intentional stop / start: BOOT button and serial. The loop must stay alive while stopped.
// ================================================================================================================================================
static void pressBoot(sim::Sim& s) {
  mock::pinLevel[PIN_BOOT_BUTTON] = LOW;  s.run(0.12);
  mock::pinLevel[PIN_BOOT_BUTTON] = HIGH; s.run(0.12);
}

static void testButtonAndSerial() {
  powerOn();
  sim::Sim s; s.scripted = bitsMid(0b010);
  s.run(3.4);
  CHECK(robotState() == STATE_LINE_FOLLOWING && !s.bothWheelsOff());

  pressBoot(s);                                              // BOOT = intentional stop
  CHECK(robotState() == STATE_STOPPED && s.bothWheelsOff());
  const int statusBefore = countLines("STATUS");
  s.run(2.1);                                                // the loop is still alive: status lines keep coming, sensors are read
  CHECK(countLines("STATUS") > statusBefore + 2);
  mock::serialIn = "m"; s.run(0.5);
  CHECK(countLines("SENSORS IR1..IR5") >= 3);
  mock::serialIn = "m"; s.run(0.1);

  pressBoot(s);                                              // BOOT again: start, with the short manual delay, then drive
  CHECK(robotState() == STATE_START_DELAY);
  s.run(MANUAL_START_DELAY_MS / 1000.0 + 0.3);
  CHECK(robotState() == STATE_LINE_FOLLOWING && !s.bothWheelsOff());

  mock::serialIn = "x"; s.run(0.1);                          // serial stop and start
  CHECK(robotState() == STATE_STOPPED && s.bothWheelsOff());
  mock::serialIn = "g"; s.run(MANUAL_START_DELAY_MS / 1000.0 + 0.3);
  CHECK(robotState() == STATE_LINE_FOLLOWING && !s.bothWheelsOff());
  CHECK(countLines("RUN STOP") == 2 && countLines("RUN START") >= 3);
}

// ================================================================================================================================================
// Movement test (serial 'w'): every movement function reaches the right pins
// ================================================================================================================================================
static void testMovementTest() {
  powerOn();
  sim::Sim s; s.scripted = bitsMid(0b010);
  s.run(0.2);
  mock::serialIn = "x"; s.run(0.1);
  CHECK(robotState() == STATE_STOPPED);

  mock::serialIn = "w"; s.run(0.01);
  CHECK(robotState() == STATE_MOVEMENT_TEST);
  struct Expect { const char* name; int l, r; int rel; };   // sign of left / right wheel; rel: -1 left slower, +1 right slower, 0 n/a
  const Expect expect[8] = {{"LEFT only", +1, 0, 0}, {"RIGHT only", 0, +1, 0}, {"forward", +1, +1, 0}, {"backward", -1, -1, 0},
                            {"pivotLeft", -1, +1, 0}, {"pivotRight", +1, -1, 0}, {"curveLeft", +1, +1, -1}, {"curveRight", +1, +1, +1}};
  const double stepS = (MOVEMENT_TEST_STEP_MS + MOVEMENT_TEST_PAUSE_MS) / 1000.0;
  s.run(0.7);
  for (int k = 0; k < 8; k++) {
    const double tl = s.wheelTarget(true), tr = s.wheelTarget(false);
    const int sl = tl > 0 ? 1 : (tl < 0 ? -1 : 0), sr = tr > 0 ? 1 : (tr < 0 ? -1 : 0);
    CHECK_MSG(sl == expect[k].l && sr == expect[k].r, "step %d (%s): left %.0f right %.0f", k + 1, expect[k].name, tl, tr);
    if (expect[k].rel < 0) CHECK(tl < tr);
    if (expect[k].rel > 0) CHECK(tr < tl);
    s.run(stepS);
  }
  s.run(0.5);
  CHECK(robotState() == STATE_STOPPED && s.bothWheelsOff());
}

// ================================================================================================================================================
// Reset handling: an unexpected reset (brownout etc.) is reported, marker memory comes back, a running robot resumes without BOOT
// ================================================================================================================================================
static void softReset(int reason) {          // what a brownout does: time restarts, pins keep their level, RTC memory survives
  mock::timeUs = 0;
  mock::resetReason = reason;
  mock::serialOut.clear();
  robotSetup();
}

static void testResetHandling() {
  powerOn();
  sim::Sim s; s.scripted = bitsMid(0b010);
  s.run(3.3);
  CHECK(robotState() == STATE_LINE_FOLLOWING);
  s.scripted = bitsMid(0b010) | SENSOR_IR1; s.run(0.1);       // a LEFT box goes by
  s.scripted = bitsMid(0b010); s.run(0.2);
  CHECK(robotNavMemory().pendingTurn == DIR_LEFT);

  softReset(ESP_RST_BROWNOUT);                                // power dipped: the chip reset itself in the middle of the run
  CHECK(countLines("UNEXPECTED RESET #1") == 1);
  CHECK(mock::serialOut.find("BROWNOUT") != std::string::npos);
  CHECK(robotState() == STATE_START_DELAY);                   // resuming by itself, no BOOT press needed
  CHECK(robotNavMemory().pendingTurn == DIR_LEFT && robotNavMemory().lastValidMarkerDirection == DIR_LEFT);
  s.run(AUTO_RESTART_DELAY_MS / 1000.0 + 0.2);
  CHECK(robotState() == STATE_LINE_FOLLOWING && !s.bothWheelsOff());

  softReset(ESP_RST_TASK_WDT);                                // a second one is counted
  CHECK(countLines("UNEXPECTED RESET #2") == 1);
  s.run(0.5);
  CHECK(robotState() == STATE_LINE_FOLLOWING);

  mock::serialIn = "x"; s.run(0.1);                           // stopped on purpose, then the chip resets: it must not drive off
  CHECK(robotState() == STATE_STOPPED);
  softReset(ESP_RST_PANIC);
  CHECK(robotState() == STATE_STOPPED);
  s.run(1.0);
  CHECK(robotState() == STATE_STOPPED && s.bothWheelsOff());

  powerOn();                                                  // a normal power-up: clean memory, normal auto-start delay
  CHECK(robotNavMemory().pendingTurn == DIR_NONE && robotNavMemory().lastValidMarkerDirection == DIR_NONE);
  CHECK(robotState() == STATE_START_DELAY);
  CHECK(countLines("UNEXPECTED") == 0);
}

// ================================================================================================================================================
// The arena
// ================================================================================================================================================
static std::string decisionText(const std::string& line) {      // "[123] DECISION #4 TURN RIGHT (marker)" -> "TURN RIGHT"
  const size_t h = line.find('#');
  const size_t sp = line.find(' ', h);
  const size_t paren = line.find(" (", sp);
  return line.substr(sp + 1, paren == std::string::npos ? std::string::npos : paren - sp - 1);
}

static void traceRow(FILE* f, const sim::Sim& s) {
  if (!f) return;
  char bits[6];
  sensorBitsToString(s.geometryBits(), bits);
  fprintf(f, "%.4f,%.1f,%.1f,%.4f,%s,%d,%d,%s\n", s.nowSeconds(), s.x, s.y, s.th, robotStateName(robotState()), motorLeftApplied(), motorRightApplied(), bits);
}

struct ArenaResult {
  bool finished = false;
  double finishSeconds = 0;
  std::vector<std::string> decisions;     // "TURN LEFT", "STRAIGHT", ... in the order the firmware made them
  int lostEvents = 0;
  uint32_t maxZeroMs = 0;
  RobotState endState = STATE_STOPPED;
  bool endWheelsOff = false;
};

static const char* const EXPECTED_ROUTE[12] = {"TURN LEFT", "TURN RIGHT", "TURN RIGHT", "TURN RIGHT", "TURN RIGHT", "TURN LEFT",
                                               "FALLBACK LEFT", "TURN LEFT", "STRAIGHT", "TURN RIGHT", "TURN RIGHT", "STRAIGHT"};

static bool routeMatches(const ArenaResult& r) {
  if (r.decisions.size() < 12) return false;
  for (int i = 0; i < 12; i++) if (r.decisions[i] != EXPECTED_ROUTE[i]) return false;
  return true;
}

// Power the firmware on at the START line and let it drive until a few seconds after IR3 enters the finish box (or 240 s pass).
static ArenaResult runArena(const sim::RobotModel& model, double markerOffsetMm, double settleAfterFinishS, bool debugOutputs) {
  powerOn();
  sim::Sim s(markerOffsetMm);
  s.model = model;
  s.placeAtStart();
  FILE* trace = (debugOutputs && getenv("LF_TRACE")) ? fopen(getenv("LF_TRACE"), "w") : nullptr;
  if (trace) fprintf(trace, "t,x,y,theta,state,left,right,sensors\n");
  // Debug aid (LF_THICK=1): how long is IR2+IR3+IR4 all black each time, and where? Boxes and track bends can both do it.
  const bool thickDebug = debugOutputs && getenv("LF_THICK") != nullptr;
  double thickStart = -1, thickX = 0, thickY = 0;

  ArenaResult r;
  const uint64_t maxSteps = (uint64_t)(240 * 1e6 / sim::Sim::STEP_US);
  for (uint64_t i = 0; i < maxSteps; i++) {
    s.step();
    if (trace && i % 20 == 0) traceRow(trace, s);
    if (thickDebug) {
      const bool thickNow = middlePattern(s.geometryBits()) == MID_ALL;
      if (thickNow && thickStart < 0) { thickStart = s.nowSeconds(); thickX = s.x / sim::MM_PER_PX; thickY = sim::IMG_H - s.y / sim::MM_PER_PX; }
      if (!thickNow && thickStart >= 0) {
        printf("    thick run: %5.0f ms  at px(%4.0f,%4.0f)  t=%.2f s  state=%s\n", (s.nowSeconds() - thickStart) * 1000, thickX, thickY, thickStart, robotStateName(robotState()));
        thickStart = -1;
      }
    }
    if (s.irThreeInFinish() && !r.finished) { r.finished = true; r.finishSeconds = s.nowSeconds(); }
    if (r.finished && s.nowSeconds() > r.finishSeconds + settleAfterFinishS) break;
  }
  if (trace) fclose(trace);
  if (debugOutputs && getenv("LF_LOG")) {                          // the firmware's whole serial output, for debugging
    FILE* f = fopen(getenv("LF_LOG"), "w");
    if (f) { fwrite(mock::serialOut.data(), 1, mock::serialOut.size(), f); fclose(f); }
  }
  for (const std::string& line : sim::logLinesContaining(mock::serialOut, "DECISION #")) r.decisions.push_back(decisionText(line));
  r.lostEvents = countLines("LINE LOST");
  r.maxZeroMs = s.maxZeroRunMs;
  r.endState = robotState();
  r.endWheelsOff = s.bothWheelsOff();
  return r;
}

static void testArenaRun() {
  const ArenaResult r = runArena(sim::RobotModel(), 48, 6.0, true);
  printf("    decisions made before/at the finish:\n");
  for (size_t i = 0; i < r.decisions.size() && i < 16; i++) printf("      %2zu  %s\n", i + 1, r.decisions[i].c_str());

  CHECK_MSG(r.finished, "the robot never reached the finish box");
  if (r.finished) printf("    reached the finish box after %.1f s\n", r.finishSeconds);
  CHECK_MSG(r.decisions.size() >= 12, "only %zu decisions", r.decisions.size());
  for (size_t i = 0; i < 12 && i < r.decisions.size(); i++) {
    CHECK_MSG(r.decisions[i] == EXPECTED_ROUTE[i], "decision %zu was '%s', expected '%s'", i + 1, r.decisions[i].c_str(), EXPECTED_ROUTE[i]);
  }
  printf("    line-lost events: %d, STRAIGHT tests: %d, longest both-wheels-off run: %u ms\n", r.lostEvents, countLines("testing for a straight line"), r.maxZeroMs);
  CHECK_MSG(r.maxZeroMs < 60, "both wheels were commanded off for %u ms", r.maxZeroMs);
  CHECK(countLines("RUN STOP") == 0);
#if FINISH_STOP_ENABLED
  CHECK(r.endState == STATE_FINISHED && r.endWheelsOff);
  CHECK(countLines("FINISH zone reached") == 1);
#else
  CHECK(r.endState != STATE_STOPPED && r.endState != STATE_FINISHED);
#endif
}

// The same route with the simulated robot built a little differently (speed, sensor geometry, motor lag, dead zone, track width, marker
// placement). The firmware's timed numbers are tuned for the nominal robot, so this shows how much slack the logic has, not that any robot works.
static void testRobustnessSweep() {
  struct Variant { const char* name; bool nominal; std::function<void(sim::RobotModel&, double&)> apply; };
  const std::vector<Variant> variants = {
    {"baseline", true, [](sim::RobotModel&, double&) {}},
    {"speed -8%", true, [](sim::RobotModel& m, double&) { m.vMaxMmS *= 0.92; }},
    {"speed +8%", true, [](sim::RobotModel& m, double&) { m.vMaxMmS *= 1.08; }},
    {"sensor bar 5 mm further forward", true, [](sim::RobotModel& m, double&) { m.sensorForwardMm += 5; }},
    {"sensor bar 5 mm further back", true, [](sim::RobotModel& m, double&) { m.sensorForwardMm -= 5; }},
    {"motor lag 30 ms", true, [](sim::RobotModel& m, double&) { m.tauS = 0.03; }},
    {"motor lag 80 ms", true, [](sim::RobotModel& m, double&) { m.tauS = 0.08; }},
    {"dead zone 45", true, [](sim::RobotModel& m, double&) { m.deadDuty = 45; }},
    {"dead zone 65", true, [](sim::RobotModel& m, double&) { m.deadDuty = 65; }},
    {"track width 120 mm", true, [](sim::RobotModel& m, double&) { m.trackMm = 120; }},
    {"track width 145 mm", true, [](sim::RobotModel& m, double&) { m.trackMm = 145; }},
    {"IR2/IR4 pitch 14 mm", true, [](sim::RobotModel& m, double&) { m.midPitchMm = 14; }},
    {"IR2/IR4 pitch 18 mm", true, [](sim::RobotModel& m, double&) { m.midPitchMm = 18; }},
    {"markers 6 mm off the outer sensors' path", true, [](sim::RobotModel&, double& off) { off = 54; }},
    {"markers 6 mm the other way", true, [](sim::RobotModel&, double& off) { off = 42; }},
    {"speed -15% (stress)", false, [](sim::RobotModel& m, double&) { m.vMaxMmS *= 0.85; }},
    {"speed +15% (stress)", false, [](sim::RobotModel& m, double&) { m.vMaxMmS *= 1.15; }},
    {"sensor bar 15 mm further forward (stress)", false, [](sim::RobotModel& m, double&) { m.sensorForwardMm += 15; }},
  };
  int nominalPassed = 0, nominalTotal = 0;
  printf("    %-44s %-9s %-9s %s\n", "variant", "finish", "route", "line-lost");
  const char* only = getenv("LF_ONLY");                  // e.g. LF_ONLY="baseline,lag" runs just the variants whose name contains one of these
  for (const Variant& v : variants) {
    if (only) {
      bool match = false;
      std::string list = only;
      size_t pos = 0;
      while (pos <= list.size()) {
        size_t comma = list.find(',', pos);
        if (comma == std::string::npos) comma = list.size();
        const std::string token = list.substr(pos, comma - pos);
        if (!token.empty() && std::string(v.name).find(token) != std::string::npos) match = true;
        pos = comma + 1;
      }
      if (!match) continue;
    }
    sim::RobotModel m;
    double markerOffset = 48;
    v.apply(m, markerOffset);
    const ArenaResult r = runArena(m, markerOffset, 0.5, only != nullptr);   // with LF_ONLY the LF_LOG / LF_TRACE files are written too
    const bool ok = r.finished && routeMatches(r) && r.maxZeroMs < 60;
    printf("    %-44s %-9s %-9s %d   %s\n", v.name, r.finished ? "reached" : "NO", routeMatches(r) ? "all 12 ok" : "WRONG", r.lostEvents, v.nominal ? "" : "(informational)");
    if (v.nominal) { nominalTotal++; if (ok) nominalPassed++; }
    CHECK_MSG(!v.nominal || ok, "nominal variant '%s' failed", v.name);
  }
  printf("    nominal variants passed: %d / %d\n", nominalPassed, nominalTotal);
}

// Build with -DFINISH_STOP_ENABLED=true: the robot stops inside the finish box by itself, but does not wait for BOOT to carry on.
static void testFinishStopAndResume() {
#if !FINISH_STOP_ENABLED
  g_skipped = true;
  printf("    (skipped: FINISH_STOP_ENABLED is false in this build; run with EXTRA_FLAGS=\"-DFINISH_STOP_ENABLED=true\")\n");
#else
  powerOn();
  sim::Sim s;
  s.placeAtStart();
  const bool reached = s.runUntil([]() { return robotState() == STATE_FINISHED; }, 120);
  CHECK_MSG(reached, "never entered STATE_FINISHED");
  s.run(0.5);
  CHECK(robotState() == STATE_FINISHED && s.bothWheelsOff());
  CHECK(s.irThreeInFinish());                                    // it stopped INSIDE the finish box
  printf("    stopped in the finish box after %.1f s\n", s.nowSeconds());

  const int statusBefore = countLines("STATUS");                 // still alive while stopped
  s.run(2.0);
  CHECK(countLines("STATUS") > statusBefore + 2);
  CHECK(robotState() == STATE_FINISHED);

  s.scripted = 0;                                                // the robot is picked up / pushed out: sensors read white
  s.run(FINISH_RESUME_MS / 1000.0 + 0.5);
  CHECK_MSG(robotState() != STATE_FINISHED && robotState() != STATE_STOPPED, "state=%s", robotStateName(robotState()));
  s.run(2.0);
  CHECK(!s.bothWheelsOff());                                     // moving again, no BOOT needed
  CHECK(countLines("RUN STOP") == 0);
#endif
}

// Build with -DAUTO_START_ON_BOOT=false: bench mode waits for BOOT; the default (competition) build starts by itself.
static void testBenchMode() {
#if AUTO_START_ON_BOOT
  g_skipped = true;
  printf("    (skipped: AUTO_START_ON_BOOT is true in this build; run with EXTRA_FLAGS=\"-DAUTO_START_ON_BOOT=false\")\n");
#else
  powerOn();
  sim::Sim s; s.scripted = bitsMid(0b010);
  s.run(10);
  CHECK(robotState() == STATE_STOPPED && s.bothWheelsOff());     // waits for BOOT, as configured
  CHECK(countLines("STATUS") > 5);                               // but the loop is alive
  pressBoot(s);
  s.run(MANUAL_START_DELAY_MS / 1000.0 + 0.3);
  CHECK(robotState() == STATE_LINE_FOLLOWING && !s.bothWheelsOff());
#endif
}

// A robot dropped beside the start line, crooked, must find the line and carry on to the first intersection.
static void testRecoversWhenPlacedOffLine() {
  powerOn();
  sim::Sim s;
  s.placeAtStart();
  s.x += 55;                       // 55 mm to the right of the line
  s.th = sim::PI / 2 - 0.45;          // pointing about 26 degrees to the right of north
  const bool got = s.runUntil([]() { return countLines("DECISION #1 ") > 0; }, 90);
  CHECK_MSG(got, "never reached the first intersection after being placed off the line");
  CHECK(countLines("LINE LOST") >= 1);
  CHECK(s.maxZeroRunMs < 60);
  printf("    off-line start: first intersection after %.1f s, %d line-lost events\n", s.nowSeconds(), countLines("LINE LOST"));
}

// ================================================================================================================================================
struct TestCase { const char* name; void (*fn)(); };
static const TestCase TESTS[] = {
  {"markers", testMarkerDetector},
  {"marker_suppression", testMarkerSuppressionInRobot},
  {"navigation", testNavigationRules},
  {"motors", testMotorMapping},
  {"pd", testPdSigns},
  {"soak_white", testSoakAllWhite},
  {"soak_black", testSoakAllBlack},
  {"soak_noise", testSoakRandomNoise},
  {"button_serial", testButtonAndSerial},
  {"movement_test", testMovementTest},
  {"reset", testResetHandling},
  {"arena", testArenaRun},
  {"sweep", testRobustnessSweep},
  {"off_line_start", testRecoversWhenPlacedOffLine},
  {"finish_stop", testFinishStopAndResume},
  {"bench_mode", testBenchMode},
};

int main(int argc, char** argv) {
  int ran = 0, failedTests = 0, skipped = 0;
  for (const TestCase& t : TESTS) {
    bool want = argc <= 1;
    for (int i = 1; i < argc; i++) if (strcmp(argv[i], t.name) == 0) want = true;
    if (!want) continue;
    const int before = g_failures;
    g_skipped = false;
    printf("[ RUN  ] %s\n", t.name);
    t.fn();
    const bool ok = g_failures == before;
    printf("[ %s ] %s\n", !ok ? "FAIL" : (g_skipped ? "SKIP" : " OK "), t.name);
    if (g_skipped) { skipped++; continue; }
    ran++;
    if (!ok) failedTests++;
  }
  printf("\n%d tests run, %d failed, %d skipped (other build configuration), %d checks, %d check failures\n", ran, failedTests, skipped, g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
