#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "common.h"

/*
  ==================================================================================================================================================
   CONFIGURATION — every value you may need to calibrate on the real robot and the real arena lives in this file.
  ==================================================================================================================================================
   Sections:  1 Pins   2 Sensor polarity   3 Motors   4 Control loop & PD   5 Markers   6 Intersections & straight test
              7 Turning   8 Line recovery   9 Arena route rules   10 Finish zone   11 Start / stop / reset   12 Diagnostics

   IMPORTANT: no encoders and no gyroscope are fitted. Every duration below that makes the robot move "a bit" (advance, pivot, reverse) is a
   TIMED APPROXIMATION. It is not a measured distance or angle, and it changes with battery voltage, wheel grip and floor surface.
   The default numbers are starting guesses (tuned only in the host simulator in test/), not calibrated values for your robot.
   Re-calibrate them on the arena, in the order of the checklist in README.md.

   Conventions used everywhere:
     * Wheel commands are SIGNED PWM:  +1..+255 = wheel drives the robot forward,  -1..-255 = reverse,  0 = off.
     * A steering error is NEGATIVE when the line is to the robot's LEFT and POSITIVE when it is to the RIGHT.
     * "ms" values are measured with millis(); the program never blocks, so none of them stalls the control loop.
*/

// ================================================================================================================================================
// 1. PINS (ESP32 GPIO numbers)
// ================================================================================================================================================
// Logical sensor order IR1 -> IR5 = bit 0 -> bit 4 everywhere in the program (this is NOT the GPIO number, and not the handout's "IR_1..IR_5").
//   IR1 = leftmost (LEFT box sensor)   IR2 = middle-left (line)   IR3 = centre (line)   IR4 = middle-right (line)   IR5 = rightmost (RIGHT box sensor)
// The handout lists GPIO 18,19,21,22,23 as its IR_1..IR_5. The project wiring table (and the earlier sketches) mount GPIO23 on the LEFT.
// If the sensor test (serial 'm') shows left/right swapped on your robot, just reverse this array.
const uint8_t IR_GPIO[5] = {23, 22, 21, 19, 18};

const int PIN_IN1 = 27;   // Motor A direction
const int PIN_IN2 = 26;
const int PIN_IN3 = 13;   // Motor B direction
const int PIN_IN4 = 25;
const int PIN_ENA = 14;   // Motor A PWM enable
const int PIN_ENB = 33;   // Motor B PWM enable
const int PIN_BOOT_BUTTON = 0;   // BOOT button, active LOW. Manual start / stop.

// ================================================================================================================================================
// 2. SENSOR POLARITY   <<< CALIBRATE THIS FIRST >>>
// ================================================================================================================================================
// The level that digitalRead() returns when a sensor is over BLACK.
// The MECC handout says the modules are active-low on reflection: white drives DO to 0, black leaves DO floating and INPUT_PULLUP reads 1,
// so BLACK = HIGH. An earlier sketch assumed the opposite. Do not trust either: run the sensor test (serial 'm') with the robot on the arena,
// put each sensor over the black line and check the bit turns to 1. If the bits are inverted, change HIGH to LOW here. The sensitivity
// (distance at which white / black flips) is the little potentiometer on each module.
#ifndef SENSOR_BLACK_LEVEL
#define SENSOR_BLACK_LEVEL HIGH
#endif
const bool SENSOR_USE_PULLUP = true;   // INPUT_PULLUP (the handout's setting) instead of plain INPUT

// ================================================================================================================================================
// 3. MOTORS
// ================================================================================================================================================
// Which driver channel is the robot's physical LEFT wheel? In the handout Motor A (IN1/IN2/ENA) is the LEFT wheel, Motor B the RIGHT wheel.
const bool LEFT_WHEEL_IS_MOTOR_A = true;
// Direction inversion per driver channel. Un-inverted, "forward" means IN(first)=HIGH, IN(second)=LOW. The handout says the right motor is
// mounted facing the opposite way, so its pattern is mirrored: Motor B is inverted. If a wheel spins the wrong way in the movement test
// (serial 'w'), flip that channel's flag.
const bool MOTOR_A_INVERT = false;
const bool MOTOR_B_INVERT = true;

const uint32_t PWM_FREQ_HZ = 5000;        // PWM frequency on ENA / ENB
const uint8_t PWM_RESOLUTION_BITS = 8;    // 8 bits = duty 0..255. Do not raise without also changing MAX_PWM.
const int MAX_PWM = 255;                  // hard ceiling for any wheel command
const int MIN_MOVE_PWM = 70;              // dead zone: the lowest PWM that reliably turns a wheel (nonzero commands below it are raised to it)
const float LEFT_PWM_SCALE = 1.00f;       // compensate for one motor being stronger: scale each wheel's command (e.g. 0.95)
const float RIGHT_PWM_SCALE = 1.00f;
// Largest PWM change per millisecond. Reversing a wheel at speed makes a current spike that can sag the supply and brown the ESP32 out;
// the slew limit ramps through zero instead. Set to 255 to disable. The robot never stops either wheel on purpose while it is running.
const int MOTOR_SLEW_PWM_PER_MS = 10;

// ================================================================================================================================================
// 4. CONTROL LOOP & LINE FOLLOWING (PD)
// ================================================================================================================================================
const uint32_t CONTROL_PERIOD_MS = 2;     // sensors are read and the state machine runs every 2 ms (500 Hz)

// Middle-sensor error mapping (IR2 IR3 IR4 -> error). Negative = line to the LEFT = steer LEFT, positive = steer RIGHT.
//     IR2 IR3 IR4
//      1   0   0   -1.0     line under IR2 only
//      1   1   0   -0.5
//      0   1   0    0.0     line centred under IR3      <-- the aim
//      0   1   1   +0.5
//      0   0   1   +1.0     line under IR4 only
//      1   0   1    -      ambiguous: keep the previous error
//      1   1   1    -      thick box (intersection): see section 6
//      0   0   0    -      line lost: see section 8
// steer = PD_KP*error + PD_KD*(error change over DERIVATIVE_WINDOW_MS);  left wheel = speed + steer,  right wheel = speed - steer.
const int BASE_SPEED = 150;               // cruise PWM when the line is centred
const int MIN_FOLLOW_SPEED = 100;         // slowest cruise PWM while steering hard
const float CORNER_SLOWDOWN = 40.0f;      // PWM removed from the cruise speed per unit of |error|
const float PD_KP = 55.0f;                // PWM of steering per unit of error
const float PD_KD = 60.0f;                // PWM of steering per unit of error CHANGE over the derivative window (damping)
const uint32_t DERIVATIVE_WINDOW_MS = 24;
const int MAX_STEER_PWM = 130;            // clamp on the PD correction
const int PD_MIN_WHEEL_PWM = 0;           // slowest a wheel may go during ordinary following (0 = it may stop, never reverse)

// ================================================================================================================================================
// 5. DIRECTIONAL MARKERS (IR1 = LEFT box, IR5 = RIGHT box)
// ================================================================================================================================================
// One physical box = one event. A side counts a box when its sensor was black for MARKER_CONFIRM_MS..MARKER_MAX_MS without a break and
// then read white for MARKER_RELEASE_MS (the "release"). The event is committed at the END of the box, so a line that runs INTO an
// intersection (the leftmost / rightmost sensor can see it just before the box, e.g. at the top of a diagonal) stays black into the thick
// box, is cut off by the suppression below, and is never counted. The same release is required before the first marker after start-up, and
// again after every period in which markers were suppressed. Longer than MARKER_MAX_MS = a line, not a box.
const uint32_t MARKER_CONFIRM_MS = 12;
const uint32_t MARKER_RELEASE_MS = 30;
const uint32_t MARKER_MAX_MS = 600;
// After one side's box ends, wait this long for the other side's box to end too. Both within the window = a double marker.
const uint32_t MARKER_PAIR_WINDOW_MS = 80;
// Markers are IGNORED (suppressed) while: any state other than LINE_FOLLOWING is active (intersection handling, straight test, turning,
// reacquisition, recovery search), or IR2+IR3+IR4 have been all black for MARKER_SUPPRESS_AFTER_MS (the thick box / a wide line must not look
// like a box beside the line). A shorter all-black blip only pauses the detector: when the robot veers towards a marker box its IR4 / IR2 can
// touch the box for a few ms, and that must not cancel the marker. Keep it well below INTERSECTION_CONFIRM_MS.
const uint32_t MARKER_SUPPRESS_AFTER_MS = 25;
//
// RULES (explicit, nothing is inferred):
//   Single-side marker  : pendingTurn = that side, AND lastValidMarkerDirection = that side. The latest marker overrides any earlier one.
//   Double marker       : pendingTurn = NONE. lastValidMarkerDirection is left UNCHANGED (a double marker is not a valid single-side marker),
//                         unless DOUBLE_MARKER_CLEARS_HISTORY is true, in which case history is cleared as well.
//   pendingTurn         : consumed when the intersection that used it COMPLETES (turn finished), never merely on entering the intersection.
//   lastValidMarkerDirection : never consumed; it is the fallback for an unmarked intersection with no straight path (section 9).
const bool DOUBLE_MARKER_CLEARS_HISTORY = false;

// ================================================================================================================================================
// 6. INTERSECTIONS (thick box) & THE STRAIGHT-PATH TEST
// ================================================================================================================================================
// A thick box = IR2, IR3, IR4 all black for INTERSECTION_CONFIRM_MS. Each box is processed once. Detection is re-armed only when the middle
// sensors have been NOT-all-black for INTERSECTION_REARM_CLEAR_MS (the box is behind us) and INTERSECTION_REARM_HOLDOFF_MS has passed since
// the previous intersection finished.
//
// CALIBRATE INTERSECTION_CONFIRM_MS: a sharp bend in the line can also turn IR2+IR3+IR4 black, but only for a moment, because the bar crosses
// a line only as wide as the line (about 20 mm) instead of a box (about 60 mm deep). In the simulator at BASE_SPEED a box gave an all-black run of
// 210-270 ms and the 45 and 90 degree bends 30-46 ms. Pick a value between your longest bend and your shortest box. The robot logs every
// all-black run ("THICK run ended after N ms") so you can read both numbers off the serial monitor while it drives round the arena.
// While a thick pattern is still unconfirmed the robot holds its heading (both wheels equal): it does not steer on a pattern that has no
// left / right information. A bend that is crossed that way is picked up again by the line-lost search.
const uint32_t INTERSECTION_CONFIRM_MS = 100;
const uint32_t INTERSECTION_REARM_CLEAR_MS = 60;
const uint32_t INTERSECTION_REARM_HOLDOFF_MS = 200;

// Speed used to move through / over an intersection (straight test and the advance before a pivot). Keep it constant: the timed distances
// below only mean something at a fixed speed.
const int INTERSECTION_PWM = 150;
// Before a marker-directed pivot the robot drives on for this long so that its WHEEL AXLE (the pivot centre) ends up over the middle of the box.
// Counted from the moment the box is CONFIRMED (so INTERSECTION_CONFIRM_MS of travel is already behind you). Roughly:
// (distance from axle to the IR array + half the box depth - distance travelled during the confirm time) / speed. CALIBRATE on the arena.
const int32_t INTERSECTION_ADVANCE_MS = 300;
// A pivot is limited to this much extra reposition travel in either direction, whatever the arithmetic says.
const int32_t MAX_REPOSITION_MS = 700;

// STRAIGHT-PATH TEST (used at an intersection with no pending marker). Two phases, entered at the moment the box is confirmed; the normal
// line-lost handler is NOT active during them.
//   Phase CLEAR: drive straight at INTERSECTION_PWM for at least STRAIGHT_CLEAR_MS AND until the middle sensors have been not-all-black for
//                THICK_CLEAR_MS (the sensor bar has crossed the box's far edge). A longer black area is handled by OVERSIZE_BLACK_MS (section 10).
//   Phase PROBE: keep driving for STRAIGHT_PROBE_MS. FOUND if a line pattern (IR2/IR3/IR4 = 100, 110, 010, 011 or 001, never 111 and never 000)
//                is stable for STRAIGHT_STABLE_MS. NOT FOUND only if that never happens during the whole probe window. A single missing reading
//                is never taken as proof that there is no straight path.
const uint32_t STRAIGHT_CLEAR_MS = 120;
const uint32_t THICK_CLEAR_MS = 15;
const uint32_t STRAIGHT_PROBE_MS = 160;
const uint32_t STRAIGHT_STABLE_MS = 25;

// ================================================================================================================================================
// 7. TURNING (no encoders, no gyro: timed pivot + sensor feedback)
// ================================================================================================================================================
// Sequence: REPOSITION (advance, or reverse after a failed straight test) -> BLIND pivot -> SEEK pivot -> LINE_REACQUISITION -> LINE_FOLLOWING.
// BLIND: pivot at TURN_PWM for TURN_BLIND_MS no matter what the sensors say, to swing the sensors off the box and off the line we arrived on
//        (and past a straight branch, if there is one, even when the robot arrives a few degrees crooked).
//        This duration is NOT an angle. It has to be long enough to clear the straight branch plus the arrival error (roughly 25-30 degrees
//        of pivot) and shorter than the time the pivot needs to reach the nearest outgoing line (45 degrees in this arena).
// SEEK : keep pivoting at TURN_SEEK_PWM until IR3 is on a line (IR2/IR3/IR4 = 010, 110 or 011) for TURN_CAPTURE_MS. The thick box (111) never
//        counts. With TURN_REQUIRE_CLEAR_GAP the sensors must also have seen "no line" (000) for TURN_CLEAR_MS since the pivot began, which
//        stops the robot grabbing the line it arrived on (or a straight branch) when the blind time is too short.
// If the pivot has taken TURN_SEEK_PATIENCE_MS without finding a line the robot does NOT stop: it hands over to the recovery search.
const int TURN_PWM = 150;
const uint32_t TURN_BLIND_MS = 130;
const int TURN_SEEK_PWM = 120;
const uint32_t TURN_CAPTURE_MS = 6;
const bool TURN_REQUIRE_CLEAR_GAP = true;
const uint32_t TURN_CLEAR_MS = 6;
const uint32_t TURN_SEEK_PATIENCE_MS = 3500;

// ================================================================================================================================================
// 8. LINE LOST & RECOVERY (never ends, never stops the motors)
// ================================================================================================================================================
// IR2+IR3+IR4 all white for LINE_LOST_CONFIRM_MS during ordinary following = line lost. The robot keeps its last steering meanwhile.
// Then, forever, in this order (every stage keeps both wheels driving, and every failed stage hands over to the next):
//   LOCAL_LINE_SEARCH : pivot sweeps about the heading, first toward lastLineErrorSide: LOCAL_SWEEP_MS, then 2x, alternating,
//                       LOCAL_SEARCH_SWEEPS sweeps in total.
//   WIDE_LINE_SEARCH  : alternating arcs (a pivot plus a little REVERSE drive, back toward where the line was last seen: a line is usually
//                       lost by overshooting a bend) with growing amplitude: WIDE_SWEEP_START_MS, x WIDE_GROWTH_PERCENT/100 each sweep,
//                       capped at WIDE_SWEEP_MAX_MS; WIDE_SEARCH_SWEEPS sweeps in total.
//   RETRACE_RECOVERY  : an APPROXIMATE retrace. Without encoders or a gyro the robot cannot know its path or guarantee to return along it. It
//                       reverses for RETRACE_REVERSE_MS, then makes RETRACE_SEARCH_SWEEPS wide sweeps, the first toward the last executed
//                       intersection turn (if the last decision was a turn) otherwise toward lastLineErrorSide.
//   ...then back to LOCAL_LINE_SEARCH with every duration scaled up by SEARCH_CYCLE_GROWTH_PERCENT per failed cycle (up to SEARCH_MAX_GROWTH_CYCLES
//   cycles), starting on the opposite side every second cycle, and so on indefinitely.
// A line is only believed (LINE_REACQUISITION) after a centred pattern is seen for SEARCH_CAPTURE_MS and then a line pattern stays stable for
// REACQUIRE_STABLE_MS while creeping forward at REACQUIRE_PWM. If it is lost again for REACQUIRE_DROP_MS the search simply resumes.
const uint32_t LINE_LOST_CONFIRM_MS = 40;
const Direction DEFAULT_SEARCH_SIDE = DIR_LEFT;   // first search side when no line error has ever been recorded

const int SEARCH_PIVOT_PWM = 130;
const uint32_t SEARCH_CAPTURE_MS = 6;
const uint32_t LOCAL_SWEEP_MS = 320;
const uint8_t LOCAL_SEARCH_SWEEPS = 3;
const uint32_t WIDE_SWEEP_START_MS = 400;
const uint32_t WIDE_GROWTH_PERCENT = 150;
const uint32_t WIDE_SWEEP_MAX_MS = 1400;
const uint8_t WIDE_SEARCH_SWEEPS = 3;
const int WIDE_BIAS_PWM = -30;                    // drive added to both wheels during a wide sweep: negative = reverse (see above)
const uint32_t RETRACE_REVERSE_MS = 350;
const int RETRACE_PWM = 120;
const uint8_t RETRACE_SEARCH_SWEEPS = 3;
const uint32_t SEARCH_CYCLE_GROWTH_PERCENT = 25;
const uint8_t SEARCH_MAX_GROWTH_CYCLES = 4;
// Losing the line again soon after finding it must not repeat the same small search for ever: line losses that come before the robot has
// followed a line for PROGRESS_FOLLOW_MS (or completed an intersection) count as one growing episode.
const uint32_t PROGRESS_FOLLOW_MS = 1500;

const int REACQUIRE_PWM = 110;
const uint32_t REACQUIRE_STABLE_MS = 30;
const uint32_t REACQUIRE_DROP_MS = 60;

// ================================================================================================================================================
// 9. ARENA ROUTE RULES
// ================================================================================================================================================
// Arena (see README.md for the diagram). The robot starts on the START line facing the top of the map; the first intersection is straight ahead.
// Decisions along the intended route, in order:
//     1 LEFT   2 RIGHT   3 RIGHT   4 RIGHT   5 RIGHT   6 LEFT   7 (no marker, NO straight path)   8 LEFT   9 (no marker, straight exists)
//    10 RIGHT 11 RIGHT  12 (no marker, straight exists) -> FINISH box.
// Three sections have no marker between two intersections:
//   * Decisions 9 and 12: a straight outgoing line exists, so "no pending marker" => the straight-path test finds it and the robot goes on.
//   * Decision 7: NO straight outgoing line exists, so the straight test fails and the robot turns the way of the LAST VALID SINGLE-SIDE MARKER
//     (the marker before decision 6, which was LEFT) => a LEFT turn.
// The rule is a program condition, not a colour or a picture:   (pendingTurn == NONE) AND (straight test failed)
//                                                                     => turn lastValidMarkerDirection, or ROUTE_FALLBACK_NO_HISTORY if none yet.
const Direction ROUTE_FALLBACK_NO_HISTORY = DIR_LEFT;

// ================================================================================================================================================
// 10. FINISH ZONE (not in the original specification — OFF by default)
// ================================================================================================================================================
// The finish is a black area far bigger than an intersection box. It is recognised as: IR2+IR3+IR4 continuously black for OVERSIZE_BLACK_MS
// while driving straight through an intersection (an ordinary box is crossed in well under half of that), and, if
// FINISH_REQUIRE_OUTER_SENSORS, IR1 and IR5 black as well.
// FINISH_STOP_ENABLED false (default, as specified: the robot never stops by itself): the black area is logged and treated as "no straight
//   path", so the robot keeps driving and searching.
// FINISH_STOP_ENABLED true: the robot stops its motors inside the finish zone (state FINISHED). It does not wait for BOOT: it resumes by
//   itself once the sensors have not seen all-black for FINISH_RESUME_MS (the robot was lifted or moved). A lifted robot reads all-black too.
#ifndef FINISH_STOP_ENABLED
#define FINISH_STOP_ENABLED false
#endif
const uint32_t OVERSIZE_BLACK_MS = 600;
const bool FINISH_REQUIRE_OUTER_SENSORS = true;
const uint32_t FINISH_RESUME_MS = 1500;

// ================================================================================================================================================
// 11. START / STOP / RESET BEHAVIOUR
// ================================================================================================================================================
// AUTO_START_ON_BOOT true (competition): after power-up the robot starts by itself after AUTO_START_DELAY_MS, no button needed.
// AUTO_START_ON_BOOT false (bench testing): it waits, stopped, for the BOOT button (or serial 'g').
// BOOT toggles stop / start at any time (also serial 'x' / 'g'); a manual stop only stops the motors, the control loop keeps running.
#ifndef AUTO_START_ON_BOOT
#define AUTO_START_ON_BOOT true
#endif
const uint32_t AUTO_START_DELAY_MS = 3000;        // after power-up: time to take your hand away
const uint32_t MANUAL_START_DELAY_MS = 1500;      // after pressing BOOT / serial 'g'
const uint32_t AUTO_RESTART_DELAY_MS = 150;       // after an UNEXPECTED reset (brownout, watchdog, panic) while a run was active
// After an unexpected reset the robot restores its marker memory from RTC memory and, if a run was active, resumes by itself without BOOT.
// (Software cannot survive a real power loss or a hardware fault: this only covers a reset the chip itself recovers from.)
const bool RESUME_RUN_AFTER_UNEXPECTED_RESET = true;
const bool RESET_NAV_MEMORY_ON_MANUAL_START = true;   // a fresh BOOT start forgets the old markers
const uint32_t BUTTON_DEBOUNCE_MS = 50;

// ================================================================================================================================================
// 12. DIAGNOSTICS
// ================================================================================================================================================
const uint32_t SERIAL_BAUD = 115200;
#ifndef LOG_PERIODIC_STATUS
#define LOG_PERIODIC_STATUS true
#endif
const uint32_t STATUS_LOG_PERIOD_MS = 500;
// Log how long every all-black run of IR2+IR3+IR4 lasted (boxes, bends, the finish area). This is the data for calibrating INTERSECTION_CONFIRM_MS.
#ifndef LOG_THICK_RUNS
#define LOG_THICK_RUNS true
#endif
const uint32_t THICK_RUN_LOG_MIN_MS = 8;
const uint32_t SENSOR_MONITOR_PERIOD_MS = 100;
const int MOVEMENT_TEST_PWM = 150;
const uint32_t MOVEMENT_TEST_STEP_MS = 1000;
const uint32_t MOVEMENT_TEST_PAUSE_MS = 500;

#endif
