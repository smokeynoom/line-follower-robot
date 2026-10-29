// Host-side simulator: the REAL firmware (line_follower_robot/*.cpp) drives a simulated differential-drive robot over the MECC arena.
//
// The simulator has its OWN idea of how the robot is wired (taken from the handout and the project wiring table, not from config.h), so a wrong
// pin / polarity / left-right / inversion setting in the firmware shows up as the simulated robot driving wrongly:
//   * sensors: IR1..IR5 are on GPIO 23,22,21,19,18 from left to right, and read HIGH over black (handout: active-low on reflection, INPUT_PULLUP)
//   * left wheel  = Motor A: forward when IN1=HIGH, IN2=LOW, speed from ENA (GPIO14) duty
//   * right wheel = Motor B: forward when IN3=LOW,  IN4=HIGH (mounted the other way round), speed from ENB (GPIO33) duty
//
// What it does NOT model: wheel slip, battery sag, sensor noise or spot size, uneven lighting, a real floor. Its numbers (speeds, sensor
// geometry, line and box sizes) are plausible guesses, so a pass proves the LOGIC works, not that the tuning suits the real robot.
#ifndef SIM_H
#define SIM_H

#include <math.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "Arduino.h"
#include "robot.h"

namespace sim {

const double PI = 3.14159265358979323846;

// ---- physical truth about the robot's wiring (independent of config.h) -------------------------------------------------------------------------
const uint8_t PHYS_SENSOR_GPIO_LEFT_TO_RIGHT[5] = {23, 22, 21, 19, 18};
const int PHYS_BLACK_LEVEL = HIGH;
const int PHYS_L_IN1 = 27, PHYS_L_IN2 = 26, PHYS_L_EN = 14;   // left wheel: forward = IN1 HIGH, IN2 LOW
const int PHYS_R_IN3 = 13, PHYS_R_IN4 = 25, PHYS_R_EN = 33;   // right wheel: forward = IN3 LOW, IN4 HIGH

struct V {
  double x, y;
};
inline V operator+(V a, V b) { return {a.x + b.x, a.y + b.y}; }
inline V operator-(V a, V b) { return {a.x - b.x, a.y - b.y}; }
inline V operator*(V a, double k) { return {a.x * k, a.y * k}; }
inline double dot(V a, V b) { return a.x * b.x + a.y * b.y; }
inline double len(V a) { return sqrt(dot(a, a)); }

struct Seg { V a, b; };
struct Rect {
  double x0, y0, x1, y1;
  bool contains(V p) const { return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1; }
};

inline double distPointSeg(V p, V a, V b) {
  const V ab = b - a;
  const double l2 = dot(ab, ab);
  double t = l2 > 0 ? dot(p - a, ab) / l2 : 0;
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  return len(p - (a + ab * t));
}

// ---- the arena, from the competition diagram. Coordinates below are PIXELS of that diagram (y down); 1 px = 20/12 mm so the line is 20 mm wide.
const double MM_PER_PX = 20.0 / 12.0;
const double IMG_H = 1125;
inline V P(double px, double py) { return {px * MM_PER_PX, (IMG_H - py) * MM_PER_PX}; }

struct Leg {                // one stretch of the intended route between two waypoints
  V from, to;
  int marker;               // 0 none, -1 a box on the LEFT of the direction of travel, +1 on the RIGHT
  double at;                // where along the leg the box sits (0..1)
};

// A square that may be turned: the marker boxes beside a diagonal stretch are diamonds (rotated to follow the track), as in the diagram.
struct TurnedBox {
  V centre;
  V dir;          // unit vector along the track
  double half;
  bool contains(V p) const {
    const V d = p - centre;
    const V n = {-dir.y, dir.x};
    return fabs(dot(d, dir)) <= half && fabs(dot(d, n)) <= half;
  }
};

struct World {
  double lineWidthMm = 20;
  std::vector<Seg> lines;
  std::vector<Rect> boxes;          // thick boxes at intersections
  std::vector<TurnedBox> markers;   // the small directional boxes
  Rect finish{};
  std::vector<Leg> route;

  bool black(V p) const {
    for (const Rect& r : boxes) if (r.contains(p)) return true;
    for (const TurnedBox& m : markers) if (m.contains(p)) return true;
    if (finish.contains(p)) return true;
    for (const Seg& s : lines) if (distPointSeg(p, s.a, s.b) <= lineWidthMm / 2) return true;
    return false;
  }
};

inline void addLine(World& w, double x0, double y0, double x1, double y1) { w.lines.push_back({P(x0, y0), P(x1, y1)}); }
inline void addBoxAt(World& w, double px, double py, double sizePx) {
  const V c = P(px, py);
  const double h = sizePx * MM_PER_PX / 2;
  w.boxes.push_back({c.x - h, c.y - h, c.x + h, c.y + h});
}

// markerOffsetMm: how far from the line the marker boxes sit. The real boxes sit where the outer sensors can see them; the simulator
// puts them exactly on the outer sensors' path so the test is about the logic, not about the placement.
inline World buildArena(double markerOffsetMm, double markerSizeMm = 35) {
  World w;
  // ---- lines -----------------------------------------------------------------------------------------------------------------------------
  addLine(w, 68, 80, 870, 80);      addLine(w, 870, 80, 960, 80);                         // top row, and the stub into the finish box
  addLine(w, 68, 80, 68, 308);      addLine(w, 68, 308, 748, 308);                        // left side and the row below
  addLine(w, 748, 308, 748, 400);   addLine(w, 748, 400, 1058, 688);                      // bend and the long diagonal down to L
  addLine(w, 247, 308, 492, 80);                                                          // diagonal C-A
  addLine(w, 600, 308, 600, 535);   addLine(w, 210, 535, 600, 535);                       // D-E and F-E
  addLine(w, 210, 535, 85, 415);                                                          // dead-end stub at F
  addLine(w, 210, 535, 210, 812);                                                         // F-G
  addLine(w, 210, 812, 333, 928);                                                         // diagonal G-H
  addLine(w, 210, 812, 68, 812);    addLine(w, 68, 812, 68, 1060);                        // bottom-left loop
  addLine(w, 68, 1060, 333, 1060);  addLine(w, 333, 1060, 333, 928);
  addLine(w, 80, 1055, 195, 945);                                                         // stub inside the loop
  addLine(w, 333, 928, 1060, 928);                                                        // main horizontal H-I-J-K
  addLine(w, 560, 1110, 560, 798);                                                        // start line, crossing the horizontal at I
  addLine(w, 600, 535, 820, 745);   addLine(w, 820, 745, 818, 928);                       // diagonal E down to J
  addLine(w, 818, 928, 818, 1055);  addLine(w, 818, 1055, 1065, 1055); addLine(w, 1065, 1055, 1060, 928);   // loop under J-K
  addLine(w, 1060, 928, 1058, 688);                                                       // K-L
  addLine(w, 1058, 688, 1055, 388); addLine(w, 1055, 388, 870, 388); addLine(w, 870, 388, 870, 80);       // L up and round to B
  // ---- thick boxes at the intersections (38 px = 63 mm) ------------------------------------------------------------------------------------
  const double nodes[][2] = {{492, 80}, {870, 80}, {247, 308}, {600, 308}, {600, 535}, {210, 535}, {210, 812},
                             {333, 928}, {560, 928}, {818, 928}, {1060, 928}, {1058, 688}, {80, 1055}};
  for (const auto& n : nodes) addBoxAt(w, n[0], n[1], 38);
  w.finish = {958 * MM_PER_PX, (IMG_H - 160) * MM_PER_PX, 1125 * MM_PER_PX, IMG_H * MM_PER_PX};

  // ---- the intended route, with the marker (if any) between each pair of intersections ------------------------------------------------------
  struct L { double ax, ay, bx, by; int marker; double at; };
  const L legs[] = {
    {560, 1100, 560, 928, -1, 0.45},   // START -> I : LEFT box
    {560, 928, 333, 928, +1, 0.50},    // I -> H     : RIGHT
    {333, 928, 210, 812, +1, 0.50},    // H -> G     : RIGHT
    {210, 812, 210, 535, +1, 0.50},    // G -> F     : RIGHT
    {210, 535, 600, 535, +1, 0.50},    // F -> E     : RIGHT
    {600, 535, 820, 745, -1, 0.60},    // E -> bend  : LEFT (before J)
    {820, 745, 818, 928, 0, 0},        // bend -> J
    {818, 928, 1060, 928, 0, 0},       // J -> K     : NO marker, no straight path at K
    {1060, 928, 1058, 688, -1, 0.50},  // K -> L     : LEFT
    {1058, 688, 748, 400, 0, 0},       // L -> bend  : NO marker, straight exists at D
    {748, 400, 748, 308, 0, 0},
    {748, 308, 600, 308, 0, 0},        // -> D
    {600, 308, 247, 308, +1, 0.50},    // D -> C     : RIGHT
    {247, 308, 492, 80, +1, 0.50},     // C -> A     : RIGHT
    {492, 80, 870, 80, 0, 0},          // A -> B     : NO marker, straight exists at B
    {870, 80, 960, 80, 0, 0},          // B -> FINISH
  };
  for (const L& l : legs) {
    const V a = P(l.ax, l.ay), b = P(l.bx, l.by);
    w.route.push_back({a, b, l.marker, l.at});
    if (l.marker == 0) continue;
    const V d = b - a;
    const V dir = d * (1.0 / len(d));
    const V left = {-dir.y, dir.x};
    const V c = a + d * l.at + left * (-l.marker * markerOffsetMm);   // marker -1 (LEFT) => +left vector
    w.markers.push_back({c, dir, markerSizeMm / 2});     // aligned with the direction of travel, so a diamond beside a diagonal stretch
  }
  return w;
}

// ---- the simulated robot -------------------------------------------------------------------------------------------------------------------------
struct RobotModel {
  double trackMm = 130;           // distance between the wheels
  double sensorForwardMm = 70;    // the IR bar is this far ahead of the wheel axle
  double midPitchMm = 16;         // IR2 / IR4 are this far either side of IR3
  double outerOffsetMm = 48;      // IR1 / IR5 are this far either side of IR3
  double vMaxMmS = 550;           // wheel speed at full duty
  double deadDuty = 55;           // duty below which the wheel does not turn
  double tauS = 0.05;             // motor response time constant
};

class Sim {
 public:
  static const uint32_t STEP_US = 500;

  World world;
  RobotModel model;
  double x = 0, y = 0, th = 0;           // pose of the wheel-axle centre; th = heading, radians counter-clockwise from +x (north = +y)
  double vl = 0, vr = 0;                 // actual wheel speeds, mm/s, forward positive
  int scripted = -1;                     // >= 0: use these logical sensor bits instead of the geometry (bit0 = IR1 leftmost)
  bool invertSensors = false;            // simulate a robot whose sensors read the opposite way
  uint64_t zeroRunUs = 0;                // how long both wheels have been commanded off in a row while the robot was in a running state
  uint32_t maxZeroRunMs = 0;             // the longest such run
  uint64_t steps = 0;

  explicit Sim(double markerOffsetMm = 48) : world(buildArena(markerOffsetMm)) {}

  void placeAtStart() {
    const V s = P(560, 1100);
    x = s.x; y = s.y;                               // wheel-axle centre at the foot of the start line
    th = PI / 2;                                  // heading north, up the page
    vl = vr = 0;
  }
  void place(V p, double headingRad) { x = p.x; y = p.y; th = headingRad; vl = vr = 0; }

  V sensorPos(int i) const {   // i = 0..4 for IR1..IR5 (left to right)
    const double lateral[5] = {+model.outerOffsetMm, +model.midPitchMm, 0, -model.midPitchMm, -model.outerOffsetMm};
    const V f = {cos(th), sin(th)};
    const V l = {-sin(th), cos(th)};
    return V{x, y} + f * model.sensorForwardMm + l * lateral[i];
  }
  uint8_t geometryBits() const {
    uint8_t bits = 0;
    for (int i = 0; i < 5; i++) if (world.black(sensorPos(i))) bits |= (uint8_t)(1 << i);
    return bits;
  }
  bool irThreeInFinish() const { return world.finish.contains(sensorPos(2)); }

  double wheelTarget(bool left) const {
    int in1, in2, en;
    int forwardIn1Level;
    if (left) { in1 = PHYS_L_IN1; in2 = PHYS_L_IN2; en = PHYS_L_EN; forwardIn1Level = HIGH; }
    else      { in1 = PHYS_R_IN3; in2 = PHYS_R_IN4; en = PHYS_R_EN; forwardIn1Level = LOW; }
    const int a = mock::pinLevel[in1], b = mock::pinLevel[in2];
    int dir = 0;
    if (a == forwardIn1Level && b != forwardIn1Level) dir = +1;
    else if (a != forwardIn1Level && b == forwardIn1Level) dir = -1;
    const double duty = (double)mock::pwmDuty[en];
    if (dir == 0 || duty <= model.deadDuty) return 0;
    return dir * model.vMaxMmS * (duty - model.deadDuty) / (255.0 - model.deadDuty);
  }

  bool bothWheelsOff() const { return wheelTarget(true) == 0 && wheelTarget(false) == 0; }

  void setSensorPins(uint8_t bits) {
    for (int i = 0; i < 5; i++) {
      const bool black = (bits >> i) & 1;
      const bool level = (black != invertSensors) ? (PHYS_BLACK_LEVEL == HIGH) : (PHYS_BLACK_LEVEL != HIGH);
      mock::pinLevel[PHYS_SENSOR_GPIO_LEFT_TO_RIGHT[i]] = level ? HIGH : LOW;
    }
  }

  // One 0.5 ms step: sensors -> firmware -> motors -> motion.
  void step() {
    mock::advanceUs(STEP_US);
    setSensorPins(scripted >= 0 ? (uint8_t)scripted : geometryBits());
    robotLoop();

    const double dt = STEP_US * 1e-6;
    const double tl = wheelTarget(true), tr = wheelTarget(false);
    vl += (tl - vl) * (dt / model.tauS);
    vr += (tr - vr) * (dt / model.tauS);
    const double v = (vl + vr) / 2, w = (vr - vl) / model.trackMm;
    th += w * dt;
    x += v * cos(th) * dt;
    y += v * sin(th) * dt;
    steps++;

    const RobotState s = robotState();
    const bool running = s != STATE_STOPPED && s != STATE_FINISHED && s != STATE_MOVEMENT_TEST && s != STATE_START_DELAY;
    if (running && bothWheelsOff()) {
      zeroRunUs += STEP_US;
      const uint32_t ms = (uint32_t)(zeroRunUs / 1000);
      if (ms > maxZeroRunMs) maxZeroRunMs = ms;
    } else {
      zeroRunUs = 0;
    }
  }

  void run(double seconds) {
    const uint64_t n = (uint64_t)(seconds * 1e6 / STEP_US);
    for (uint64_t i = 0; i < n; i++) step();
  }
  template <class Pred>
  bool runUntil(Pred done, double maxSeconds) {
    const uint64_t n = (uint64_t)(maxSeconds * 1e6 / STEP_US);
    for (uint64_t i = 0; i < n; i++) {
      step();
      if (done()) return true;
    }
    return false;
  }
  double nowSeconds() const { return mock::timeUs / 1e6; }
};

// ---- helpers to read the firmware's log ----------------------------------------------------------------------------------------------------------
inline std::vector<std::string> logLinesContaining(const std::string& text, const std::string& needle) {
  std::vector<std::string> out;
  size_t pos = 0;
  while (pos < text.size()) {
    size_t eol = text.find('\n', pos);
    if (eol == std::string::npos) eol = text.size();
    const std::string line = text.substr(pos, eol - pos);
    if (line.find(needle) != std::string::npos) out.push_back(line);
    pos = eol + 1;
  }
  return out;
}

}  // namespace sim

#endif
