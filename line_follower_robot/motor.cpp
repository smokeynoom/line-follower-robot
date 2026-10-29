#include "motor.h"
#include "common.h"
#include "config.h"

namespace {

// One driver channel: two direction pins and one PWM enable pin.
struct Channel {
  int in1 = 0, in2 = 0, enable = 0;
  bool invert = false;
};

enum { WHEEL_LEFT = 0, WHEEL_RIGHT = 1 };

Channel channelA;                 // Motor A (IN1, IN2, ENA)
Channel channelB;                 // Motor B (IN3, IN4, ENB)
Channel* wheel[2] = {&channelA, &channelB};
int targetPwm[2] = {0, 0};        // what the program asked for
int appliedPwm[2] = {0, 0};       // what the slew limiter has reached so far
int writtenPwm[2] = {0, 0};
bool written[2] = {false, false};

// Turns a signed command into direction pins + PWM duty for one wheel.
void writeWheel(int index, int command) {
  const Channel& c = *wheel[index];
  float scaled = command * (index == WHEEL_LEFT ? LEFT_PWM_SCALE : RIGHT_PWM_SCALE);
  int signedPwm = clampInt((int)(scaled >= 0 ? scaled + 0.5f : scaled - 0.5f), -MAX_PWM, MAX_PWM);
  int duty = absInt(signedPwm);

  if (duty == 0) {                                  // off: both direction pins low, so even a left-over ENA jumper cannot drive it
    digitalWrite(c.in1, LOW);
    digitalWrite(c.in2, LOW);
    ledcWrite(c.enable, 0);
    return;
  }
  if (duty < MIN_MOVE_PWM) duty = MIN_MOVE_PWM;     // dead zone: below this the wheel would only hum

  bool forward = signedPwm > 0;
  if (c.invert) forward = !forward;
  digitalWrite(c.in1, forward ? HIGH : LOW);
  digitalWrite(c.in2, forward ? LOW : HIGH);
  ledcWrite(c.enable, duty);
}

void writeIfChanged(int index) {
  if (written[index] && writtenPwm[index] == appliedPwm[index]) return;
  writeWheel(index, appliedPwm[index]);
  writtenPwm[index] = appliedPwm[index];
  written[index] = true;
}

void setWheelNow(int index, int command) {
  targetPwm[index] = command;
  appliedPwm[index] = command;
  writeIfChanged(index);
}

}  // namespace

void motorInit(int in1, int in2, int in3, int in4, int ena, int enb) {
  channelA.in1 = in1; channelA.in2 = in2; channelA.enable = ena; channelA.invert = MOTOR_A_INVERT;
  channelB.in1 = in3; channelB.in2 = in4; channelB.enable = enb; channelB.invert = MOTOR_B_INVERT;
  // wheel[WHEEL_LEFT] / wheel[WHEEL_RIGHT] point at whichever channel is physically on that side
  wheel[WHEEL_LEFT] = LEFT_WHEEL_IS_MOTOR_A ? &channelA : &channelB;
  wheel[WHEEL_RIGHT] = LEFT_WHEEL_IS_MOTOR_A ? &channelB : &channelA;

  const Channel* all[2] = {&channelA, &channelB};
  for (int i = 0; i < 2; i++) {
    pinMode(all[i]->in1, OUTPUT);
    pinMode(all[i]->in2, OUTPUT);
    pinMode(all[i]->enable, OUTPUT);
    digitalWrite(all[i]->in1, LOW);
    digitalWrite(all[i]->in2, LOW);
    ledcAttach(all[i]->enable, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);   // ESP32 core 3.x PWM: one call per pin
    ledcWrite(all[i]->enable, 0);
  }
  for (int i = 0; i < 2; i++) {
    targetPwm[i] = appliedPwm[i] = writtenPwm[i] = 0;
    written[i] = false;
  }
}

void setMotorSpeeds(int leftPwm, int rightPwm) {
  targetPwm[WHEEL_LEFT] = clampInt(leftPwm, -MAX_PWM, MAX_PWM);
  targetPwm[WHEEL_RIGHT] = clampInt(rightPwm, -MAX_PWM, MAX_PWM);
}

void motorUpdate(uint32_t dtMs) {
  if (dtMs < 1) dtMs = 1;
  if (dtMs > 20) dtMs = 20;
  const int maxStep = (int)(MOTOR_SLEW_PWM_PER_MS * dtMs);
  for (int i = 0; i < 2; i++) {
    const int delta = targetPwm[i] - appliedPwm[i];
    appliedPwm[i] += clampInt(delta, -maxStep, maxStep);
    writeIfChanged(i);
  }
}

void motorStop() {
  setWheelNow(WHEEL_LEFT, 0);
  setWheelNow(WHEEL_RIGHT, 0);
}

void moveForward(int speed) { setMotorSpeeds(speed, speed); }
void moveBackward(int speed) { setMotorSpeeds(-speed, -speed); }
void pivotLeft(int speed) { setMotorSpeeds(-speed, speed); }
void pivotRight(int speed) { setMotorSpeeds(speed, -speed); }
void curveLeft(int speed, int delta) { setMotorSpeeds(speed - delta, speed); }
void curveRight(int speed, int delta) { setMotorSpeeds(speed, speed - delta); }

int motorLeftApplied() { return appliedPwm[WHEEL_LEFT]; }
int motorRightApplied() { return appliedPwm[WHEEL_RIGHT]; }

// ---- Workshop API: one wheel at a time, immediately ------------------------------------------------------------------------------------------
void leftForward(uint8_t speed) { setWheelNow(WHEEL_LEFT, speed); }
void leftReverse(uint8_t speed) { setWheelNow(WHEEL_LEFT, -(int)speed); }
void rightForward(uint8_t speed) { setWheelNow(WHEEL_RIGHT, speed); }
void rightReverse(uint8_t speed) { setWheelNow(WHEEL_RIGHT, -(int)speed); }
void stopAll() { motorStop(); }
