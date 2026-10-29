#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void motorInit(int in1, int in2, int in3, int in4, int ena, int enb);

// ================================================================================================================================================
// Differential-drive API. Wheel commands are SIGNED PWM: +forward, -reverse, 0 = off, range -MAX_PWM..+MAX_PWM.
//   straight                 both wheels forward at about the same speed
//   gradual curve (left)     both wheels forward, the LEFT wheel slower than the right
//   gradual curve (right)    both wheels forward, the RIGHT wheel slower than the left
//   pivot left               left wheel reverses, right wheel drives forward  (turns on the spot)
//   pivot right              left wheel drives forward, right wheel reverses
// Left / right are the ROBOT's left and right wheel (which driver channel that is, and which way it spins, is configured in config.h).
// A differential-drive robot cannot move sideways: "left" and "right" always mean a curve or a pivot.
// ================================================================================================================================================
void setMotorSpeeds(int leftPwm, int rightPwm);   // sets the target speeds; motorUpdate() ramps toward them
void motorUpdate(uint32_t dtMs);                  // call every control tick: slew limiting + the actual pin / PWM writes
void motorStop();                                 // intentional stop: both wheels off at once (coast). setMotorSpeeds() makes them run again.

void moveForward(int speed);
void moveBackward(int speed);
void pivotLeft(int speed);
void pivotRight(int speed);
void curveLeft(int speed, int delta);             // left wheel slower than the right by delta
void curveRight(int speed, int delta);            // right wheel slower than the left by delta

// Current applied command per wheel (after the slew limiter, before scaling / dead zone), for diagnostics and tests.
int motorLeftApplied();
int motorRightApplied();

// ================================================================================================================================================
// Workshop API (Task 2 skeleton). Each call drives one wheel immediately, without the slew limiter.
// ================================================================================================================================================
void leftForward(uint8_t speed);
void leftReverse(uint8_t speed);
void rightForward(uint8_t speed);
void rightReverse(uint8_t speed);
void stopAll();

#endif
