#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "navigation.h"

/*
  The robot's state machine. Everything is non-blocking and millis()-based: robotLoop() returns immediately when no control tick is due, and a tick
  only reads sensors, updates one state and sets the wheel targets.

  STATE_STOPPED is the only state in which the motors stay off, and it is entered only by an intentional command (BOOT, serial 'x', bench mode
  at power-up). Nothing the robot sees on the track — a lost line, a strange sensor pattern, a failed turn, a failed search — ever leads there.
*/
enum RobotState : uint8_t {
  STATE_STOPPED = 0,             // intentional stop (BOOT / serial 'x', or AUTO_START_ON_BOOT false). The loop keeps running.
  STATE_START_DELAY,             // short countdown before a run; leaves by itself
  STATE_LINE_FOLLOWING,          // PD line following; watches for a thick box and for line loss
  STATE_INTERSECTION_DETECTED,   // thick box confirmed once: latch the marker direction and decide
  STATE_STRAIGHT_PATH_CHECK,     // no marker: drive over the box and look for an outgoing straight line
  STATE_TURNING_LEFT,            // reposition + timed pivot + sensor-guided pivot
  STATE_TURNING_RIGHT,
  STATE_LINE_REACQUISITION,      // a candidate line was found: creep forward until it is stable
  STATE_LOCAL_LINE_SEARCH,       // recovery stage 1
  STATE_WIDE_LINE_SEARCH,        // recovery stage 2
  STATE_RETRACE_RECOVERY,        // recovery stage 3: approximate reverse-and-search
  STATE_FINISHED,                // only when FINISH_STOP_ENABLED
  STATE_MOVEMENT_TEST,           // manual wheel test from serial 'w' (robot must be lifted)
  STATE_COUNT
};

void robotSetup();               // call from setup()
void robotLoop();                // call from loop(), as often as possible
void robotRequestStart();        // same as the BOOT button when stopped
void robotRequestStop();         // intentional motor stop; the control loop keeps running and a later start works normally

RobotState robotState();
const char* robotStateName(RobotState s);
const NavMemory& robotNavMemory();

#endif
