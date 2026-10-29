/*
  ==================================================================================================================================================
   MECC 2026 — LINE FOLLOWER ROBOT (ESP32 Dev Module, Arduino-ESP32 core 3.x)
  ==================================================================================================================================================
  IR3 (centre sensor) follows the black line under PD control. IR1 / IR5 read the small directional boxes beside the line, a thick box
  (IR2+IR3+IR4 all black) is an intersection, and the robot turns at it as the latest marker says. With no marker it first tries to go straight;
  if there is no straight line it uses the last marker it saw. If the line is lost it searches, forever, and never stops by itself.

  File map          config.h       every calibration value, with the arena rules and the marker rules written out
                    sensors.*      logical sensor bits (IR1..IR5 = bit 0..4), marker detector     (workshop functions kept)
                    motor.*        differential drive, PWM, left/right assignment, inversion      (workshop functions kept)
                    navigation.*   marker memory, fallback rule (pure logic, tested on a PC)
                    robot.*        the state machine
                    diagnostics.*  serial log, reset reason, memory that survives an unexpected reset

  Serial monitor at 115200 baud:  g = start   x = stop motors   w = wheel test   m = sensor monitor   ? = help.   BOOT button = start / stop.
  Read README.md before the first run. Bench testing: set AUTO_START_ON_BOOT to false in config.h, or put the robot on a stand.
*/

#include "robot.h"

void setup() {
  robotSetup();   // pins, serial, reset-reason report, start policy
}

void loop() {
  robotLoop();    // non-blocking: runs one control tick when it is due, then returns
}
