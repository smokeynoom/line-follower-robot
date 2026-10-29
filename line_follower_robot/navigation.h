#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "common.h"
#include "sensors.h"

/*
  ==================================================================================================================================================
   NAVIGATION — what to do at an intersection. Decisions only: LEFT, RIGHT or STRAIGHT-first. How the wheels carry them out is robot.cpp / motor.cpp.
  ==================================================================================================================================================

   ARENA. The robot starts on the START line heading NORTH (up the page) with IR3 on the line; the first intersection is straight ahead.
   Intersections are named by the letters used in README.md (I = the cross on the START line). A marker's side is as seen travelling forward.

        #   box   arrive heading   marker before it    action
        1   I     north            LEFT                turn LEFT  (90 degrees)
        2   H     west             RIGHT               turn RIGHT (about 45 degrees)
        3   G     north-west       RIGHT               turn RIGHT (about 45 degrees)
        4   F     north            RIGHT               turn RIGHT (90 degrees)
        5   E     east             RIGHT               turn RIGHT (about 45 degrees, onto a diagonal)
        6   J     south            LEFT                turn LEFT  (90 degrees)
        7   K     east             none                straight does NOT exist -> fallback: last valid marker (LEFT from #6) -> turn LEFT
        8   L     north            LEFT                turn LEFT  (about 45 degrees, onto the long diagonal)
        9   D     west             none                straight exists -> go straight
       10   C     west             RIGHT               turn RIGHT (about 135 degrees, onto a diagonal)
       11   A     north-east       RIGHT               turn RIGHT (about 45 degrees)
       12   B     east             none                straight exists -> go straight into the FINISH box

   The three marker-less sections are #7, #9 and #12. The rule that covers all of them is a program condition, not a colour:
        pendingTurn == NONE   AND   the straight-path test found no outgoing line
            ->  turn toward lastValidMarkerDirection   (or ROUTE_FALLBACK_NO_HISTORY = LEFT when no marker has been seen yet)
   At #9 and #12 the straight-path test succeeds, so the fallback is never reached; at #7 it fails, which gives the required LEFT turn.

   The five sensors cannot see the whole junction layout. The straight-path test only tells whether a line appears straight ahead after the
   thick box has been crossed; everything else comes from the markers and the rule above.
*/

// All the memory the navigation rules need. Kept as a plain struct so it can be tested, saved across a reset, and printed.
struct NavMemory {
  Direction pendingTurn = DIR_NONE;               // direction instructed by the latest marker and not yet used by an intersection
  Direction lastValidMarkerDirection = DIR_NONE;  // direction of the latest valid single-side marker (history; survives use)
  Direction lastLineErrorSide = DIR_NONE;         // side the line was last seen on while following it (steers recovery searches)
  Direction lastTurnExecuted = DIR_NONE;          // turn made at the latest completed intersection, NONE if it went straight
  uint16_t markerCount = 0;                       // valid single-side markers seen (diagnostics)
  uint16_t intersectionsCompleted = 0;            // diagnostics
};

void navReset(NavMemory& m);

// A completed marker event updates pendingTurn / lastValidMarkerDirection. Rules are documented in config.h (section 5).
void navOnMarkerEvent(NavMemory& m, MarkerEvent e);

// Direction to take at an intersection that has no pending marker AND no straight outgoing line.
Direction navFallbackTurn(const NavMemory& m);

// The intersection that used pendingTurn is complete: consume it and remember what was done (DIR_NONE = went straight).
void navCompleteIntersection(NavMemory& m, Direction turnExecuted);

// Remember which side the line was on whenever it is off-centre. error < 0: line LEFT of the robot; error > 0: RIGHT.
void navNoteLineError(NavMemory& m, float error);

#endif
