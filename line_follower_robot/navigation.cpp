#include "navigation.h"
#include "config.h"

void navReset(NavMemory& m) {
  m = NavMemory();
}

void navOnMarkerEvent(NavMemory& m, MarkerEvent e) {
  switch (e) {
    case MARKER_LEFT:
      m.pendingTurn = DIR_LEFT;                // the most recent single-side marker overrides any earlier pending direction
      m.lastValidMarkerDirection = DIR_LEFT;
      m.markerCount++;
      break;
    case MARKER_RIGHT:
      m.pendingTurn = DIR_RIGHT;
      m.lastValidMarkerDirection = DIR_RIGHT;
      m.markerCount++;
      break;
    case MARKER_BOTH:
      m.pendingTurn = DIR_NONE;                // the two boxes cancel: no bias, and never an arbitrary choice
      if (DOUBLE_MARKER_CLEARS_HISTORY) m.lastValidMarkerDirection = DIR_NONE;
      break;
    default:
      break;
  }
}

Direction navFallbackTurn(const NavMemory& m) {
  return m.lastValidMarkerDirection != DIR_NONE ? m.lastValidMarkerDirection : ROUTE_FALLBACK_NO_HISTORY;
}

void navCompleteIntersection(NavMemory& m, Direction turnExecuted) {
  m.pendingTurn = DIR_NONE;
  m.lastTurnExecuted = turnExecuted;
  m.intersectionsCompleted++;
}

void navNoteLineError(NavMemory& m, float error) {
  if (error < 0) m.lastLineErrorSide = DIR_LEFT;
  else if (error > 0) m.lastLineErrorSide = DIR_RIGHT;
}
