
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "state.h"
#include "home.h"
#include "motor.h"
#include "stop.h"
#include "move.h"
#include "clock.h"

void chkHoming() {
  switch(ms->homingState) {
    case movingToFwdSide:
      if(!limitSwOn()) {
        // start normal homing
        ms->targetDir = sv->homingDir;
        ms->homingState = goingHome;
      }
      break;
            
    case goingHome:
      if(limitSwOn()) {
        // arrived at switch while homing backward
        // HOMED_BIT not cleared until now, homing might have been interrupted
        setStateBit(HOMED_BIT, false);
        ms->targetDir   = !sv->homingDir;
        ms->targetSpeed = sv->homingBackUpSpeed;
        ms->homingState = homeReversing;
      }
      break;
      
    case homeReversing:
      if(!limitSwOn()) {
        // passed switch second (or third) time
        ms->homeTestPos = ms->curPos;
        ms->curPos      = 0;
        ms->homingState = homingToOfs;
       }
      break;
    
    case homingToOfs: 
      if(sv->homingDir ? (ms->curPos <= sv->homeOfs) 
                       : (ms->curPos >= sv->homeOfs)) {
        ms->homing = false;
        setStateBit(HOMED_BIT, 1);
        ms->curPos = sv->homePos;
        stopStepping();
        return;
      }
      break;
  }
}
void homeCommand(bool start) {
  ms->slowing = false;
  if((ms->stateByte & BUSY_BIT) == 0) {
    // not moving -- init speed
    motorOn();
    disableAllInts;
    ms->lastStepTicks = timeTicks;
    enableAllInts;
    ms->curSpeed = sv->jerk;
  }
  if(start && ms->limitPort) {
    ms->limitCountHi = ms->limitCountLo = 0;
    ms->homing = true;
    if(limitSwOn()) {
      // go to fwd side of switch at full homing speed
      ms->homingState = movingToFwdSide;
      ms->targetDir   = !sv->homingDir;
    }
    else {
      ms->homingState = goingHome;
      ms->targetDir   = sv->homingDir;
    }
    ms->targetSpeed = sv->homingSpeed;
    setStateBit(BUSY_BIT,  1);
    // homed sState bit unchanged until we get to limit switch
    //   so homing can be interrupted by move command
  }
  else {
    // fake homing for motors with no limit switch
    // hard stop with no reset
    // set wherever it lands to home pos
    stopStepping();
    ms->curPos = sv->homePos;
    setStateBit(HOMED_BIT, 1);
  }
}
