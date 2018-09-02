
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
    case homeStarting:
      ms->targetSpeed = sv->homingSpeed;
      ms->targetDir   = ms->homeReversed;
      ms->homingState =  goingHome;
      break;

    case goingHome:
      if(limitClosed() != ms->homeReversed) {
        // just passed switch
        if(ms->homeReversed) {
          // now home in the normal direction
          ms->homeReversed = false;
          ms->homingState = homeStarting;
          break;
        }
        ms->targetDir   = !ms->homeReversed;
        ms->targetSpeed =  sv->homingBackUpSpeed;
        ms->homingState =  homeReversing;
      }
      break;
      
    case homeReversing:
      if(limitClosed() == ms->homeReversed) {
        // just passed switch in other direction
        ms->homeTestPos = ms->curPos;
        ms->curPos = 0;
        ms->homingState = homingToOfs;
       }
      break;
    
    case homingToOfs: 
      if(ms->curPos >= sv->homeOfs) {
        ms->homing = false;
        setStateBit(HOMED_BIT, 1);
        ms->curPos = sv->homePos;
        stopStepping();
        return;
      }
      break;
  }
}

#ifdef BM
void homeCommand() {
  if((ms->stateByte & BUSY_BIT) == 0) {
    GIE=0;
    ms->lastStepTicks = timeTicks;
    ms->curSpeed = sv->homingSpeed;
    GIE=1;
    setDacToSpeed();
  }
  setStateBit(BUSY_BIT, 1);
  setStateBit(HOMED_BIT, 0);
  motorOn();
  ms->homing = true;
  ms->homingState = homeStarting;
  ms->ustep = MAX_USTEP;
}
#else
void homeCommand() {
  if(limitPort[motorIdx]) {
    // this motor has limit switch
    setStateBit(HOMED_BIT, 0);
    motorOn();
    ms->homeReversed = (sv->homeToLim && (sv->homeToLim - 1) == limitClosed());
    ms->homing       = true;
    ms->stopping     = false;
    ms->homingState  = goingHome;
    if((ms->stateByte & BUSY_BIT) == 0) {
      GIE=0;
      ms->lastStepTicks = timeTicks;
      GIE=1;
      ms->curSpeed = sv->noAccelSpeedLimit;
      setDacToSpeed();
    }
    setStateBit(BUSY_BIT, 1);
    chkHoming();
  }
  else {
    setStateBit(HOMED_BIT, 1);
    ms->curPos = sv->homePos;
  }
}
#endif

