
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "state.h"
#include "home.h"
#include "motor.h"

void chkHoming() {
  switch(ms->homingState) {
    case homeStarting:
      ms->targetSpeed = sv->homingSpeed;
      ms->targetDir   = ms->homeReversed;
      ms->homingState =  goingHome;
      break;
      
    case goingHome:
      if(limitClosed() != ms->homeReversed) {
        ms->targetDir   = !ms->homeReversed;
        ms->targetSpeed =  sv->homingBackUpSpeed;
        ms->homingState =  homeReversing;
      }
      break;
      
    case homeReversing:
      if(limitClosed() == ms->homeReversed) {
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
  setStateBit(HOMED_BIT, 0);
  setStateBit(MOTOR_ON_BIT, 1);
  resetLAT = 1;
  ms->homingState = goingHome;
  setStateBit(BUSY_BIT, true);
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
    setStateBit(BUSY_BIT, 1);
    chkHoming();
  }
  else {
    setStateBit(HOMED_BIT, 1);
    ms->curPos = sv->homePos;
  }
}
#endif

