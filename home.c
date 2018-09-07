
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
      ms->homingState = goingHome;
      break;

    case goingHome:
      if(limitClosed() != ms->homeReversed) {
        // just passed switch
        if(sv->limitSwCtl >= 2 && ms->homeReversed) {
          // now start homing again in the normal direction
          ms->homeReversed = false;
          ms->homingState  = homeStarting;
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

void homeCommand(bool start) {
  ms->slowing  = false;
  if((ms->stateByte & BUSY_BIT) == 0) {
    // not moving -- init speed
    motorOn();
    GIE=0;
    ms->lastStepTicks = timeTicks;
    GIE=1;
    ms->curSpeed = sv->startStopSpeed;
    setDacToSpeed();
  }
  
  
  //  TODO
  // code multiple things into sv->limitSwCtl
  // start dir: 0, 1, limit sw, not limit sw
  // limit sw closed value
  // restart when reversed
  
  
  switch(sv->limitSwCtl) {
    case 0: ms->homeReversed = false; 
    case 1: ms->homeReversed = true; 
    case 2: ms->homeReversed =  limitClosed(); 
    case 3: ms->homeReversed = !limitClosed(); 
  }
  if(start && limitPort[motorIdx]) {
    ms->homing = true;
    ms->homingState = homeStarting;
    setStateBit(BUSY_BIT,  1);
    setStateBit(HOMED_BIT, 0);
  }
  else {
    ms->curPos = sv->homePos;
    setStateBit(BUSY_BIT,  0);
    setStateBit(HOMED_BIT, 1);
  }
}
