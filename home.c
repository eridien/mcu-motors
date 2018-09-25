
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
      ms->targetDir   = ms->homeDir;
      ms->homingState = goingHome;
      break;

    case goingHome:
      if(limitSwOn() != ms->homeWillReverse) {
        // passed switch
        setStateBit(HOMED_BIT, false);
        if(ms->homeWillReverse) {
          // start homing again in the other direction
          ms->homeWillReverse = false;
          ms->homeDir         = !ms->homeDir;
          ms->homingState     = homeStarting;
        }
        else {
          ms->targetDir   = !ms->homeDir;
          ms->targetSpeed =  sv->homingBackUpSpeed;
          ms->homingState =  homeReversing;
        }
      }
      break;
      
    case homeReversing:
      if(!limitSwOn()) {
        // passed switch second time
        ms->homeTestPos = ms->curPos;
        ms->curPos = 0;
        ms->homingState = homingToOfs;
       }
      break;
    
    case homingToOfs: 
      if((ms->curPos >= sv->homeOfs) != ms->homeDir) {
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
    ms->curSpeed = sv->startStopSpeed;
    setDacToSpeed();
  }
  if(start && limitPort[motorIdx]) {
    ms->homing = true;
    ms->homingState = homeStarting;
    switch ((sv->limitSwCtl >> 3) & 0x03) {
      case 0: ms->homeDir = 0;            break;
      case 1: ms->homeDir = 1;            break;
      case 2: ms->homeDir =  limitSwOn(); break;
      case 3: ms->homeDir = !limitSwOn(); break;
    }
    ms->homeWillReverse = ((ms->homeEndSide == 1 &&  limitSwOn()) ||
                           (ms->homeEndSide == 2 && !limitSwOn()));
    setStateBit(BUSY_BIT,  1);
    // homed state bit unchanged until we get to limit switch
    // so homing can be interrupted by move command
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
