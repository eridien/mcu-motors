
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"

void calcMotion() {
  bool accelerate = false;
  bool decelerate = false;
  
  if(underAccellLimit()) {
    if(!ms->homing && !ms->stopping && ms->curPos == ms->targetPos) {
      // finished normal move
      stopStepping();
      return;
    }
    if(ms->curDir != ms->targetDir) {
      ms->curDir = ms->targetDir;
    }
    else if(ms->curSpeed < ms->targetSpeed) {
      accelerate = true;
    }
    else if (ms->curSpeed > ms->targetSpeed) {
      decelerate = true;
    }
  }
  else {
    if(ms->curDir != ms->targetDir) {
      decelerate = true;
    }
    else {
      if(!ms->homing && !ms->stopping) {
        int16 distRemaining = (ms->targetPos - ms->curPos);
        if(distRemaining < 0) {
          distRemaining = -distRemaining;
        }
        for(uint8 i = 0; i < sizeof(decellTable)/4; i++) {
          if(ms->curSpeed >= decellTable[i][0] &&
             distRemaining <= decellTable[i][1]) {
            decelerate = true;
            break;
          }
        }
      }
      if(!decelerate) {
        if(ms->curSpeed > ms->targetSpeed) {
          decelerate = true;
        }
        else if(ms->curSpeed < ms->targetSpeed) {
          accelerate = true;
        }
      }
    }
  }
  if(decelerate && ms->curSpeed > sv->accellerationRate) {
    ms->curSpeed -= sv->accellerationRate;
  }
  else if (accelerate) {
    ms->curSpeed += sv->accellerationRate;
    if(ms->curSpeed > sv->maxSpeed) 
       ms->curSpeed = sv->maxSpeed;
  }
  setStep();
}

void moveCommand() {
  if((ms->stateByte & HOMED_BIT) == 0) {
    setError(NOT_HOMED_ERROR);
    return;
  }
  ms->homing      = false;
  ms->stopping    = false;
  ms->targetDir   = (ms->targetPos >= ms->curPos);   
  setStateBit(BUSY_BIT, true);
  calcMotion();
}



