
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"

void chkMoving() {
  // in the process of stepping
  if(ms->stepPending || ms->stepped) return;
  
  if(ms->curPos < 0 || ms->curPos >= sv->maxPos) {
    setError(MOTOR_LIMIT_ERROR);
    return;
  }
  ms->targetDir = (ms->targetPos >= ms->curPos); 
  
  bool accelerate = false;
  bool decelerate = false;
  
  if(underAccellLimit()) {
    if(ms->curPos == ms->targetPos) {
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
    if(ms->curSpeed > sv->maxSpeed) ms->curSpeed = sv->maxSpeed;
  }
  setStep();
}

void moveCommand() {
  if((ms->stateByte & HOMED_BIT) == 0) {
    setError(NOT_HOMED_ERROR);
    return;
  }
  ms->homing   = false;
  ms->moving   = true;
  ms->stopping = false;
  if(!underAccellLimit()) {
    // already moving fast, keep going same way
    chkMoving();
  }
  else if(ms->curPos != ms->targetPos) {
    ms->curDir = (ms->targetPos >= ms->curPos);
    // start moving
    setStateBit(BUSY_BIT, true);
    ms->targetSpeed = sv->maxSpeed;
    chkMoving();
  }
  else {
    // already at target
    stopStepping();
  }
}



