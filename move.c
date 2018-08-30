
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"

void chkMoving() {
  // in the process of stepping

  if(limitClosed()) setError(MOTOR_LIMIT_ERROR);
  
  if(ms->stepPending || ms->stepped || haveError()) return;
  
  if((ms->curPos == ms->targetPos) && underAccellLimit()) {
    stopStepping();
    return;
  }
  
  ms->targetDir = (ms->targetPos >= ms->curPos);

  // check ms->speed/acceleration
  if((!underAccellLimit() && withinDecellDist()) || (ms->dir != ms->targetDir))
    // decellerate
    ms->speed -= sv->accellerationRate;
  else if(ms->targetSpeed > ms->speed) {
    // accelerate
    ms->speed += sv->accellerationRate;
  }
  if(ms->speed > sv->maxSpeed) ms->speed = sv->maxSpeed;
  if(ms->speed < sv->minSpeed) ms->speed = sv->minSpeed;

  // check direction
  if(ms->dir != ms->targetDir && underAccellLimit()) {
    // slow enough, change ms->dir
    ms->dir = ms->targetDir;
  }
  setStep();
}

void moveCommand(int16 pos) {
  if(ms->curPos == POS_UNKNOWN_CODE) {
    setError(NOT_HOMED_ERROR);
    return;
  }
  ms->targetPos = pos;
  if(!underAccellLimit()) {
    // already moving fast, keep going same way
    chkMoving();
  }
  else if(ms->curPos != ms->targetPos) {
    ms->dir = (ms->targetPos >= ms->curPos);
    // start moving
    setBusyState(BUSY_MOVING);
    ms->targetSpeed = sv->maxSpeed;
    chkMoving();
  }
  else {
    // already at target
    stopStepping();
  }
}



