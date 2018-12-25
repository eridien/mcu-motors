
#include <xc.h>
#include "pins.h"
#include "state.h"
#include "stop.h"
#include "motor.h"
#include "move.h"
#include "clock.h"

void stopStepping() {
  ms->stepPending = false;
  ms->stepped     = false;
  ms->homing      = false;
  ms->slowing     = false;
  ms->stopping    = false;
  ms->curSpeed    = 0;
  setStateBit(BUSY_BIT, 0);
}

void resetMotor() {
  setResetLo();
  stopStepping();
  setStateBit(MOTOR_ON_BIT, 0);
  setStateBit(HOMED_BIT, 0);
}

void softStopCommand(bool resetAfter) {
  ms->slowing            = true;
  ms->homing             = false;
  ms->targetDir          = ms->curDir;
  ms->targetSpeed        = 0;
  ms->resetAfterSoftStop = resetAfter;
  if((ms->stateByte & BUSY_BIT) == 0) {
    disableAllInts;
    ms->lastStepTicks = timeTicks;
    enableAllInts;
    ms->curSpeed = sv->jerk; // triggers shutdown code
  }
  setStateBit(BUSY_BIT, 1);
  ms->stopping = true;
}

void chkStopping() {
  if(ms->curSpeed <= sv->jerk || sv->accelIdx == 0) {
    stopStepping();
    if(ms->resetAfterSoftStop) {
      resetMotor();
    }
    return;
  }
}



