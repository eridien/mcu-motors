
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
  ms->stopping    = false;
  ms->curSpeed    = sv->noAccelSpeedLimit;
  setDacToSpeed();
  setStateBit(BUSY_BIT, 0);
}

void resetMotor(bool all) {
  // all bi motors share reset line
#ifdef BM
  resetLAT = 0; 
#endif
  uint8 savedMotorIdx = motorIdx;
  // set all global motor vars just like event loop
  for(motorIdx=0; motorIdx < NUM_MOTORS; motorIdx++) {
    if(!all && motorIdx != savedMotorIdx) continue;
    mp = stepPort[motorIdx]; // (&PORT)
    mm = stepMask[motorIdx]; // 0xf0 or 0x0f or step bit
    ms = &mState[motorIdx];
    sv = &(mSet[motorIdx].val);
#ifndef BM
    clrUniPort();
#endif
    stopStepping();
    setStateBit(MOTOR_ON_BIT, 0);
    setStateBit(HOMED_BIT, 0);
  }
  // restore global motor vars
  motorIdx = savedMotorIdx;
  mp = stepPort[motorIdx]; // (&PORT)
  mm = stepMask[motorIdx]; // 0xf0 or 0x0f or step bit
  ms = &mState[motorIdx];
  sv = &(mSet[motorIdx].val);
}

void softStopCommand(bool resetAfter) {
  ms->nearTarget         = true;
  ms->homing             = false;
  ms->targetDir          = ms->curDir;
  ms->targetSpeed        = 0;
  ms->resetAfterSoftStop = resetAfter;
  if((ms->stateByte & BUSY_BIT) == 0) {
    GIE=0;
    ms->lastStepTicks = timeTicks;
    GIE=1;
    ms->curSpeed = sv->noAccelSpeedLimit;
    setDacToSpeed();
  }
  setStateBit(BUSY_BIT, 1);
  ms->stopping = true;
}

void chkStopping() {
  if(ms->curSpeed <= sv->noAccelSpeedLimit) {
    stopStepping();
    if(ms->resetAfterSoftStop) {
      // reset only this motor
      resetMotor(false);
    }
    return;
  }
}



