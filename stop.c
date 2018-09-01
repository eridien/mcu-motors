
#include <xc.h>
#include "pins.h"
#include "state.h"
#include "motor.h"
#include "move.h"
#include "stop.h"

void stopStepping() {
  ms->stepPending = false;
  ms->stepped     = false;
  ms->homing      = false;
  ms->stopping    = false;
  ms->curSpeed    = 0;
  setStateBit(BUSY_BIT, false);
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
  ms->homing             = false;
  ms->targetDir          = ms->curDir;
  ms->targetSpeed        = 0;
  ms->stopping           = true;
  ms->resetAfterSoftStop = resetAfter;
  setStateBit(BUSY_BIT, 1);
}

void chkStopping() {
  if(underAccelLimit()) {
    stopStepping();
    if(ms->resetAfterSoftStop) {
      // reset only this motor
      resetMotor(false);
    }
    return;
  }
}



