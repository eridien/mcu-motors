
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "state.h"
#include "home.h"
#include "motor.h"

void chkHoming() {
  // in the process of stepping
  if(ms->stepPending || ms->stepped) return;
  
  if(limitClosed()) {
    // home switch is closed
    ms->targetDir = 1;
    ms->curSpeed     = sv->homingBackUpSpeed;
    ms->homingState   = homingSwitch;
  }
  else {
    // home switch is open
    if(ms->homingState == homingSwitch) {
      ms->curPos = 0;
      ms->homingState = homingOfs;
    } 
    else if(ms->homingState == homingOfs && ms->curPos >= sv->homeOfs) {
      ms->homingState = homingIdle;
      setStateBit(HOMED_BIT, 1);
      ms->curPos = sv->homePos;
      stopStepping();
      return;
    }
  }
  // set homing ms->speed
  int16 speedDiff = (ms->curDir ? 1 : -1) * sv->accellerationRate;
  if(ms->curSpeed > sv->homingSpeed) {
    // decellerate
    ms->curSpeed -= speedDiff;
  }
  else if(ms->homingState == homeStarting) {
    ms->curSpeed = sv->homingSpeed;
    ms->curDir = 0;
    ms->homingState = homingIn;
  }
  setStep();
}

#ifdef BM
void homeCommand() {
  setStateBit(HOMED_BIT, 0);
  setStateBit(MOTOR_ON_BIT, 1);
  resetLAT = 1;
  ms->homingState = homeStarting;
  setStateBit(BUSY_BIT, true);
}
#else
void homeCommand() {
  if(limitPort[motorIdx]) {
    // this motor has limit switch
    setStateBit(HOMED_BIT, 0);
    setStateBit(MOTOR_ON_BIT, 1);
    ms->homing = true;
    ms->moving = false;
    ms->stopping = false;
    ms->homingState = homeStarting;
    setStateBit(BUSY_BIT, 1);
  }
  else {
    setStateBit(HOMED_BIT, 1);
    ms->curPos = sv->homePos;
  }
}
#endif

