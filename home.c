
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "state.h"
#include "home.h"
#include "motor.h"

uint8 homingState = homingIdle;

void chkHoming() {
  // in the process of stepping
  if(ms->stepPending || ms->stepped) return;
  
  if(limitClosed()) {
    // home switch is closed
    ms->targetDir = 1;
    ms->speed     = sv->homingBackUpSpeed;
    homingState   = homingSwitch;
  }
  else {
    // home switch is open
    if(homingState == homingSwitch) {
      ms->curPos = 0;
      homingState = homingOfs;
    } 
    else if(homingState == homingOfs && ms->curPos >= sv->homeOfs) {
      homingState = homingIdle;
      setStateBit(HOMED_BIT, 1);
      ms->curPos = sv->homePos;
      stopStepping();
      return;
    }
  }
  // set homing ms->speed
  int16 speedDiff = (ms->dir ? 1 : -1) * sv->accellerationRate;
  if(ms->speed > sv->homingSpeed) {
    // decellerate
    ms->speed -= speedDiff;
  }
  else if(homingState == homeStarting) {
    ms->speed = sv->homingSpeed;
    ms->dir = 0;
    homingState = homingIn;
  }
  setStep();
}

#ifdef BM
void homeCommand() {
  setStateBit(HOMED_BIT, 0);
  setStateBit(MOTOR_ON_BIT, 1);
  resetLAT = 1;
  homingState = homeStarting;
  setBusyState(BUSY_HOMING);
}
#else
void homeCommand() {
  if(motorIdx == limitZIDX) {
    setStateBit(HOMED_BIT, 0);
    setStateBit(MOTOR_ON_BIT, 1);
    homingState = homeStarting;
    setBusyState(BUSY_HOMING);
  }
  else {
    stopStepping();
    setStateBit(HOMED_BIT, 1);
    ms->curPos = 0;
  }
}
#endif

