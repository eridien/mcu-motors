
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"
#include "clock.h"

void setStep() {
#ifdef BM
  // adjust ustep
  while(true) {
    // approximate pulsesPerSec
    uint16 pulsesPerSec = ms->curSpeed >> (MAX_USTEP - ms->ustep);
    if(ms->ustep < MAX_USTEP && pulsesPerSec < 500) {
      ms->ustep++;
    }
    // note that you can only reduce ustep when the phase is correct
    else if(ms->ustep > MIN_USTEP && pulsesPerSec > 1500 && 
           (ms->curPos & uStepPhaseMask[ms->ustep]) == 0) {
      ms->ustep--;
    }
    else break;
  }
  // set step timing
  uint16 clkTicks;
  switch (ms->ustep) {
    case 1: clkTicks = (CLK_TICKS_PER_SEC / 4) / ms->curSpeed;
    case 2: clkTicks = (CLK_TICKS_PER_SEC / 2) / ms->curSpeed;
    case 3: clkTicks = (CLK_TICKS_PER_SEC / 1) / ms->curSpeed;
  }
  setNextStep(getLastStep() + clkTicks);
  ms->stepped = false;
  setBiStepLo();
  ms->stepPending = true;

#else
  // check step timing
  uint16 clkTicks = CLK_TICKS_PER_SEC / ms->curSpeed; // 20 usecs/tick
  setNextStep(getLastStep() + clkTicks);
  ms->stepped = false;
  ms->phase += ((ms->curDir ? 1 : -1) & 0x03);
  ms->stepPending = true;
#endif /* BM */
}

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



