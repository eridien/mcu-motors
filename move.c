
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "stop.h"
#include "dist-table.h"

const uint16 accelTable[8] = 
       {4000, 8000, 16000, 24000, 32000, 40000, 50000, 60000};
uint16 accel[NUM_MOTORS];

void calcAccel(uint8 motIdx) {
  accel[motIdx] = accelTable[mSet[motIdx].val.accelCode];
}

uint16 dbgMinClkTicks;

void setStep(bool coasting) {
  uint16 clkTicks;
#ifdef BM
  if(!coasting) {
    // adjust ustep unless deceleratin
    if(!ms->nearTarget) {
      while(true) {
        // approximate pulsesPerSec
        uint16 pulsesPerSec = ms->curSpeed >> (MAX_USTEP - ms->ustep);
        if(ms->ustep < MAX_USTEP && pulsesPerSec < 500) {
          ms->ustep++;
        }
        // note that you can only reduce ustep when the phase is correct
        else if(ms->ustep > MIN_USTEP && pulsesPerSec > 1000 && 
               (ms->curPos & uStepPhaseMask[ms->ustep]) == 0) {
          ms->ustep--;
        }
        else break;
      }
    }
    else {
      // final deceleration must have at least ustep of 1
      // 1/1 stepping is unstable at slow speeds
      if(ms->ustep == 0) {
        ms->ustep = 1;
      }
    }
    // set step timing
    switch (ms->ustep) {
      case 0: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 3); break;
      case 1: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 2); break;
      case 2: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 1); break;
      case 3: clkTicks = CLK_TICKS_PER_SEC /  ms->curSpeed      ; break;
    }
  }
  else { 
    // coasting to final exact position with ustep = max
    ms->ustep = MAX_USTEP;
    clkTicks = (CLK_TICKS_PER_SEC / 45);  // ~ 1 mm/sec
  }
  if (clkTicks < dbgMinClkTicks) {
    dbgMinClkTicks = clkTicks;
  }
  bool err;
  GIE = 0;
  ms->nextStepTicks = ms->lastStepTicks + clkTicks;
  // modulo 2**16 arithmetic
  err = (ms->nextStepTicks - (timeTicks+1)) > 32000;
  GIE = 1;
  if(err) { 
    // nextStepTicks is in the past
    setError(STEP_NOT_DONE_ERROR); 
  }
    
  ms->stepped = false;
  setBiStepLo();
  ms->stepPending = true;
  
#else
  // check step timing
  uint16 clkTicks = CLK_TICKS_PER_SEC / ms->curSpeed; // 40 usecs/tick
  setNextStepTicks(getLastStepTicks() + clkTicks);
  ms->stepped = false;
  ms->phase += ((ms->curDir ? 1 : -1) & 0x03);
  ms->stepPending = true;
#endif /* BM */
}

void checkMotor() {
  bool accelerate = false;
  bool decelerate = false;
  bool coast      = false;
  
  if(!ms->homing && !ms->stopping) {
    // normal move to target position
    
    if (!sv->useAccel) {
      // not using acceleration
      ms->curSpeed = ms->targetSpeed;
      ms->curDir = ms->targetDir = (ms->targetPos > ms->curPos);
    }
    
    else {
      if (ms->curSpeed <= sv->startStopSpeed) {
        // going slower than accel threshold

        if(ms->curPos == ms->targetPos) {
          // finished normal move
          stopStepping();
          return;
        }
        int16 distRemaining = (ms->targetPos - ms->curPos);
        bool distRemPositive = (distRemaining >= 0);
        if(!distRemPositive) {
          distRemaining = -distRemaining;
        }
        if(distRemaining <= uStepDist[MIN_USTEP]) {
          // dist is smaller than largest step
          // move to target at speed 1 and max ustep to make sure to hit target
          coast = true;
        }
        // can chg dir any time when slow
        ms->curDir = ms->targetDir = distRemPositive;
      }

      // going faster than accel threshold
      else if(ms->nearTarget) {
        decelerate = true;
      }
      else {
        if(ms->curDir != ms->targetDir) {
          // we need to chg dir but we are going too fast
          decelerate = true;
        }
        else {
          int16 distRemaining = (ms->targetPos - ms->curPos);
          if(distRemaining < 0) {
            distRemaining = -distRemaining;
          }
          // look up decel dist target
          uint16 decelUstep = ms->ustep;
          if(decelUstep == 0) {
            // never decel to zero at stepping 1/1 -- unstable
            decelUstep = 1;
          }
          uint16 distTgt = calcDist(decelUstep, sv->accelCode, ms->curSpeed);
          if(distRemaining < (distTgt + 200 /* margin */)) {
            decelerate = true;
            ms->nearTarget = true;
          }
        }
      }
      if(!decelerate && !accelerate && !coast) {
        if(ms->curSpeed > ms->targetSpeed) {
          decelerate = true;
        }
        else if(!ms->nearTarget && ms->curSpeed < ms->targetSpeed) {
          accelerate = true;
        }
      }
      if(decelerate) {
        // accel/step = accel/sec / steps/sec
        uint16 deltaSpeed = (accel[motorIdx] / ms->curSpeed);
        if(deltaSpeed == 0) deltaSpeed = 1;
        if(deltaSpeed < ms->curSpeed) {
          ms->curSpeed -= deltaSpeed;
        }
      }
      else if (accelerate) {
        // accel/step = accel/sec / steps/sec
        uint16 deltaSpeed = (accel[motorIdx] / ms->curSpeed);
        if(deltaSpeed == 0) deltaSpeed = 1;
        ms->curSpeed += deltaSpeed;
        if(ms->curSpeed > ms->targetSpeed) {
         // we just passed target speed
         // we should never go faster than target speed
         ms->curSpeed = ms->targetSpeed;
        }
      }
    }
  }
  setDacToSpeed();
  setStep(coast);
}

void moveCommand() {
  dbgMinClkTicks = 0xffff;
          
  if((ms->stateByte & HOMED_BIT) == 0) {
    setError(NOT_HOMED_ERROR);
    return;
  }
  ms->nearTarget  = false;
  ms->homing      = false;
  ms->stopping    = false;
  ms->targetDir = (ms->targetPos >= ms->curPos);   
  if(ms->curSpeed == 0 || (ms->stateByte & BUSY_BIT) == 0) {
    GIE=0;
    ms->lastStepTicks = timeTicks;
    GIE=1;
    ms->curSpeed = sv->startStopSpeed;
    ms->curDir = ms->targetDir;
    setDacToSpeed();
  }
  setStateBit(BUSY_BIT, 1);
}



