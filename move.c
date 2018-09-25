
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "stop.h"
#include "dist-table.h"

const uint16 uStepPhaseMask[4] = {0x07, 0x03, 0x01, 0x00};
const uint16 uStepDist[4]      = {   8,    4,    2,    1};
const uint16 accelTable[8] = 
       {4000, 8000, 16000, 24000, 32000, 40000, 50000, 60000};

uint16 dbgMinClkTicks;

void setStep(bool closing) {
  uint16 clkTicks;
#ifdef BM
  if(!closing) {
    // adjust ustep
    uint8 tgtUstep;
    // we want pps to be between 500 and 1000, if possible
    // low pps gives sw more time to keep up
    if      (ms->curSpeed > (8192 + 4096) / 2) tgtUstep = 0;
    else if (ms->curSpeed > (4096 + 2048) / 2) tgtUstep = 1;
    else if (ms->curSpeed > (2048 + 1024) / 2) tgtUstep = 2;
    else                                       tgtUstep = 3;

    if(tgtUstep != ms->ustep) {
      // you can only change ustep when the drv8825 phase is correct
      if((ms->phase & uStepPhaseMask[tgtUstep]) == 0) {
        ms->ustep = tgtUstep;
      }
    }
  }
  else { 
    // we are closing in (very close to target))
    // if needed, adjust ustep to hit exact position
    int8 dist = ms->targetPos - ms->curPos;
    if(dist < 0) dist = -dist;
    if(dist & uStepPhaseMask[ms->ustep]) {
      ms->ustep++;
    }
  }
  // set step timing
  switch (ms->ustep) {
    case 0: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 3); break;
    case 1: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 2); break;
    case 2: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 1); break;
    case 3: clkTicks = CLK_TICKS_PER_SEC /  ms->curSpeed      ; break;
  }
  if (clkTicks < dbgMinClkTicks) {
    dbgMinClkTicks = clkTicks;
  }
  bool err;
  disableAllInts;
  ms->nextStepTicks = ms->lastStepTicks + clkTicks;
  // modulo 2**16 arithmetic
  err = (ms->nextStepTicks - (timeTicks+1)) > 32000;
  enableAllInts;
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
  bool  accelerate = false;
  bool  decelerate = false;
  bool  closing    = false;
  
  if(ms->homing) {
    if (sv->accelIdx == 0 || 
        ms->curSpeed <= sv->startStopSpeed) {
      ms->curSpeed = ms->targetSpeed;
      ms->curDir = ms->targetDir;
    }
    else {
      decelerate = true;
    }
  }
  else if(ms->stopping) {
    decelerate = true;
  }
  else {
    // normal move to target position

    int16 distRemaining = (ms->targetPos - ms->curPos);
    bool  distRemPositive = (distRemaining >= 0);
    if(!distRemPositive) {
      distRemaining = -distRemaining;
    }
    if(distRemaining == 0) {
      // finished normal move
      stopStepping();
      return;
    }
    if(distRemaining <= uStepDist[MIN_USTEP]) {
      // dist is smaller than largest step
      // adjust ustep each pulse to make sure to hit target exactly
      closing = true;
    }
    else {
      if (sv->accelIdx == 0) {
        // not using acceleration
        ms->curSpeed = ms->targetSpeed;
        ms->curDir = ms->targetDir = (ms->targetPos > ms->curPos);
      }
      else {
        // using acceleration
        if (ms->curSpeed <= sv->startStopSpeed) {
          // going slower than accel threshold
          
          if(ms->curPos == ms->targetPos) {
            // finished normal move
            stopStepping();
            return;
          }
          // can chg dir any time when slow
          ms->curDir = ms->targetDir = distRemPositive;
        }
        
        // going faster than accel threshold
        else if(ms->slowing) {
          decelerate = true;
        }
        else {
          if(ms->curDir != ms->targetDir) {
            // we need to chg dir but we are going too fast
            decelerate = true;
          }
          else {
            // look up decel dist target
            uint16 distTgt = calcDist(sv->accelIdx, ms->curSpeed);
            if(distRemaining < distTgt) {
              decelerate = true;
              ms->slowing = true;
            }
          }
        }
        if(!decelerate && !accelerate && !closing) {
          if(ms->curSpeed > ms->targetSpeed) {
            decelerate = true;
          }
          else if(!ms->slowing && ms->curSpeed < ms->targetSpeed) {
            accelerate = true;
          }
        }
      }
    }
  }
  if(decelerate) {
    // accel/step = accel/sec / steps/sec
    uint16 deltaSpeed = (ms->acceleration / ms->curSpeed);
    if(deltaSpeed == 0) deltaSpeed = 1;
    if(deltaSpeed < ms->curSpeed) {
      ms->curSpeed -= deltaSpeed;
    } 
    else {
      ms->curSpeed = sv->startStopSpeed;
    }
  }
  else if (accelerate) {
    // accel/step = accel/sec / steps/sec
    uint16 deltaSpeed = (ms->acceleration / ms->curSpeed);
    if(deltaSpeed == 0) deltaSpeed = 1;
    ms->curSpeed += deltaSpeed;
    if(ms->curSpeed > ms->targetSpeed) {
      // we just passed target speed
      // we should never go faster than target speed
      ms->curSpeed = ms->targetSpeed;
    }
  }
  setDacToSpeed();
  setStep(closing);
}

void moveCommand() {
  dbgMinClkTicks = 0xffff;

  if((ms->stateByte & HOMED_BIT) == 0) {
    setError(NOT_HOMED_ERROR);
    return;
  }
  ms->slowing     = false;
  ms->homing      = false;
  ms->stopping    = false;
  ms->targetDir   = (ms->targetPos >= ms->curPos);   
  if(ms->curSpeed == 0 || (ms->stateByte & BUSY_BIT) == 0) {
    disableAllInts;
    ms->lastStepTicks = timeTicks;
    enableAllInts;
    ms->curSpeed = sv->startStopSpeed;
    ms->curDir   = ms->targetDir;
    setDacToSpeed();
  }
  setStateBit(BUSY_BIT, 1);
}



