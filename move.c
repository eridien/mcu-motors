
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "stop.h"

uint8 accellLeadingZeros[NUM_MOTORS];

// speed doesn't matter, only run on settings load
void calcDecel(uint8 motIdx) {
  uint16 numZeros;
  for(uint8 i = 0; i < NUM_MOTORS; i++) {
    uint16 accel = mSet[motIdx].val.acceleration;
    for(numZeros = 0; numZeros < 16 && !(accel & 0x8000); 
                      numZeros++, accel <<= 1);
  }
  accellLeadingZeros[motIdx] = numZeros;
}

void setStep(bool coasting) {
  uint16 clkTicks;
#ifdef BM
  if(!coasting) {
    // adjust ustep
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
    // set step timing
    switch (ms->ustep) {
      case 0: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 3); break;
      case 1: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 2); break;
      case 2: clkTicks = CLK_TICKS_PER_SEC / (ms->curSpeed >> 1); break;
      case 3: clkTicks = CLK_TICKS_PER_SEC /  ms->curSpeed      ; break;
    }
  }
  else { // coasting to final position with ustep = max
    ms->ustep = MAX_USTEP;
    clkTicks = (CLK_TICKS_PER_SEC / 45);  // ~ 1 mm/sec
  }
  GIE = 0;
  ms->nextStepTicks = ms->lastStepTicks + clkTicks;
  GIE = 1;
    
  ms->stepped = false;
  setBiStepLo();
  
  ms->stepPending = true;

#else
  // check step timing
  uint16 clkTicks = CLK_TICKS_PER_SEC / ms->curSpeed; // 20 usecs/tick
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
      if(distRemaining <= 4) {
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
        bool distRemPositive = (distRemaining >= 0);
        if(distRemPositive != ms->targetDir) {
          // past target, go back
          ms->targetDir  = distRemPositive;
          ms->nearTarget = true;
          decelerate = true;
        }
        else {
          if(!distRemPositive) {
            distRemaining = -distRemaining;
          }          
          // debug : dist should be 700
          
          // distance to start stopping is (speed*speed)/acceleration
          dbg1=1;
          uint16 speedHi = (ms->curSpeed - sv->startStopSpeed) >> 8;
          uint16 speedSq = (speedHi * speedHi);
          uint8  zeros = accellLeadingZeros[motorIdx];
          if(zeros <= 13 && speedSq > (1 << (13-zeros))) {
            decelerate = true;
            ms->nearTarget = true;
          }
          else {
            uint16 decelDist = speedSq << (zeros+3);
            if(distRemaining <= decelDist) {
              decelerate = true;
              ms->nearTarget = true;
            }
            dbg1=0;
          }
        }
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
    uint16 deltaSpeed = (sv->acceleration / ms->curSpeed);
    if(deltaSpeed == 0) deltaSpeed = 1;
    if(deltaSpeed < ms->curSpeed) {
      ms->curSpeed -= deltaSpeed;
    }
  }
  else if (accelerate) {
    // accel/step = accel/sec / steps/sec
    uint16 deltaSpeed = (sv->acceleration / ms->curSpeed);
    if(deltaSpeed == 0) deltaSpeed = 1;
    ms->curSpeed += deltaSpeed;
    if(ms->curSpeed > ms->targetSpeed) {
     // we just passed target speed
     // we should never go faster than target speed
     ms->curSpeed = ms->targetSpeed;
    }
  }
  setDacToSpeed();
  setStep(coast);
}

void moveCommand() {
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



