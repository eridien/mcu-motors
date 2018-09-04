
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "stop.h"

//#define DECEL_TABLE_SIZE 5
//int16 decelTableSpeeds[DECEL_TABLE_SIZE] = {8000, 6500, 5000, 3500, 2000};
//
//// distance before target pos that deceleration should start
//// indexed on speed from decelTableSpeeds above
//// calculates distance based on that speed and acceleration setting
//uint16 decelDist[NUM_MOTORS][DECEL_TABLE_SIZE];
//
//void calcDecelTable(uint8 motIdx) {
//  uint16 accel = mSet[motIdx].val.acceleration;
//  for(uint8 i = 0; i < DECEL_TABLE_SIZE; i++) {
//    uint16 speed = decelTableSpeeds[i];
//    uint16 tgtSpeed = mSet[motIdx].val.noAccelSpeedLimit;
//    // each loop simulates one step of deceleration
//    uint16 dist = 0;
//    for(; speed > tgtSpeed; dist++) {
//      // accel/step = accel/sec / steps/sec
//      uint16 deltaSpeed = (accel / speed);
//      if(deltaSpeed == 0) deltaSpeed = 1;
//      if(deltaSpeed > speed) break;
//      speed -= deltaSpeed;
//    }
//    decelDist[motIdx][i] = dist*3;
//  }
//}

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
    
    if (ms->curSpeed <= sv->noAccelSpeedLimit) {
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
      if(distRemaining <= 8) {
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
          // check distance to target to see if we need to slow down
//          for(uint8 i = 0; i < DECEL_TABLE_SIZE; i++) {
//            if(ms->curSpeed >= decelTableSpeeds[i] &&
//               distRemaining <= decelDist[motorIdx][i]) {
//              decelerate = true;
//              ms->nearTarget = true;
//              break;
//            }
//          }
          // very crude estimate of when to start slowing down
          if(ms->curSpeed >= distRemaining*3) {
              decelerate = true;
              ms->nearTarget = true;
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
    ms->curSpeed = sv->noAccelSpeedLimit;
    ms->curDir = ms->targetDir;
    setDacToSpeed();
  }
  setStateBit(BUSY_BIT, 1);
}



