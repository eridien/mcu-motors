
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "stop.h"

#define DECEL_TABLE_SIZE 5
// for 40 steps/mm this is 200 mm/sec, 150 mm/sec, ...
int16 decelTableSpeeds[DECEL_TABLE_SIZE] = {8000, 6000, 4000, 2500, 2000};

// distance before target pos that deceleration should start
// indexed on speed from decelTableSpeeds above
// calculates distance based on that speed and acceleration setting
uint16 decelDist[NUM_MOTORS][DECEL_TABLE_SIZE];

// this is gonna be SSLLOOWW
void calcDecelTable(uint8 motIdx) {
  uint16 accel = mSet[motIdx].val.acceleration;
  for(uint8 i = 0; i < DECEL_TABLE_SIZE; i++) {
    uint16 speed = decelTableSpeeds[i];
    uint16 tgtSpeed = mSet[motIdx].val.noAccelSpeedLimit;
    // each loop simulates one step of deceleration
    uint16 dist = 0;
    for(; speed > tgtSpeed; dist++) {
      // accel/step = accel/sec / steps/sec
      uint16 deltaSpeed = (accel / speed);
      if(deltaSpeed == 0) deltaSpeed = 1;
      if(deltaSpeed > speed) break;
      speed -= deltaSpeed;
    }
    decelDist[motIdx][i] = dist;
  }
}

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
  uint16 lastTicks = getLastStepTicks();
  setNextStepTicks(lastTicks + clkTicks);
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
  
  if(underAccelLimit()) {
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
        for(uint8 i = 0; i < sizeof(decelDist)/4; i++) {
          if(ms->curSpeed >= decelDist[i][0] &&
             distRemaining <= decelDist[i][1]) {
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
  if(decelerate) {
    // accel/step = accel/sec / steps/sec
    uint16 deltaSpeed = (sv->acceleration / ms->curSpeed);
    if(deltaSpeed == 0) deltaSpeed = 1;
    if(deltaSpeed < ms->curSpeed) {
      ms->curSpeed -= deltaSpeed;
    }
    if(ms->curSpeed < ms->targetSpeed) {
      ms->curSpeed = ms->targetSpeed;
    }
  }
  else if (accelerate) {
    // accel/step = accel/sec / steps/sec
    uint16 deltaSpeed = (sv->acceleration / ms->curSpeed);
    if(deltaSpeed == 0) deltaSpeed = 1;
    ms->curSpeed += deltaSpeed;
    if(ms->curSpeed > ms->targetSpeed) {
      ms->curSpeed = ms->targetSpeed;
    }
  }
  setStep();
}

void moveCommand() {
  if((ms->stateByte & HOMED_BIT) == 0) {
    setError(NOT_HOMED_ERROR);
    return;
  }
  if(ms->curSpeed == 0) {
    ms->targetSpeed = sv->noAccelSpeedLimit;
  }
  ms->homing      = false;
  ms->stopping    = false;
  ms->targetDir   = (ms->targetPos >= ms->curPos);   
  ms->ustep       = MAX_USTEP;
  setStateBit(BUSY_BIT, true);
  checkMotor();
}



