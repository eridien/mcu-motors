
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "motor.h"
#include "i2c.h"
#include "state.h"
#include "clock.h"
#include "home.h"
#include "move.h"
#include "stop.h"

union settingsUnion mSet[NUM_MOTORS];

// must match settingsStruct
// assumes 1/40 mm per step
// default is same for all motors
const uint16 settingsInit[NUM_SETTING_WORDS] = {
  4,    // acceleration index,  0 is no acceleration (1000 mm/sec/sec))
  4000, // default speed is 100 mm
  2000, // jerk (start/stop speed limit) (50 mm/sec)
 32000, // max pos is 800 mm (debug))
  2000, // homing speed (50 mm/sec) (must be <= jerk speed!)
  100,   // homing back-up ms->speed (2.5 mm/sec)
  10,   // home offset distance: 0.25 mm
  0,    // home pos value, set cur pos to this after homing
  0,    // limit sw control (0 is normal)
  0,    // backlash width of dead interval
  2400, // backlash speed over dead interval
  30,   // period of clock in usecs  (applies to all motors in mcu)
};

volatile uint16 *stepPort[NUM_MOTORS] = {&stepRPORT, &stepEPORT, &stepXPORT, &stepFPORT, &stepZPORT};
const    uint16 stepMask[NUM_MOTORS] = {stepRBIT, stepEBIT, stepXBIT, stepFBIT, stepZBIT};

volatile uint16 *resetPort[NUM_MOTORS] = {&resetPORT, &resetPORT, &resetPORT, &resetPORT, &resetPORT};
const    uint16 resetMask[NUM_MOTORS] = {resetBIT, resetBIT, resetBIT, resetBIT, resetBIT};

volatile uint16 *faultPort[NUM_MOTORS] = {&faultRPORT, &faultEPORT, &faultXPORT, &faultFPORT, &faultZPORT};
const    uint16 faultMask[NUM_MOTORS] = {faultRBIT, faultEBIT, faultXBIT, faultFBIT, faultZBIT};

// debug for broken board -- moved Z limit to E limit -- using E in place of Z
volatile uint16 *limitPort[NUM_MOTORS] = {&limitRPORT, &limitZPORT, &limitXPORT, &limitFPORT, 0};
const    uint16 limitMask[NUM_MOTORS]  = { limitRBIT,   limitZBIT,   limitXBIT,   limitFBIT,  0};

// globals for use in main chk loop
uint8 motorIdx;
struct motorState *ms;
struct motorSettings *sv;
uint8 mm; // current motor mask (0xf0 or 0x0f or step bit)

void motorInit() {
  dirTRIS = 0;
  ms1TRIS = 0;
  ms2TRIS = 0;
  ms3TRIS = 0;
  resetLAT = 0; // start with reset on
  resetTRIS = 0;
  stepRTRIS = 0;
  stepRLAT = 1; 
  stepETRIS = 0;
  stepELAT = 1; 
  stepXTRIS = 0;
  stepXLAT = 1; 
  stepFTRIS = 0;
  stepFLAT = 1; 
  stepZTRIS = 0;
  stepZLAT = 1; 

  faultRTRIS = 1; // zero means motor fault
  faultETRIS = 1; 
  faultXTRIS = 1;  
  faultFTRIS = 1;  
  faultZTRIS = 1;  

  limitRTRIS = 1; // zero means at limit switch
  limitXTRIS = 1; 
  limitFTRIS = 1; 
  limitZTRIS = 1; 

  uint8 motIdx;
  for (motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
    struct motorState *msp = &mState[motIdx];
    msp->stateByte = 0; // no err, not busy, motor off, and not homed
    msp->phase = 0; // cur step phase
    msp->haveCommand = false;
    msp->stepPending = false;
    msp->stepped = false;
    msp->curSpeed = 0;
    uint8 i;
    for (i = 0; i < NUM_SETTING_WORDS; i++) {
      mSet[motIdx].reg[i] = settingsInit[i];
    }
    msp->acceleration = accelTable[mSet[motIdx].val.accelIdx];
    uint8 lsc = mSet[motIdx].val.limitSwCtl;
    msp->limitSwPolarity = (lsc >> 2) & 0x01;
    msp->homeEndSide = lsc & 0x03;
  }
  setTicksSec();
  clkTicksPerSec = ((uint16) (1000000 / mSet[0].val.mcuClock));
}

bool haveFault() {
#ifdef DEBUG
  return false;
#else
  volatile uint16 *p = faultPort[motorIdx];
  return !(*p & faultMask[motorIdx]);
#endif
}

bool limitSwOn() {
volatile uint16 *p = limitPort[motorIdx];
  if (p != 0) {
    return (ms->limitSwPolarity ? (*p & limitMask[motorIdx])
            : !(*p & limitMask[motorIdx]));
  }
  return false;
}


// setting words are big endian
// write may be short, only setting first entries

void setMotorSettings(uint8 numWordsRecvd) {
  uint8 i;
  for (i = 0; i < numWordsRecvd; i++) {
    mSet[motorIdx].reg[i] = (i2cRecvBytes[motorIdx][2 * i + 2] << 8) |
            i2cRecvBytes[motorIdx][2 * i + 3];
  }
  ms->acceleration = accelTable[sv->accelIdx];
  ms->limitSwPolarity = (sv->limitSwCtl >> 2) & 0x01;
  ms->homeEndSide = sv->limitSwCtl & 0x03;
  setTicksSec();  
  clkTicksPerSec = ((uint16) (1000000 / mSet[0].val.mcuClock));
}

// from event loop

void checkAll() {
  if (haveFault()) {
    setError(MOTOR_FAULT_ERROR);
    return;
  }
  if (ms->stepPending) {
    return;
  }
  
  if (ms->stepped) {
    ms->stepped = false;
    // either adjust curBacklashOfs or curPos
    if(ms->insideBacklash) {
      ms->curBacklashOfs += ((ms->curDir) ? -1 : 1);
#ifdef BM
      ms->phase += ((ms->curDir) ? 1 : -1); 
    }
    else {
      uint8 stepDist = uStepDist[ms->ustep];
      if (ms->curDir) {
        ms->curPos += stepDist;
        ms->phase  += stepDist;
      } else {
        ms->curPos -= stepDist;
        ms->phase -= stepDist;
      }
    }
#else
    }
    else ms->curPos += ((ms->curDir) ? 1 : -1);
#endif
  }
  if ((ms->stateByte & BUSY_BIT) && !haveError()) {
    if (ms->homing) {
      chkHoming();
    } else if (ms->stopping) {
      chkStopping();
    } else {
      // normal moving
      if ((ms->curPos < 0 || ms->curPos >= sv->maxPos) && !ms->noBounds) {
        setError(BOUNDS_ERROR);
        return;
      }
    }
  }
  if ((ms->stateByte & BUSY_BIT) && !haveError()) {
    checkMotor();
  }
}

void motorOn() {
  setStateBit(MOTOR_ON_BIT, 1);
#ifdef BM
  if (resetIsLo()) {
    setResetHi();
    // counter in drv8825 is cleared by reset
    // phase always matches counter in drv8825
    ms->phase = 0;
  }
#else
  setUniPort(ms->phase);
#endif
}

uint8 numBytesRecvd;

bool lenIs(uint8 expected) {
  if (expected != numBytesRecvd) {
    setError(CMD_DATA_ERROR);
    return false;
  }
  return true;
}

void processCommand() {
  volatile uint8 *rb = ((volatile uint8 *) i2cRecvBytes[motorIdx]);
  numBytesRecvd = rb[0];
  uint8 firstByte = rb[1];

  if ((firstByte & 0x80) == 0x80) {
    if (lenIs(2)) {
      // move command
      ms->targetSpeed = sv->speed;
      ms->targetPos = ((int16) (firstByte & 0x7f) << 8) | rb[2];
      moveCommand(false);
    }
  } else if ((firstByte & 0xc0) == 0x40) {
    // speed-move command
    if (lenIs(3)) {
      // changes settings for speed
      sv->speed = (uint16) (firstByte & 0x3f) << 8;
      ms->targetSpeed = sv->speed;
      ms->targetPos = ((int16) rb[2] << 8) | rb[3];
      moveCommand(false);
    }
  } else if ((firstByte & 0xf8) == 0x08) {
    // accel-speed-move command
    if (lenIs(5)) {
      // changes settings for acceleration and speed
      sv->accelIdx = (firstByte & 0x07);
      sv->speed = (((uint16) rb[2] << 8) | rb[3]);
      ms->acceleration = accelTable[sv->accelIdx];
      ms->targetSpeed = sv->speed;
      ms->targetPos = ((int16) rb[4] << 8) | rb[5];
      moveCommand(false);
    }
  } else if ((firstByte & 0xe0) == 0x20) {
    // jog command - no bounds checking and doesn't need to be homed
    if (lenIs(2)) {
      motorOn();
      uint16 dist = (((uint16) (firstByte & 0x0f) << 8) | rb[2]);
      // direction bit is 0x10
      if(firstByte & 0x10) ms->targetPos = ms->curPos + dist;
      else                 ms->targetPos = ms->curPos - dist;
      ms->acceleration = 0;
      ms->targetSpeed  = sv->jerk;
      moveCommand(true);
    }
  } else if (firstByte == 0x1f) {
    // load settings command
    uint8 numWords = (numBytesRecvd - 1) / 2;
    if ((numBytesRecvd & 0x01) == 1 &&
            numWords > 0 && numWords <= NUM_SETTING_WORDS) {
      setMotorSettings(numWords);
    } else {
      setError(CMD_DATA_ERROR);
    }
  } else if ((firstByte & 0xf0) == 0x10) {
    // one-byte commands
    if (lenIs(1)) {
      switch (firstByte & 0x0f) {
        case 0: homeCommand(true);
          break; // start homing
        case 1: ms->nextStateTestPos = true;
          break; // next read pos is actually test pos
        case 2: softStopCommand(false);
          break; // stop,no reset
        case 3: softStopCommand(true);
          break; // stop with reset
        case 4: resetMotor(false);
          break; // hard stop (immediate reset)
        case 5: motorOn();
          break; // reset off
        case 6: homeCommand(false);
          break; // stop, set curpos to setting
        default: setError(CMD_DATA_ERROR);
      }
    }
  } else setError(CMD_DATA_ERROR);
}
void __attribute__((interrupt, shadow, auto_psv)) _T1Interrupt(void) {
  _T1IF = 0;
  timeTicks++;
  int motIdx;
  for (motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
    struct motorState *p = &mState[motIdx];
    if (p->stepPending && p->nextStepTicks == timeTicks) {
      if (p->stepped) {
        // last motor step not handled yet
        setErrorInt(motIdx, STEP_NOT_DONE_ERROR);
        return;
      }
      ms1LAT = ((p->ustep & 0x01) ? 1 : 0);
      ms2LAT = ((p->ustep & 0x02) ? 1 : 0);
      ms3LAT = ((p->ustep & 0x04) ? 1 : 0);
      dirLAT =   p->curDir        ? 1 : 0;
      setBiStepHiInt(motIdx);
      p->stepPending = false;
      p->lastStepTicks = timeTicks;
      p->stepped = true;
    }
  }
}
