
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

bool haveSettings[NUM_MOTORS];
union settingsUnion mSet[NUM_MOTORS];

// these should be in state record

volatile uint16 *stepPort[NUM_MOTORS] = {&stepAPORT, &stepBPORT, &stepCPORT, &stepDPORT};
const    uint16  stepMask[NUM_MOTORS] = {stepABIT, stepBBIT, stepCBIT, stepDBIT};

volatile uint16 *resetPort[NUM_MOTORS] = {&resetAPORT, &resetBPORT, &resetCPORT, &resetDPORT};
const    uint16  resetMask[NUM_MOTORS] = {resetABIT, resetBBIT, resetCBIT, resetDBIT};

volatile uint16 *faultPort[NUM_MOTORS] = {&faultAPORT, &faultBPORT, &faultCPORT, &faultDPORT};
const    uint16  faultMask[NUM_MOTORS] = {faultABIT, faultBBIT, faultCBIT, faultDBIT};
         
volatile uint16 *limPort[NUM_MOTORS]   = {&limAPORT, &limBPORT, &limCPORT, &limDPORT};
const    uint16  limMask[NUM_MOTORS]   = {limABIT, limBBIT, limCBIT, limDBIT};
         
// globals for use in main chk loop
uint8                 motorIdx;
struct motorState    *ms;
struct motorSettings *sv;

void motorInit() {
 dirTRIS = 0;
  ms1TRIS = 0;
  ms2TRIS = 0;
  
  resetALAT  = 0; // start with reset on
  resetATRIS = 0;
  resetBLAT  = 0; // start with reset on
  resetBTRIS = 0;
  resetCLAT  = 0; // start with reset on
  resetCTRIS = 0;
  resetDLAT  = 0; // start with reset on
  resetDTRIS = 0;
  
  stepALAT   = 1; 
  stepBLAT   = 1; 
  stepCLAT   = 1; 
  stepDLAT   = 1; 

  stepATRIS  = 0;
  stepBTRIS  = 0;
  stepCTRIS  = 0;
  stepDTRIS  = 0;

#ifndef DEBUG
  faultATRIS = 1; // zero input means motor fault
  faultBTRIS = 1; 
  faultCTRIS = 1;  
  faultDTRIS = 1;  
#endif
  
  limATRIS = 1; // limit switch input
  limBTRIS = 1; 
  limCTRIS = 1; 
  limDTRIS = 1; 

  uint8 motIdx;
  for (motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
    haveSettings[motIdx] = false;
    struct motorState *msp = &mState[motIdx];
    msp->stateByte = 0; // no err, not busy, motor off, and not homed
    msp->phase = 0; // cur step phase
    msp->haveCommand = false;
    msp->stepPending = false;
    msp->stepped = false;
    msp->curSpeed = 0;
  }
}
#include "i2c.h" // DEBUG

bool haveFault() {
#ifdef DEBUG
  return false;
#else
  if(i2cAddrBase == I2C_ADDR_1 && motorIdx == 2) 
           return false;  // DEBUG -- IGNORE BAD MCUB CHIP A4 input -- TODO
  volatile uint16 *p = faultPort[motorIdx];
  return !(*p & faultMask[motorIdx]);
#endif
}

bool limitSwOn() {
  volatile uint16 *p = ms->limitPort;
  if (p != 0) {
    bool swOn;
    if(ms->limActThres)
         swOn =  (ms->limitCntTimeout > ms->limActThres);
    else swOn = !(*p & ms->limitMask);
    return ((sv->limitSwCtl & LIM_POL_MASK) ? !swOn : swOn);
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
  ms->acceleration = accelTable[mSet[motorIdx].val.accelIdx];
  uint16 lsc = mSet[motorIdx].val.limitSwCtl;
  if(lsc) {
    ms->limitPort   = limPort[motorIdx];
    ms->limitMask   = limMask[motorIdx];
    ms->limActThres = (lsc & LIM_ACT_TIMEOUT_MASK) >> (LIM_ACT_TIMEOUT_OFS-6);
    ms->limActHyst  = (lsc & LIM_ACT_HYST_MASK)    << (5-LIM_ACT_HYST_OFS);
  }
  setTicksSec();
  clkTicksPerSec = ((uint16) (1000000 / mSet[0].val.mcuClock));
  haveSettings[motorIdx] = true;
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
    uint8 stepDist = uStepDist[ms->ustep];
    int8  signedDist = ((ms->curDir) ? stepDist : -stepDist); 
    ms->phase += signedDist;
    
    if(sv->backlashWid) {
      if((ms->backlashPos < 0) && ms->curDir) {
        // reversing from backward to forward outside dead zone
        ms->backlashPos = stepDist;
        signedDist -= sv->backlashWid;
        if(signedDist < 0) signedDist = 0;
      }
      else if((ms->backlashPos >= (int16) sv->backlashWid) && !ms->curDir) {
        // reversing from forward to backward outside dead zone
        ms->backlashPos = sv->backlashWid - stepDist;
        signedDist += sv->backlashWid;
        if(signedDist > 0) signedDist = 0;
      }
      else if(ms->backlashPos >= 0 && ms->backlashPos < (int16) sv->backlashWid){
        // moving inside backlash dead zone
        ms->backlashPos += signedDist;
        if(ms->backlashPos < 0) {
          signedDist = ms->backlashPos;
        }
        else if(ms->backlashPos >= (int16) sv->backlashWid) {
          signedDist = ms->backlashPos - sv->backlashWid;
        }
        else signedDist = 0;
      }
    }
    ms->curPos += signedDist;
    
    if(ms->limActThres) {
      ms->limitCntTimeout ++;
      if(*ms->limitPort & ms->limitMask) {
        if(ms->limitCountLo > ms->limActHyst) ms->limitCntTimeout = 0;
        ms->limitCountHi++;
        ms->limitCountLo = 0;
      }
      else {
        if(ms->limitCountHi > ms->limActHyst) ms->limitCntTimeout = 0;
        ms->limitCountLo++;
        ms->limitCountHi = 0;
      }
    }
  }
  if ((ms->stateByte & BUSY_BIT) && !haveError()) {
    if (ms->homing) {
      chkHoming();
    } else if (ms->stopping) {
      chkStopping();
    } else {
      // normal moving
      if ((ms->curPos < sv->minPos || ms->curPos > sv->maxPos) 
           && !ms->noBounds) {
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
  if (resetIsLo()) {
    setResetHi();
    // counter in drv8825 is cleared by reset
    // ms->phase always matches internal phase counter in drv8825
    ms->phase = 0;
  }
}

uint8 numBytesRecvd;

// called on every command except settings
bool lenIs(uint8 expected, bool chkSettings) {
  if(chkSettings && !haveSettings[motorIdx]) {
    setError(NO_SETTINGS);
    return false;
  }
  if (expected != numBytesRecvd) {
    setError(CMD_DATA_ERROR);
    return false;
  }
  return true;
}

void processCommand() {
  volatile uint8 *rb = ((volatile uint8 *) i2cRecvBytes[motorIdx]);
  numBytesRecvd   = rb[0];
  uint8 firstByte = rb[1];
  if ((firstByte & 0x80) == 0x80) {
    if (lenIs(2, true)) {
      // move command
      ms->targetSpeed = sv->speed;
      ms->targetPos = ((int16) (firstByte & 0x7f) << 8) | rb[2];
      moveCommand(false);
    }
  } else if ((firstByte & 0xc0) == 0x40) {
    // speed-move command
    if (lenIs(3, true)) {
      // changes settings for speed
      sv->speed = (uint16) (firstByte & 0x3f) << 8;
      ms->targetSpeed = sv->speed;
      ms->targetPos = (int16) (((uint16) rb[2] << 8) | rb[3]);
      moveCommand(false);
    }
  } else if ((firstByte & 0xf8) == 0x08) {
    // accel-speed-move command
    if (lenIs(5, true)) {
      // changes settings for acceleration and speed
      sv->accelIdx = (firstByte & 0x07);
      sv->speed = (((uint16) rb[2] << 8) | rb[3]);
      ms->acceleration = accelTable[sv->accelIdx];
      ms->targetSpeed = sv->speed;
      ms->targetPos = (int16) (((uint16) rb[4] << 8) | rb[5]);
      moveCommand(false);
    }
  } else if ((firstByte & 0xe0) == 0x20) {
    // jog command relative - no bounds checking and doesn't need to be homed
    if (lenIs(2, true)) {
      motorOn();
      uint16 dist = ((( (uint16) firstByte & 0x0f) << 8) | rb[2]);
      // direction bit is in d4
      if(firstByte & 0x10) ms->targetPos = ms->curPos + dist;
      else                 ms->targetPos = ms->curPos - dist;
      ms->acceleration = 0;
      ms->targetSpeed  = sv->jerk;
      moveCommand(true);
    }
  } else if (firstByte == 0x02) {
    // jog command relative - no bounds checking and doesn't need to be homed
    if (lenIs(3, true)) {
      motorOn(); 
      ms->targetPos    = ms->curPos + (int16) (((uint16) rb[2] << 8) | rb[3]);
      ms->acceleration = 0;
      ms->targetSpeed  = sv->jerk;
      moveCommand(true);
    }
  } else if (firstByte == 0x03) {
    // jog command relative - no bounds checking and doesn't need to be homed
    if (lenIs(3, true)) {
      motorOn();
      ms->targetPos    = (int16) (((uint16) rb[2] << 8) | rb[3]);
      ms->acceleration = 0;
      ms->targetSpeed  = sv->jerk;
      moveCommand(true);
    }
  } else if (firstByte == 0x01) {
    // setPos command
    if (lenIs(3, false)) {
      ms->curPos =  (int16) (((uint16) rb[2] << 8) | rb[3]);
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
  } else if(firstByte == 0x07) {
   // 2-byte extra commands
    if (lenIs(2, true)) {
      if((rb[2] & 0xf8) == 0x08) {
        // clamp or unclamp limit switch to ground
        switch(rb[2] & 0x06) {
          case 0: limALAT = 0; limATRIS = (rb[2] & 0x01); break;
          case 2: limBLAT = 0; limBTRIS = (rb[2] & 0x01); break;
          case 4: limCLAT = 0; limCTRIS = (rb[2] & 0x01); break;
          case 6: limDLAT = 0; limDTRIS = (rb[2] & 0x01); break;
        }
      } else {
        setError(CMD_DATA_ERROR);
      }
    } else {
      setError(CMD_DATA_ERROR);
    }
  } else if((firstByte & 0xfc) == 0x04) {
   // next status contains special value
    if (lenIs(1, true)) {
      ms->nextStateSpecialVal = (firstByte & 0x03) + 1;
    } else {
      setError(CMD_DATA_ERROR);
    }
  } else if ((firstByte & 0xf0) == 0x10) {

    uint8 bottomNib = firstByte & 0x0f;
    // one-byte commands
    if (lenIs(1, (bottomNib != 4 && bottomNib != 7))) {
      switch (bottomNib) {
        case 0: homeCommand(true);           break; // start homing
        case 2: softStopCommand(false);      break; // stop,no reset
        case 3: softStopCommand(true);       break; // stop with reset
        case 4: resetMotor();                break; // hard stop (immediate reset)
        case 5: motorOn();                   break; // reset off
        case 6: homeCommand(false);          break; // stop, set curpos to setting
        default: setError(CMD_DATA_ERROR);
      }
    }
  } 
  else setError(CMD_DATA_ERROR);
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
      dirLAT =   p->curDir        ? 1 : 0;
      setBiStepHiInt(motIdx);
      p->stepPending = false;
      p->lastStepTicks = timeTicks;
      p->stepped = true;
    }
  }
}
