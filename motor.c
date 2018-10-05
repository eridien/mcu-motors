
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

#ifdef U5
volatile uint16 *stepPort[NUM_MOTORS] = {
  &motAPORT, // tube 1
  &motBPORT, // tube 2
  &motCPORT, // tube 3
  &motPPORT, // paster
  &motFPORT, // focus
};

const uint16 stepMask[NUM_MOTORS] = {
  0x000f << motAOFS,
  0x000f << motBOFS,
  0x000f << motCOFS,
  0x000f << motPOFS,
  0x000f << motFOFS,
};

volatile uint16 *faultPort[NUM_MOTORS] = {0, 0, 0, 0, 0};
const uint16 faultMask[NUM_MOTORS] = {0, 0, 0, 0, 0};

// -------- phases ----------
// Color        Bl Pi Ye Or  (red is +5))
//              {1, 1, 0, 0},
//              {0, 1, 1, 0},
//              {0, 0, 1, 1},
//              {1, 0, 0, 1}
uint16 motPhaseValue[NUM_MOTORS][4] = {// motor, phase
  {0x0c << motAOFS, 0x06 << motAOFS, 0x03 << motAOFS, 0x09 << motAOFS},
  {0x0c << motBOFS, 0x06 << motBOFS, 0x03 << motBOFS, 0x09 << motBOFS},
  {0x0c << motCOFS, 0x06 << motCOFS, 0x03 << motCOFS, 0x09 << motCOFS},
  {0x0c << motPOFS, 0x06 << motPOFS, 0x03 << motPOFS, 0x09 << motPOFS},
  {0x0c << motFOFS, 0x06 << motFOFS, 0x03 << motFOFS, 0x09 << motFOFS},
};
#endif

// must match settingsStruct
#ifdef BM
// assumes 1/40 mm per step
// default is same for all motors
const uint16 settingsInit[NUM_SETTING_WORDS] = {
  5, // acceleration rate index,  0 is no acceleration
  4000, // default speed is 100 mm
  1200, // start/stop speed limit (30 mm/sec)
  16000, // max pos is 400 mm
  4000, // homing speed (100 mm/sec)
  60, // homing back-up ms->speed (1.5 mm/sec)
  40, // home offset distance: 1 mm
  0, // home pos value, set cur pos to this after homing
  0, // limit sw control (0 is normal)
  40, // period of clock in usecs  (applies to all motors)
};

#else

// assumes 1/50 mm per step
// default is same for all motors
const uint16 settingsInit[NUM_SETTING_WORDS] = {
  5, // acceleration rate index,  0 is no acceleration
  4000, // default speed is 100 mm
  1200, // start/stop speed limit (30 mm/sec)
  16000, // max pos is 400 mm
  4000, // homing speed (100 mm/sec)
  60, // homing back-up ms->speed (1.5 mm/sec)
  40, // home offset distance: 1 mm
  0, // home pos value, set cur pos to this after homing
  0, // limit sw control (0 is normal)
  40, // period of clock in usecs  (applies to all motors)
};
#endif /* BM */

#ifdef B1
volatile uint8 *stepPort[NUM_MOTORS] = {&stepPORT};
const    uint8  stepMask[NUM_MOTORS] =  {stepMASK};

volatile uint8 *resetPort[NUM_MOTORS] = {&resetPORT};
const     uint8 resetMask[NUM_MOTORS] = {resetMASK};

volatile uint8 *faultPort[NUM_MOTORS] = {&faultPORT};
const    uint8  faultMask[NUM_MOTORS] = {faultMASK};

volatile uint8 *limitPort[NUM_MOTORS] = {&limitPORT};
const    uint8  limitMask[NUM_MOTORS] = {limitMASK};
#endif

#ifdef B4
volatile uint16 *stepPort[NUM_MOTORS] = {&stepRPORT, &stepEPORT, &stepXPORT};
const    uint16 stepMask[NUM_MOTORS] = {stepRBIT, stepEBIT, stepXBIT};

volatile uint16 *resetPort[NUM_MOTORS] = {&resetRPORT, &resetEPORT, &resetXPORT};
const    uint16 resetMask[NUM_MOTORS] = {resetRBIT, resetEBIT, resetXBIT};

volatile uint16 *faultPort[NUM_MOTORS] = {&faultRPORT, &faultEPORT, &faultXPORT};
const    uint16 faultMask[NUM_MOTORS] = {faultRBIT, faultEBIT, faultXBIT};

volatile uint16 *limitPort[NUM_MOTORS] = {&limitRPORT, 0, &limitXPORT};
const    uint16 limitMask[NUM_MOTORS] = {limitRBIT, 0, limitXBIT};
#endif

// globals for use in main chk loop
uint8 motorIdx;
struct motorState *ms;
struct motorSettings *sv;
uint8 mm; // current motor mask (0xf0 or 0x0f or step bit)

#ifdef B1
volatile unsigned char *mp; // current motor port (like &PORTA)
#else
volatile uint16 *mp; // current motor port (like &PORTA)
#endif

void motorInit() {
#ifdef BM
  dirTRIS = 0;
  ms1TRIS = 0;
  ms2TRIS = 0;
  ms3TRIS = 0;
#ifdef B1
  resetLAT = 0; // start with reset on
  resetTRIS = 0;
#else
  resetRLAT = 0; // start with reset on
  resetELAT = 0; // start with reset on
  resetXLAT = 0; // start with reset on
  resetRTRIS = 0;
  resetETRIS = 0;
  resetXTRIS = 0;
#endif
#endif

#ifdef B1
  stepTRIS = 0;
  faultTRIS = 1; // zero means motor fault
  limitTRIS = 1; // zero means at limit switch  // may be used by dbg4
//  debug1TRIS = 0; // uncomment to use dbg1, overrides faultETRIS
#endif

#ifdef B4
  stepRTRIS = 0;
  stepRLAT = 1; // step idle is high
  stepETRIS = 0;
  stepELAT = 1; // step idle is high
  stepXTRIS = 0;
  stepXLAT = 1; // step idle is high

  faultRTRIS = 1; // zero means motor fault
  faultETRIS = 1; // zero means motor fault
  faultXTRIS = 1; // zero means motor fault 

  limitRTRIS = 1; // zero means at limit switch
  limitXTRIS = 1; // zero means at limit switch
#endif /* B4 */

#ifdef U5
  motATRIS = (motATRIS & ~(0x0f << motAOFS));
  motBTRIS = (motBTRIS & ~(0x0f << motBOFS));
  motCTRIS = (motCTRIS & ~(0x0f << motCOFS));
  motPTRIS = (motPTRIS & ~(0x0f << motPOFS));
  motFTRIS = (motFTRIS & ~(0x0f << motFOFS));

  // const uint16 *limitPort[NUM_MOTORS] = {0,0,0,0,0};
  //  const uint16  limitMask[NUM_MOTORS] = {0,0,0,0,0};
#endif

  uint8 motIdx;
  for (motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
    struct motorState *msp = &mState[motIdx];
    msp->stateByte = 0; // no err, not busy, motor off, and not homed
    msp->phase = 0; // cur step phase
    msp->haveCommand = false;
    msp->stepPending = false;
    msp->stepped = false;
    msp->curSpeed = 0;
    setDacToSpeed();
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
}

bool haveFault() { // B1: comment out to use dbg1 or dbg2
#ifdef B1
  volatile uint8 *p = faultPort[motorIdx];
  return !(*p & faultMask[motorIdx]);
#endif
#ifdef B4
  volatile uint16 *p = faultPort[motorIdx];
  
  // fault input pins for motor R (B4) and E(A4) are not working!   TODO
  if(motorIdx < 2) return false;
  
  return !(*p & faultMask[motorIdx]);
#endif
#ifdef U5
  return false;
#endif
}

bool limitSwOn() { // B1: comment out when limit sw used by dbg4
#ifndef U5
#ifdef B1
  volatile uint8 *p = limitPort[motorIdx];
#else
  volatile uint16 *p = limitPort[motorIdx];
#endif
  if (p != 0) {
    return (ms->limitSwPolarity ? (*p & limitMask[motorIdx])
            : !(*p & limitMask[motorIdx]));
  }
#endif /* U5 */
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
}

// from event loop

void checkAll() {
  if (haveFault()) {
    dbg21
    setError(MOTOR_FAULT_ERROR);
    dbg20
    return;
  }
  if (ms->stepPending) {
    return;
  }
  if (ms->stepped) {
    if (ms->curDir) {
      uint8 stepDist = uStepDist[ms->ustep];
      ms->curPos += stepDist;
      ms->phase  += stepDist;
    } else {
      uint8 stepDist = uStepDist[ms->ustep];
      ms->curPos -= stepDist;
      ms->phase -= stepDist;
    }
    ms->stepped = false;
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
  volatile uint8 *rb = ((volatile uint8 *) & i2cRecvBytes[motorIdx]);
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
      ms->targetSpeed  = sv->startStopSpeed;
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
          break; // next read pos is test pos
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
#ifdef B1

void clockInterrupt(void) {
#else

void __attribute__((interrupt, shadow, auto_psv)) _T1Interrupt(void) {
  _T1IF = 0;
#endif
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
#ifdef BM
      ms1LAT = ((p->ustep & 0x01) ? 1 : 0);
      ms2LAT = ((p->ustep & 0x02) ? 1 : 0);
      ms3LAT = ((p->ustep & 0x04) ? 1 : 0);
      dirLAT =   p->curDir ? 1 : 0;
      setBiStepHiInt(motIdx);
#else
      setUniPortInt(motIdx, p->phase);
#endif
      p->stepPending = false;
      p->lastStepTicks = timeTicks;
      p->stepped = true;
    }
  }
}
