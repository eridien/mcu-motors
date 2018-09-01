
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

// globals for use in main chk loop
uint8  motorIdx;
struct motorState      *ms;
struct motorSettings   *sv;
volatile unsigned char *mp; // current motor port (like &PORTA)
uint8                   mm; // current motor mask (0xf0 or 0x0f or step bit)

void motorInit() {
#ifdef BM
  dirTRIS   = 0;
  ms1TRIS   = 0;
  ms2TRIS   = 0;
  ms3TRIS   = 0;
  resetLAT  = 0;  // start with reset on
  resetTRIS = 0;
#endif
  
#ifdef B1
  stepTRIS  = 0;
  faultTRIS = 1;  // zero means motor fault
  limitTRIS = 1;  // zero means at limit switch
#endif
  
#ifdef B3
  stepRTRIS  = 0;
  stepRLAT   = 1;   // step idle is high
  stepETRIS  = 0;
  stepELAT   = 1;   // step idle is high
  stepXTRIS  = 0;
  stepXLAT   = 1;   // step idle is high

  faultRTRIS = 1;  // zero means motor fault
  faultETRIS = 1;  // zero means motor fault
  faultXTRIS = 1;  // zero means motor fault 
  
  limitRTRIS = 1;  // zero means at limit switch
  limitXTRIS = 1;  // zero means at limit switch
#endif /* B3 */

  for(uint8 motIdx=0; motIdx < NUM_MOTORS; motIdx++) {
    struct motorState *p = &mState[motIdx];
    p->stateByte   = 0;    // no err, not busy, motor off, and not homed
    p->phase       = 0;    // cur step phase (unipolar only)
    p->haveCommand  = false;
    p->stepPending = false;
    p->stepped     = false;
    p->curSpeed    = 0;
    for(uint8 i = 0; i < NUM_SETTING_WORDS; i++) {
       mSet[motIdx].reg[i] = settingsInit[i];
    }
    calcDecelTable(motIdx);
  }
}

bool haveFault() {
  volatile unsigned char *p = faultPort[motorIdx];
  if(p != NULL) {
    return !(*p & faultMask[motorIdx]);
  }
  return false;
}

bool limitClosed() {
  volatile unsigned char *p = limitPort[motorIdx];
  if(p != NULL) {
    return !(*p & limitMask[motorIdx]);
  }
  return false;
}

// setting words are big endian
void setMotorSettings() {
  for(uint8 i = 0; i < NUM_SETTING_WORDS; i++) {
    mSet[motorIdx].reg[i] = (i2cRecvBytes[motorIdx][2*i + 2] << 8) | 
                             i2cRecvBytes[motorIdx][2*i + 3];
  }
  calcDecelTable(motorIdx);
}

// from event loop
void chkMotor() {
  if(haveFault()) {
    setError(MOTOR_FAULT_ERROR);
    return;
  }
  if(ms->curPos < 0 || ms->curPos >= sv->maxPos) {
    setError(MOTOR_LIMIT_ERROR);
    return;
  }
  if(ms->stepPending) {
    return;
  }
  if(ms->stepped) {
    if(ms->curDir) {
      ms->curPos += uStepDist[ms->ustep];
    }
    else {
      ms->curPos -= uStepDist[ms->ustep];
    }
    ms->stepped = false;
  }
  if (!haveError()) {
    if(ms->homing) {
      chkHoming();
    }
    if(ms->stopping) {
      chkStopping();
    }
    if(ms->stateByte & BUSY_BIT) {
      calcMotion();
    }
  }
}

void motorOn() {
  setStateBit(MOTOR_ON_BIT, 1);
#ifdef BM
  resetLAT = 1; 
#else
  setUniPort(ms->phase); 
#endif
}

uint8 numBytesRecvd;

bool lenIs(uint8 expected) {
  if(expected != numBytesRecvd) {
    setError(CMD_DATA_ERROR);
    return false;
  }
  return true;
}

// from i2c
void processMotorCmd() {
  volatile uint8 *rb = ((volatile uint8 *) &i2cRecvBytes[motorIdx]);
  numBytesRecvd   = rb[0];
  uint8 firstByte = rb[1];
  
  if((firstByte & 0x80) == 0x80) {
    if(lenIs(2)) {
      // simple goto pos command
      ms->targetSpeed = sv->maxSpeed;
      ms->targetPos   = ((int16) (firstByte & 0x7f) << 8) | rb[2];
      moveCommand();
    }
  }
  // speed-move command
  else if((firstByte & 0xc0) == 0x40) {
    if(lenIs(3)) {
      ms->targetSpeed = (uint16) (firstByte & 0x3f) << 8;
      ms->targetPos   = ((int16) rb[2] << 8) | rb[3];
      moveCommand();
    }
  }
  else if(firstByte == 0x1f) {
    if(lenIs(1+NUM_SETTING_WORDS*2)) setMotorSettings();
  }
  else if((firstByte & 0xf0) == 0x10) {
    if(lenIs(1)) {
      switch(firstByte & 0x0f) {
        case 0: homeCommand();                 break; // start homing
        case 1: ms->nextStateTestPos = true;   break; // next read pos is test pos
        case 2: softStopCommand(false);        break; // stop,no reset
        case 3: softStopCommand(true);         break; // stop with reset
        case 4: resetMotor(false);             break; // hard stop (immediate reset)
        case 5: motorOn();                     break; // reset off
        case 6: ms->curPos = sv->homePos;      break;  // set curpos to setting
        default: lenIs(255); // invalid cmd sets CMD_DATA_ERROR
      }
    }
  }
  else lenIs(255); // invalid cmd sets CMD_DATA_ERROR
}

// lastStepTicks used in interrupts
uint16 getLastStep(void) {
  bool tempGIE = GIE;
  GIE = 0;
  uint16 temp = ms->lastStepTicks;
  GIE = tempGIE; 
  return temp;
}

// nextStepTicks used in interrupts
void setNextStep(uint16 ticks) {
  bool tempGIE = GIE;
  GIE = 0;
  ms->nextStepTicks = ticks;
  GIE = tempGIE; 
}

void clockInterrupt(void) {
  timeTicks++;
  for(int motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
    struct motorState *p = &mState[motIdx];
    if(p->stepPending && p->nextStepTicks == timeTicks) {
      if(p->stepped) {
        // last motor step not handled yet
        setErrorInt(motIdx, STEP_NOT_DONE_ERROR);
        return;
      }
#ifdef BM
      ms1LAT = ((p->ustep & 0x01) ? 1 : 0);
      ms2LAT = ((p->ustep & 0x02) ? 1 : 0);
      ms3LAT = ((p->ustep & 0x04) ? 1 : 0);
      dirLAT =   p->curDir        ? 1 : 0;
      setBiStepHiInt(motIdx);
#else
      setUniPortInt(motIdx, ms->phase); 
#endif
      p->stepPending = false;
      p->lastStepTicks = timeTicks;
      p->stepped = true;
    }
  }
}
