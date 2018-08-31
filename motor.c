
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "motor.h"
#include "i2c.h"
#include "state.h"
#include "clock.h"
#include "home.h"
#include "move.h"

// globals for use in main chk loop
uint8  motorIdx;
struct motorState      *ms;
struct motorSettings   *sv;
volatile unsigned char *mp; // current motor port (like &PORTA)
uint8                   mm; // current motor mask (0xf0 or 0x0f or step bit)

#ifdef B1
volatile unsigned char *stepPort[NUM_MOTORS]  = {&stepPORT};
uint8                   stepMask[NUM_MOTORS]  = { stepMASK};

volatile unsigned char *faultPort[NUM_MOTORS] = {&faultPORT};
uint8                   faultMask[NUM_MOTORS] = { faultMASK};
  
volatile unsigned char *limitPort[NUM_MOTORS] = {&limitPORT};
uint8                   limitMask[NUM_MOTORS] = { limitMASK};
#endif /*B1 */
  
#ifdef B3
volatile unsigned char 
     *stepPort[NUM_MOTORS] = {&stepRPORT, &stepEPORT, &stepXPORT};
uint8 stepMask[NUM_MOTORS] = { stepRBIT,   stepEBIT,   stepXBIT};

volatile unsigned char 
     *faultPort[NUM_MOTORS] = {&faultRPORT, &faultEPORT, &faultXPORT};
uint8 faultMask[NUM_MOTORS] = { faultRBIT,   faultEBIT,   faultXBIT};

volatile unsigned char 
     *limitPort[NUM_MOTORS] = {&limitRPORT, 0, &limitXPORT};
uint8 limitMask[NUM_MOTORS] = { limitRBIT,  0,  limitXBIT};
#endif /* B3 */

#ifdef U6
volatile unsigned char *stepPort[NUM_MOTORS] = {
  &motAPORT, // tube 1
  &motBPORT, // tube 2
  &motCPORT, // tube 3
  &motPPORT, // paster
  &motZPORT, // camera height
  &motFPORT, // focus
};

uint8 stepMask[NUM_MOTORS] = {
  0x0f << motAOFS,
  0x0f << motBOFS,
  0x0f << motCOFS,
  0x0f << motPOFS,
  0x0f << motZOFS,
  0x0f << motFOFS,
};

volatile unsigned char 
     *faultPort[NUM_MOTORS] = {0,0,0,0,0,0};
uint8 faultMask[NUM_MOTORS] = {0,0,0,0,0,0};

volatile unsigned char 
     *limitPort[NUM_MOTORS] = {0,0,0,0, &limitZPORT, 0};
uint8 limitMask[NUM_MOTORS] = {0,0,0,0, &limitZBIT,  0};

// -------- phases ----------
// Color        Bl Pi Ye Or  (red is +5))
//              {1, 1, 0, 0},
//              {0, 1, 1, 0},
//              {0, 0, 1, 1},
//              {1, 0, 0, 1}
uint8 motPhaseValue[NUM_MOTORS][4] = { // motor, phase
  {0x0c << motAOFS, 0x06 << motAOFS, 0x03 << motAOFS, 0x09 << motAOFS},
  {0x0c << motBOFS, 0x06 << motBOFS, 0x03 << motBOFS, 0x09 << motBOFS},
  {0x0c << motCOFS, 0x06 << motCOFS, 0x03 << motCOFS, 0x09 << motCOFS},
  {0x0c << motPOFS, 0x06 << motPOFS, 0x03 << motPOFS, 0x09 << motPOFS},
  {0x0c << motZOFS, 0x06 << motZOFS, 0x03 << motZOFS, 0x09 << motZOFS},
  {0x0c << motFOFS, 0x06 << motFOFS, 0x03 << motFOFS, 0x09 << motFOFS},
};
#endif /* U6 */

// default startup values
// must match settingsStruct
#ifdef BM
// assumes 1/40 mm per step
uint16 settingsInit[NUM_SETTING_WORDS] = {
  2400,    // max ms->speed: steps/sec (60 mm/sec)
  1200,    // no-accelleration ms->speed limit (30 mm/sec)
  2000,    // accelleration rate step/sec/sec  (50 mm/sec/sec)
  1200,    // homing speed (30 mm/sec)
    60,    // homing back-up ms->speed (1.5 mm/sec)
    40,    // home offset distance: 1 mm
     0,    // home pos value, set cur pos to this after homing
};

#else

// assumes 1/50 mm per step
uint16 settingsInit[NUM_SETTING_WORDS] = {
   600,    // max speed: steps/sec (12 mm/sec )
   300,    // no-accelleration ms->speed limit (6 mm/sec)
   200,    // accelleration rate step/sec/sec  (4 mm/sec/sec)
   300,    // homing speed (6 mm/sec)
   100,    // homing back-up ms->speed (2 mm/sec)
    50,    // home offset distance: 1 mm
     0,    // home pos value, set cur pos to this after homing
};
#endif /* BM */

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
    p->curPos      = POS_UNKNOWN_CODE;
    p->phase       = 0;    // cur step phase (unipolar only)
    p->i2cCmdBusy  = false;
    p->stepPending = false;
    p->stepped     = false;
    for(uint8 i = 0; i < NUM_SETTING_WORDS; i++) {
       mSet[motIdx].reg[i] = settingsInit[i];
    }
  }
}

#ifdef BM
// estimated decell distance by speed
// these could be calculated
// wild guess for now
uint16 decellTable[][2] = {
  {2000, 160},
  {1000,  80},
  { 500,  40},
  { 250,  20},
  { 125,  10},
};

#else

uint16 decellTable[][2] = {
  {1000,  40},
  { 500,  20},
  { 250,  10},
  { 125,   5},
};

#endif /* BM */

void haveFault() {
  extern volatile unsigned char *p = faultPort[motorIdx];
  if(p != NULL) {
    return !(*p & faultMask[motorIdx]);
  }
  return false;
}

bool limitClosed() {
  extern volatile unsigned char *p = limitPort[motorIdx];
  if(p != NULL) {
    return !(*p & limitMask[motorIdx]);
  }
  return false;
}

void setMotorSettings() {
  for(uint8 i = 0; i < NUM_SETTING_WORDS; i++) {
    mSet[motorIdx].reg[i] = (i2cRecvBytes[motorIdx][2*i + 1] << 8) | 
                             i2cRecvBytes[motorIdx][2*i + 2];
  }
}

void stopStepping() {
  ms->stepPending = false;
  ms->stepped     = false;
  ms->moving      = false;
  ms->homing      = false;
  ms->stopping    = false;
  setStateBit(BUSY_BIT, false);
}

void resetAllMotors() {
  // all bi motors share reset line
#ifdef BM
  resetLAT = 0; 
#endif
  uint8 savedMotorIdx = motorIdx;
  // set all global motor vars just like event loop
  for(motorIdx=0; motorIdx < NUM_MOTORS; motorIdx++) {
    mp = stepPort[motorIdx]; // (&PORT)
    mm = stepMask[motorIdx]; // 0xf0 or 0x0f or step bit
    ms = &mState[motorIdx];
    sv = &(mSet[motorIdx].val);
#ifndef BM
    clrUniPort();
#endif
    stopStepping();
    ms->curPos = POS_UNKNOWN_CODE;
    setStateBit(MOTOR_ON_BIT, 0);
    setStateBit(HOMED_BIT, 0);
  }
  // restore global motor vars
  motorIdx = savedMotorIdx;
  mp = stepPort[motorIdx]; // (&PORT)
  mm = stepMask[motorIdx]; // 0xf0 or 0x0f or step bit
  ms = &mState[motorIdx];
  sv = &(mSet[motorIdx].val);
}

bool underAccellLimit() {
  return (ms->speed <= sv->noAccelSpeedLimit);
}

// curUStep, only 1/2 -> 1/8 (1..3) is allowed 
#define MIN_USTEP 1
#define MAX_USTEP 3
uint16 uStepPhaseMask[] = {0x07, 0x03, 0x01, 0x00};
uint16 uStepFactor[]    = {   8,    4,    2,    1};

void setStep() {
#ifdef BM
  // set ustep
  while(true) {
    // approximate pulsesPerSec
    uint16 pulsesPerSec = ms->speed >> (MAX_USTEP - ms->ustep);
    if(ms->ustep < MAX_USTEP && pulsesPerSec < 500) {
      ms->ustep++;
    }
    else if(ms->ustep > MIN_USTEP && pulsesPerSec > 1500 && 
           (ms->curPos & uStepPhaseMask[ms->ustep]) == 0) {
      ms->ustep--;
    }
    else break;
  }
  // set step timing
  uint16 absStepDist = uStepFactor[ms->ustep];
  uint16 stepTicks = 50000 / absStepDist; // 20 usecs/tick
  if(stepTicks == 0) stepTicks = 1;
  setNextStep(getLastStep() + stepTicks);
  ms->stepped = false;
  setBiStepLo();
  ms->stepPending = true;

#else
  // check step timing
  uint16 stepTicks = 50000 / ms->speed; // 20 usecs/tick
  if(stepTicks == 0) stepTicks = 1;
  setNextStep(getLastStep() + stepTicks);
  ms->stepped = false;
  ms->phase += ((ms->dir ? 1 : -1) & 0x03);
  ms->stepPending = true;
#endif /* BM */
}

void chkStopping() {
  // in the process of stepping
  if(ms->stepPending) return;
  
  if(limitClosed() ) {
    setError(MOTOR_LIMIT_ERROR);
    return;
  }
  
  // check ms->speed/acceleration
  if(!underAccellLimit()) {
    // decellerate
    ms->speed -= sv->accellerationRate;
  }
  else {
    stopStepping();
    if(ms->resetAfterSoftStop) resetMotor();
    return;
  }

  setStep();
}

// from main loop
void chkMotor() {
  if(haveFault()) {
    setError(MOTOR_FAULT_ERROR);
    return;
  }
  if(ms->stepped) {
    if(ms->dir) {
      ms->curPos += uStepFactor[ms->ustep];
    }
    else {
      ms->curPos -= uStepFactor[ms->ustep];
    }
    ms->stepped = false;
  }
  if(ms->moving) {
    if(limitClosed()) {
      setError(MOTOR_LIMIT_ERROR);
    }
    if(!haveError()) {
      chkMoving();
    }
  }
  else if(ms->homing && !haveError()) {
    chkHoming();
  }
  else if(ms->stopping) {
    chkStopping();
  }
}

void softStopCommand(bool resetAfter) {
  ms->homing   = false;
  ms->moving   = false;
  ms->stopping = true;
  setStateBit(BUSY_BIT, 1);
  ms->resetAfterSoftStop = resetAfter;
}

void motorOnCmd() {
  setStateBit(MOTOR_ON_BIT, 1);
#ifdef BM
  resetLAT = 1; 
#else
  setUniPort(ms->phase); 
#endif
}

uint8 numBytesRecvd;

bool lenErr(uint8 expected) {
  if(expected != numBytesRecvd) {
    setError(CMD_DATA_ERROR);
    return true;
  }
  return false;
}
  
// from i2c
void processMotorCmd() {
  volatile uint8 *rb = ((volatile uint8 *) i2cRecvBytes[motorIdx]);
  numBytesRecvd   = rb[0];
  uint8 firstByte = rb[1];
  
  if((firstByte & 0x80) == 0) {
    if(lenErr(2)) return;
    // simple goto pos command, 15-bits in 1/80 mm/sec
    moveCommand(((int16) firstByte << 8) | rb[2]);
  }
  else if((firstByte & 0xfc) == 0x80) {
    if(lenErr(3)) return;
    // set max ms->speed reg encoded as 10 mm/sec, 150 max
    sv->maxSpeed = (firstByte & 0x0f) * 10;
    // followed by goto pos caommand
    moveCommand(((int16) rb[2] << 8) | rb[3]);
  }
  else switch(firstByte & 0xf0) {
    case 0x90: if(!lenErr(1)) homeCommand();          break; // start homing
    case 0xa0: if(!lenErr(1)) softStopCommand(false); break; // stop,no reset
    case 0xb0: if(!lenErr(1)) softStopCommand(true);  break; // stop with reset
    case 0xc0: if(!lenErr(1)) resetMotor();           break; // hard stop (immediate reset)
    case 0xd0: if(!lenErr(1)) motorOnCmd();           break; // reset off
    case 0xf0: if(!lenErr(NUM_SETTING_WORDS)) 
                 setMotorSettings();                  break; // set all regs
    default: lenErr(255); // invalid cmd sets CMD_DATA_ERROR
  }
}

uint16 getLastStep(void) {
  bool tempGIE = GIE;
  GIE = 0;
  uint16 temp = ms->lastStepTicks;
  GIE = tempGIE; 
  return temp;
}

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
        // oops, last motor step not handled yet
        setErrorInt(motIdx, STEP_NOT_DONE_ERROR);
        return;
      }
#ifdef BM
      ms1LAT = ((p->ustep & 0x01) ? 1 : 0);
      ms2LAT = ((p->ustep & 0x02) ? 1 : 0);
      ms3LAT = ((p->ustep & 0x04) ? 1 : 0);
      dirLAT =   p->dir           ? 1 : 0;
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
