
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
  2400,    // max speed
 16000,    // max pos is 400 mm
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
  5000,    // max pos is 100 mm
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
    p->phase       = 0;    // cur step phase (unipolar only)
    p->haveCommand  = false;
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
  ms->homing      = false;
  ms->stopping    = false;
  setStateBit(BUSY_BIT, false);
}

void resetMotor(bool all) {
  // all bi motors share reset line
#ifdef BM
  resetLAT = 0; 
#endif
  uint8 savedMotorIdx = motorIdx;
  // set all global motor vars just like event loop
  for(motorIdx=0; motorIdx < NUM_MOTORS; motorIdx++) {
    if(!all && motorIdx != savedMotorIdx) continue;
    mp = stepPort[motorIdx]; // (&PORT)
    mm = stepMask[motorIdx]; // 0xf0 or 0x0f or step bit
    ms = &mState[motorIdx];
    sv = &(mSet[motorIdx].val);
#ifndef BM
    clrUniPort();
#endif
    stopStepping();
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
  return (ms->curSpeed <= sv->noAccelSpeedLimit);
}

void chkStopping() {
  if(underAccellLimit()) {
    stopStepping();
    if(ms->resetAfterSoftStop) {
      // reset only this motor
      resetMotor(false);
    }
    return;
  }
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

void softStopCommand(bool resetAfter) {
  ms->homing             = false;
  ms->targetDir          = ms->curDir;
  ms->targetSpeed        = 0;
  ms->stopping           = true;
  ms->resetAfterSoftStop = resetAfter;
  setStateBit(BUSY_BIT, 1);
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
    if(lenIs(NUM_SETTING_WORDS)) setMotorSettings();
  }
  else if((firstByte & 0xf0) == 0x10) {
    if(lenIs(1)) {
      switch(firstByte & 0x0f) {
        case 0: homeCommand();                 break; // start homing
        case 1: ms->nextStateTestPos = true;   break; // next read pos is test pos
        case 2: softStopCommand(false);        break; // stop,no reset
        case 3: softStopCommand(true);         break; // stop with reset
        case 4: resetMotor(false);             break; // hard stop (immediate reset)
        case 5: motorOn();                  break; // reset off
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
