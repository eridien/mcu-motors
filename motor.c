
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
uint16 settingsInit[NUM_SETTING_WORDS] = {
   6400,  // max ms->speed
      2,  // min ms->speed: 2 steps/sec (else sw blows up)
    400,  // no-accelleration ms->speed limit
    100,  // accelleration rate: steps/sec/sec
   1600,  // homing ms->speed    (40 mm/sec)
     80,  // homing back-up speed (2 mm/sec)
     80,  // home offset distance (2 mm)
      0,  // homePos (rot motor is 16000)
};

#else

uint16 settingsInit[NUM_SETTING_WORDS] = {
   500,    // max ms->speed: steps/sec
    50,    // min ms->speed, else sw blows up
   100,    // no-accelleration ms->speed limit
   100,    // accelleration rate: 1 step/sec/sec
   400,    // homing ms->speed
    50,    // homing back-up ms->speed
    50,    // home offset distance: 1 mm
     0,    // homePos
};
#endif /* BM */

void motorInit() {
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
    p->curPos          = POS_UNKNOWN_CODE;  // 1/80 mm
    p->dir             = 1;  // 1 => forward
    p->ustep           = 1;  // 1/2 step per pulse
    p->targetPos       = 1;  // 1/80 mm
    p->speed           = 0;  // 1/80 mm/sec
    p->targetSpeed     = 0;  // 1/80 mm/sec
    p->stepDist        = 0;  // signed distance each step pulse in 1/80 mm
    
    for(uint8 i = 0; i < NUM_SETTING_WORDS; i++) {
       mSet[motIdx].reg[i] = settingsInit[i];
    }
  }

  dirTRIS   = 0;
  ms1TRIS   = 0;
  ms2TRIS   = 0;
  ms3TRIS   = 0;
  resetLAT  = 0;  // start with reset on
  resetTRIS = 0;
}


// estimated decell distance by ms->speed
// wild guess for now
uint16 decellTable[][2] = {
  {150*80, 32*80},
  {100*80, 16*80},
  {80*80,   8*80},
  {60*80,   4*80},
  {40*80,   2*80},
  {20*80,   1*80}
};

bool withinDecellDist() {
  int16 distRemaining = (ms->targetPos >= ms->curPos);
  ms->targetDir = 1;
  if(distRemaining < 0) {
    ms->targetDir = 0;
    distRemaining = -distRemaining;
  }
  if(ms->targetDir != ms->dir || limitClosed()) return true;
  
  for(uint8 i = 0; i < sizeof(decellTable)/2; i++) {
    if(ms->speed >= decellTable[i][0] &&
       distRemaining <= decellTable[i][1]) {
      return true;
    }
  }
  return false;
}
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
  for(uint8 i = 0; i < sizeof(mSet[motorIdx]) / 
                       sizeof(mSet[motorIdx].reg[0]); i++) {
    mSet[motorIdx].reg[i] = (i2cRecvBytes[motorIdx][2*i + 1] << 8) | 
                             i2cRecvBytes[motorIdx][2*i + 2];
  }
}

void stopStepping() {
  ms->stepPending = false;
  ms->stepped     = false;
  setBusyState(NOT_BUSY);
}

void resetMotor() {
  stopStepping();
  ms->curPos = POS_UNKNOWN_CODE;
  setStateBit(MOTOR_ON_BIT, 0);
  setStateBit(HOMED_BIT, 0);
  resetLAT = 0;
}

bool underAccellLimit() {
  return (ms->speed <= sv->noAccelSpeedLimit);
}

uint16 uStepMask[] = {0x1f, 0x0f, 0x07, 0x03, 0x01};
uint16 uStepFact[] = {   1,    2,    4,    8,   16};

void setStep() {
  // check curUStep, only 1/2 -> 1/16 (1..4) is allowed 
  while(true) {
    uint16 pulsesPerSec = ms->speed * uStepFact[ms->ustep];
    if(ms->ustep < 4 && pulsesPerSec < 500) {
      ms->ustep++;
    } 
    else if(ms->ustep > 1 && pulsesPerSec > 1500 && 
           (ms->curPos & uStepMask[ms->ustep]) == 0) {
      ms->ustep--;
    }
    else break;
  }
  // check step timing
  uint16 absStepDist =  16 / uStepFact[ms->ustep]; // each step -> 1/80 mm
  ms->stepDist = (ms->dir ? ms->stepDist : -ms->stepDist);
  uint16 curTimeTicks = getTimeTicks();
  // (dunits/sec) / dunits => 1/seconds
  uint16 invStepSecs = ms->speed / absStepDist;  // 1/sec
  // (ticks/sec) / (1/secs) -> ticks
  uint16 stepTicks = 50000 / invStepSecs; // 20 usecs/tick
  if(stepTicks == 0) stepTicks = 1;
  setNextStep(getLastStep() + stepTicks);
  ms->stepped = false;
  setBiStepLo();
  ms->stepPending = true;
}

void chkStopping() {
  // in the process of stepping
  if(ms->stepPending || ms->stepped) return;
  
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
    ms->curPos += ms->stepDist;
    ms->stepped = false;
  }
  if(getBusyState() == BUSY_MOVING) {
    if(limitClosed() ) {
      setError(MOTOR_LIMIT_ERROR);
    }
    if(!haveError()) {
      chkMoving();
    }
  }
  else if(getBusyState() == BUSY_HOMING && !haveError()) {
    chkHoming();
  }
  else if(getBusyState() == BUSY_STOPPING) {
    chkStopping();
  }
}

void softStopCommand(bool resetAfter) {
  setBusyState(BUSY_STOPPING);
  ms->resetAfterSoftStop = resetAfter;
}

void motorOnCmd() {
  ms->curPos = POS_UNKNOWN_CODE;
  setBusyState(NOT_BUSY);
  setStateBit(MOTOR_ON_BIT, 1);
  setStateBit(HOMED_BIT, 0);
  resetLAT = 1;
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
  GIE = 0;
  uint16 temp = ms->lastStepTicks;
  GIE = 1; 
  return temp;
}

void setNextStep(uint16 ticks) {
  GIE = 0;
  ms->nextStepTicks = ticks;
  GIE = 1; 
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
      ms1LAT = ((p->ustep & 0x01) ? 1 : 0);
      ms2LAT = ((p->ustep & 0x02) ? 1 : 0);
      ms3LAT = ((p->ustep & 0x04) ? 1 : 0);
      dirLAT =   p->dir           ? 1 : 0;
      setBiStepHiInt(motIdx);
      p->stepPending = false;
      p->lastStepTicks = timeTicks;
      p->stepped = true;
    }
  }
}
