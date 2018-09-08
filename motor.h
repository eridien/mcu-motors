
#ifndef MOTOR_H
#define	MOTOR_H

#include <xc.h>
#include "types.h"
#include "pins.h"

// steps are in 1/8 step (bipolar) or one phase (unipolar)
//    for bipolar:
//       steps/rev:        1600
//       dist/rev:           40 mm
//       max distance:      800 mm
//       max step count: 32,000
//
//    for unipolar:
//       steps/rev:        2048
//       dist/rev:           40 mm
//       max distance:      625 mm
//       max step count: 32,000

#ifdef B1
#define NUM_MOTORS 1
#endif
#ifdef B3
#define NUM_MOTORS 3
#endif
#ifdef U6
#define NUM_MOTORS 6
#endif

// global for use in main chk loop
extern uint8  motorIdx;
extern struct motorState      *ms;
extern struct motorSettings   *sv;
extern volatile unsigned char *mp; // motor port (like &PORTA)
extern uint8                   mm; // motor mask (0xf0 or 0x0f or step bit)

#define setBiStepLo()           *stepPort[motorIdx] &= ~stepMask[motorIdx]
#define setBiStepHiInt(_motIdx) *stepPort[_motIdx]  |=  stepMask[_motIdx]
  

#define clrUniPort()       (*mp = (*mp & ~mm));
#define setUniPort(_phase) (*mp = (*mp & ~mm) | motPhaseValue[motorIdx][_phase]);
#define setUniPortInt(_motIdx, _phase)                                   \
  (*stepPort[_motIdx] = (*stepPort[_motIdx] & ~stepMask[_motIdx]) |      \
    motPhaseValue[_motIdx][_phase]);

// constants loadable from command (all are 16 bits))
struct motorSettings {
  uint16 accelIdx;
  uint16 speed;
  uint16 startStopSpeed;
  uint16 maxPos;
  uint16 homingSpeed;
  uint16 homingBackUpSpeed;
  uint16 homeOfs;
  uint16 homePos;    // value to set cur pos after homing
  uint16 limitSwCtl; // codes starting and switch direction and reversing
};

#define NUM_SETTING_WORDS 9

union settingsUnion{
  uint16 reg[NUM_SETTING_WORDS];
  struct motorSettings val;
} mSet[NUM_MOTORS];

// show speed on DAC output pin A0 (ICP Data)
#define setDacToSpeed() DAC1CON1 = (ms->curSpeed >> 8);

// default startup values
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
};

#else

// assumes 1/50 mm per step
// default is same for all motors
const uint16 settingsInit[NUM_SETTING_WORDS] = {
      5, // acceleration rate index,  0 is no acceleration
      5, // acceleration rate index,  0 is no acceleration
   4000, // default speed is 100 mm
   1200, // start/stop speed limit (30 mm/sec)
  16000, // max pos is 400 mm
   4000, // homing speed (100 mm/sec)
     60, // homing back-up ms->speed (1.5 mm/sec)
     40, // home offset distance: 1 mm
      0, // home pos value, set cur pos to this after homing
      0, // limit sw control (0 is normal)
};
#endif /* BM */

#ifdef B1
volatile unsigned char *stepPort[NUM_MOTORS]  = {&stepPORT};
const uint8             stepMask[NUM_MOTORS]  = { stepMASK};

volatile unsigned char *faultPort[NUM_MOTORS] = {&faultPORT};
const uint8             faultMask[NUM_MOTORS] = { faultMASK};
  
volatile unsigned char *limitPort[NUM_MOTORS] = {&limitPORT};
const uint8             limitMask[NUM_MOTORS] = { limitMASK};
#endif /*B1 */
  
#ifdef B3
volatile unsigned char 
           *stepPort[NUM_MOTORS] = {&stepRPORT, &stepEPORT, &stepXPORT};
const uint8 stepMask[NUM_MOTORS] = { stepRBIT,   stepEBIT,   stepXBIT};

volatile unsigned char 
           *faultPort[NUM_MOTORS] = {&faultRPORT, &faultEPORT, &faultXPORT};
const uint8 faultMask[NUM_MOTORS] = { faultRBIT,   faultEBIT,   faultXBIT};

volatile unsigned char 
           *limitPort[NUM_MOTORS] = {&limitRPORT, 0, &limitXPORT};
const uint8 limitMask[NUM_MOTORS] = { limitRBIT,  0,  limitXBIT};
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
const uint8 faultMask[NUM_MOTORS] = {0,0,0,0,0,0};

volatile unsigned char 
           *limitPort[NUM_MOTORS] = {0,0,0,0, &limitZPORT, 0};
const uint8 limitMask[NUM_MOTORS] = {0,0,0,0,  limitZBIT,  0};

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


void motorInit(void);
void checkAll(void);
bool haveFault(void);        // bipolar only
bool limitSwOn(void);
void motorOn(void);
void processMotorCmd(void);
void clockInterrupt(void);
uint16 getLastStepTicks(void);
void   setNextStepTicks(uint16 ticks);

#endif	/* MOTOR_H */

