
#ifndef MOTOR_H
#define	MOTOR_H

#include <xc.h>
#include "types.h"
#include "pins.h"

#ifdef B1
#define NUM_MOTORS 1
#endif
#ifdef B3
#define NUM_MOTORS 3
#endif
#ifdef U6
#define NUM_MOTORS 5
#endif

// global for use in main chk loop
#ifdef B1
extern volatile uint8  *mp; // motor port (like &PORTA)
#else
extern volatile uint16 *mp;
#endif

extern uint8  motorIdx;
extern struct motorState      *ms;
extern struct motorSettings   *sv;
extern uint8                   mm; // motor mask (0xf0 or 0x0f or step bit)

#define setBiStepLo()           *stepPort[motorIdx] &= ~stepMask[motorIdx]
#define setBiStepHiInt(_motIdx) *stepPort[_motIdx]  |=  stepMask[_motIdx]

#define resetIsLo()      ((*resetPort[motorIdx] &   resetMask[motorIdx]) == 0)
#define setResetLo()       *resetPort[motorIdx] &= ~resetMask[motorIdx]
#define setResetHi()       *resetPort[motorIdx] |=  resetMask[motorIdx]

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
  uint16 mcuClock;   // period of clock in usecs  (applies to all motors)
};

#define mcuClockSettingIdx 9
#define NUM_SETTING_WORDS 10

extern const uint16 settingsInit[NUM_SETTING_WORDS];

union settingsUnion{
  uint16 reg[NUM_SETTING_WORDS];
  struct motorSettings val;
};
extern union settingsUnion mSet[NUM_MOTORS];

#ifdef B1
// show speed on DAC output pin A0 (ICP Data)
#define setDacToSpeed() DAC1CON1 = (ms->curSpeed >> 8);
#else
#define setDacToSpeed()  
#endif

// default startup values
//// must match settingsStruct
//#ifdef BM
//// assumes 1/40 mm per step
//// default is same for all motors
//const uint16 settingsInit[NUM_SETTING_WORDS] = {
//      5, // acceleration rate index,  0 is no acceleration
//   4000, // default speed is 100 mm
//   1200, // start/stop speed limit (30 mm/sec)
//  16000, // max pos is 400 mm
//   4000, // homing speed (100 mm/sec)
//     60, // homing back-up ms->speed (1.5 mm/sec)
//     40, // home offset distance: 1 mm
//      0, // home pos value, set cur pos to this after homing
//      0, // limit sw control (0 is normal)
//     50, // period of clock in usecs  (applies to all motors)
//};
//
//#else
//
//// assumes 1/50 mm per step
//// default is same for all motors
//const uint16 settingsInit[NUM_SETTING_WORDS] = {
//      5, // acceleration rate index,  0 is no acceleration
//   4000, // default speed is 100 mm
//   1200, // start/stop speed limit (30 mm/sec)
//  16000, // max pos is 400 mm
//   4000, // homing speed (100 mm/sec)
//     60, // homing back-up ms->speed (1.5 mm/sec)
//     40, // home offset distance: 1 mm
//      0, // home pos value, set cur pos to this after homing
//      0, // limit sw control (0 is normal)
//     50, // period of clock in usecs  (applies to all motors)
//};
//#endif /* B1 */

extern volatile uint16 *stepPort[NUM_MOTORS];
extern          uint16  stepMask[NUM_MOTORS];

#ifdef BM
volatile unsigned char *resetPort[NUM_MOTORS];
const uint8             resetMask[NUM_MOTORS];

extern volatile uint16 *faultPort[NUM_MOTORS];
extern const    uint16  faultMask[NUM_MOTORS];

extern volatile uint16 *limitPort[NUM_MOTORS];
extern const    uint16  limitMask[NUM_MOTORS];

#else

// -------- phases ----------
// Color        Bl Pi Ye Or  (red is +5))
//              {1, 1, 0, 0},
//              {0, 1, 1, 0},
//              {0, 0, 1, 1},
//              {1, 0, 0, 1}
extern uint16 motPhaseValue[NUM_MOTORS][4];
#endif

void motorInit(void);
void checkAll(void);
bool haveFault(void);        // bipolar only
bool limitSwOn(void);
void motorOn(void);
void processCommand(void);
void clockInterrupt(void);
void setNextStepTicks(uint16 ticks);

#endif	/* MOTOR_H */

