
#ifndef MOTOR_H
#define	MOTOR_H

#include <xc.h>
#include "types.h"
#include "pins.h"

#ifdef B1
#define NUM_MOTORS 1
#endif
#ifdef B5
#define NUM_MOTORS 5
#endif
#ifdef U3
#define NUM_MOTORS 3
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

#ifdef BM
#define setBiStepLo()           *stepPort[motorIdx] &= ~stepMask[motorIdx]
#define setBiStepHiInt(_motIdx) *stepPort[_motIdx]  |=  stepMask[_motIdx]
#define resetIsLo()      ((*resetPort[motorIdx] &   resetMask[motorIdx]) == 0)
#define setResetLo()       *resetPort[motorIdx] &= ~resetMask[motorIdx]
#define setResetHi()       *resetPort[motorIdx] |=  resetMask[motorIdx]
#else
#define clrUniPort()       (*mp = (*mp & ~mm));
#define setUniPort(_phase) (*mp = (*mp & ~mm) |                          \
                            motPhaseValue[motorIdx][_phase & 0x03]);
#define setUniPortInt(_motIdx, _phase)                                   \
  (*stepPort[_motIdx] = (*stepPort[_motIdx] & ~stepMask[_motIdx]) |      \
    motPhaseValue[_motIdx][_phase & 0x03]);
#endif

// constants loadable from command (all are 16 bits))
struct motorSettings {
  uint16 accelIdx;
  uint16 speed;
  uint16 jerk;
  uint16 maxPos;
  uint16 homingSpeed;
  uint16 homingBackUpSpeed;
  uint16 homeOfs;
  uint16 homePos;        // value to set cur pos after homing
  uint16 limitSwCtl;     // codes starting and switch direction and reversing
  uint16 backlashWid;    // backlash width of dead interval
  uint16 mcuClock;       // period of clock in usecs  (applies to all motors in mcu)
};

#define mcuClockSettingIdx 10
#define NUM_SETTING_WORDS  11

extern const uint16 settingsInit[NUM_SETTING_WORDS];

union settingsUnion{
  uint16 reg[NUM_SETTING_WORDS];
  struct motorSettings val;
};
extern union settingsUnion mSet[NUM_MOTORS];

#ifdef B1
extern volatile uint8 *limitPort[NUM_MOTORS];
extern const    uint8  limitMask[NUM_MOTORS];
#else
extern volatile uint16 *limitPort[NUM_MOTORS];
extern const    uint16  limitMask[NUM_MOTORS];
#endif

#ifdef BM

#ifdef B1
extern volatile uint8 *stepPort[NUM_MOTORS];
extern const    uint8  stepMask[NUM_MOTORS];

extern volatile uint8 *resetPort[NUM_MOTORS];
extern const    uint8  resetMask[NUM_MOTORS];

extern volatile uint8 *faultPort[NUM_MOTORS];
extern const    uint8  faultMask[NUM_MOTORS];

#else

extern volatile uint16 *stepPort[NUM_MOTORS];
extern const    uint16  stepMask[NUM_MOTORS];

extern volatile uint16  *resetPort[NUM_MOTORS];
extern const    uint16   resetMask[NUM_MOTORS];

extern volatile uint16 *faultPort[NUM_MOTORS];
extern const    uint16  faultMask[NUM_MOTORS];

#endif

#else /* not BM */

extern volatile uint16 *stepPort[NUM_MOTORS];
extern const    uint16  stepMask[NUM_MOTORS];

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

