
#ifndef MOTOR_H
#define	MOTOR_H

#include <xc.h>
#include "types.h"
#include "pins.h"

#define NUM_MOTORS 8

// global for use in main chk loop
extern volatile uint16 *mp;
extern uint8  motorIdx;
extern struct motorState      *ms;
extern struct motorSettings   *sv;
extern uint8                   mm; // motor mask (0xf0 or 0x0f or step bit)

#define setBiStepLo()           *stepPort[motorIdx] &= ~stepMask[motorIdx]
#define setBiStepHiInt(_motIdx) *stepPort[_motIdx]  |=  stepMask[_motIdx]
#define resetIsLo()      ((*resetPort[motorIdx] &   resetMask[motorIdx]) == 0)
#define setResetLo()       *resetPort[motorIdx] &= ~resetMask[motorIdx]
#define setResetHi()       *resetPort[motorIdx] |=  resetMask[motorIdx]

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
  uint16 backlashSpeed;  // backlash speed over dead interval
  uint16 mcuClock;       // period of clock in usecs  (applies to all motors in mcu)
};

#define mcuClockSettingIdx 11
#define NUM_SETTING_WORDS  12

extern const uint16 settingsInit[NUM_SETTING_WORDS];

union settingsUnion{
  uint16 reg[NUM_SETTING_WORDS];
  struct motorSettings val;
};
extern union settingsUnion mSet[NUM_MOTORS];

extern volatile uint16 *stepPort[NUM_MOTORS];
extern const    uint16  stepMask[NUM_MOTORS];

extern volatile uint16  *resetPort[NUM_MOTORS];
extern const    uint16   resetMask[NUM_MOTORS];

extern volatile uint16 *faultPort[NUM_MOTORS];
extern const    uint16  faultMask[NUM_MOTORS];

#endif

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

