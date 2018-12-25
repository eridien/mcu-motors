
#ifndef MOTOR_H
#define	MOTOR_H

#include <xc.h>
#include "types.h"
#include "pins.h"

#define NUM_MOTORS 4

#define DEF_MCU_CLK 30

// globals for use in main event loop
extern volatile uint16        *mp;
extern uint8                   motorIdx;
extern struct motorState      *ms;
extern struct motorSettings   *sv;

#define setBiStepLo()           *stepPort[motorIdx] &= ~stepMask[motorIdx]
#define setBiStepHiInt(_motIdx) *stepPort[_motIdx]  |=  stepMask[_motIdx]
#define resetIsLo()          ((*resetPort[motorIdx] &   resetMask[motorIdx]) == 0)
#define setResetLo()           *resetPort[motorIdx] &= ~resetMask[motorIdx]
#define setResetHi()           *resetPort[motorIdx] |=  resetMask[motorIdx]

extern bool haveSettings[NUM_MOTORS];

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
  uint16 limitSwCtl;     // limit sw assignment
  uint16 backlashWid;    // backlash dead width in steps
  uint16 mcuClock;       // period of clock in usecs  (applies to all motors in mcu)
};

#define mcuClockSettingIdx 10
#define NUM_SETTING_WORDS  11

union settingsUnion{
  uint16 reg[NUM_SETTING_WORDS];
  struct motorSettings val;
};
extern union settingsUnion mSet[NUM_MOTORS];

extern volatile uint16 *stepPort[NUM_MOTORS];
extern const    uint16  stepMask[NUM_MOTORS];

extern volatile uint16 *resetPort[NUM_MOTORS];
extern const    uint16  resetMask[NUM_MOTORS];

extern volatile uint16 *faultPort[NUM_MOTORS];
extern const    uint16  faultMask[NUM_MOTORS];

extern volatile uint16 *limitPort[NUM_MOTORS]; // set when settings loaded
extern          uint16  limitMask[NUM_MOTORS];

void motorInit(void);
void checkAll(void);
bool haveFault(void);
bool limitSwOn(void);
void motorOn(void);
void processCommand(void);
void clockInterrupt(void);
void setNextStepTicks(uint16 ticks);

#endif	/* MOTOR_H */

