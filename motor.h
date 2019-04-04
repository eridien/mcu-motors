
#ifndef MOTOR_H
#define	MOTOR_H

#include <xc.h>
#include "types.h"
#include "pins.h"

#define NUM_MOTORS 4

#define DEF_MCU_CLK 30

// globals for use in main event loop
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
   int16 minPos;
   int16 maxPos;
  uint16 homingDir;
  uint16 homingSpeed;
  uint16 homingBackUpSpeed;
   int16 homeOfs;
   int16 homePos;        // value to set cur pos after homing
  uint16 limitSwCtl;     // limit sw assignment
  uint16 backlashWid;    // backlash dead width in steps
  uint16 maxUstep;       // maximum ustep (0 for 5-wire unipolar stepper, else 3)
  uint16 mcuClock;       // period of clock in usecs  (applies to all motors in mcu)
};

#define mcuClockSettingIdx 13
#define NUM_SETTING_WORDS  14

#define LIM_ENBL_MASK        0x8000
#define LIM_ACT_TIMEOUT_MASK 0x0f00
#define LIM_ACT_HYST_MASK    0x00f0
#define LIM_POL_MASK         0x0001

#define LIM_ENBL_OFS         15
#define LIM_ACT_TIMEOUT_OFS  8
#define LIM_ACT_HYST_OFS     4
#define LIM_POL_OFS          0

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

void motorInit(void);
void checkAll(void);
bool haveFault(void);
bool limitSwOn(void);
void motorOn(void);
void processCommand(void);
void clockInterrupt(void);
void setNextStepTicks(uint16 ticks);

#endif	/* MOTOR_H */

