
#ifndef STATE_H
#define	STATE_H

#include "types.h"
#include "motor.h"

#define MCU_VERSION 0

extern volatile int dummy;

#define disableAllInts __builtin_disi(0x3FFF)
#define enableAllInts  __builtin_disi(0x0000) 

// Error codes 
#define MOTOR_FAULT_ERROR   0x10
#define OVERFLOW_ERROR      0x20
#define CMD_DATA_ERROR      0x30
#define STEP_NOT_DONE_ERROR 0x40
#define BOUNDS_ERROR        0x50
#define NO_SETTINGS         0x60
#define NOT_HOMED           0x70
#define CLEAR_ERROR         0xff // magic code to clear error

// state byte
#define ERR_CODE            0x70
#define AUX_RES_BIT         0x08 // do-d1 indicate what is in pos word
#define BUSY_BIT            0x04
#define MOTOR_ON_BIT        0x02
#define HOMED_BIT           0x01

struct motorState {
  uint8  stateByte;
  int16  targetPos;
  uint16 targetSpeed;
  bool   targetDir;
  bool   noBounds;
  int16  curPos;
  uint16 curSpeed;
  bool   curDir;
  int16  backlashPos; // neg is left of dead zone, >= backlashWid is right
  uint8  ustep;
  uint16 acceleration;
  bool   stepPending;
  bool   stepped;
  bool   stopping;
  bool   homing;
  uint8  homingState;
  bool   slowing;
  uint8  phase;  // bipolar: matches phase inside drv8825, unipolar: step phase
  uint16 nextStepTicks;
  uint16 lastStepTicks;
  bool   haveCommand;
  bool   resetAfterSoftStop;
  bool   nextStateSpecialVal; // flag to return homeTestPos on next read
  int16  homeTestPos;         // pos when limit sw closes
  volatile uint16 *limitPort; // set when settings loaded
  uint16 limitMask;           // set when settings loaded
  uint16 limActThres;         // convenience from limit sw ctl setting
  uint16 limActHyst;          // convenience from limit sw ctl setting
  uint16 limitCntTimeout;     // num steps since pin chg (after hyst)
  uint16 limitCountLo;        // num steps pin is low
  uint16 limitCountHi;        // num steps pin is high
};

extern struct motorState mState[NUM_MOTORS];

#define haveError() (errorIntCode || (ms->stateByte & ERR_CODE))

extern volatile uint8 errorIntMot;
extern volatile uint8 errorIntCode;

void  setStateBit(uint8 mask, uint8 set);
void  setError(uint8 err);
void  setErrorInt(uint8 motorIdx, uint8 err);
void  clrErrorInt(uint8 motorIdx);

#endif	/* STATE_H */

