
#ifndef STATE_H
#define	STATE_H

#include "types.h"
#include "motor.h"

#define MCU_VERSION 0

// when returning test pos instead of cur pos
// state byte will have this magic value which can't happen normally
#define TEST_POS_STATE      0x04

// Error codes 
#define MOTOR_FAULT_ERROR   0x10
#define I2C_OVERFLOW_ERROR  0x20
#define CMD_DATA_ERROR      0x30
#define CMD_NOT_DONE_ERROR  0x40
#define STEP_NOT_DONE_ERROR 0x50
#define MOTOR_LIMIT_ERROR   0x60
#define NOT_HOMED_ERROR     0x70
#define CLEAR_ERROR         0xff // magic code to clear error

// state bits
#define ERROR_BIT           0x08
#define BUSY_BIT            0x04
#define MOTOR_ON_BIT        0x02
#define HOMED_BIT           0x01

struct motorState {
  uint8  stateByte;
  bool   homing;
  uint8  homingState;
  bool   stopping;
  int16  curPos;
  uint16 curSpeed;
  bool   curDir;
  int16  targetPos;
  uint16 targetSpeed;
  bool   targetDir;
  uint8  ustep;  // bipolar only
  uint8  phase;  // unipolar only
  bool   stepPending;
  bool   stepped;
  uint16 nextStepTicks;
  uint16 lastStepTicks;
  bool   haveCommand;
  bool   resetAfterSoftStop;
  bool   nextStateTestPos;
  int16  homeTestPos;
  bool   homeReversed;
} mState[NUM_MOTORS];


#define haveError() (errorIntCode || (ms->stateByte & ERROR_BIT))

extern volatile bool errorIntMot;
extern volatile bool errorIntCode;

void  setStateBit(uint8 mask, uint8 set);
void  setError(uint8 err);
void  setErrorInt(uint8 motorIdx, uint8 err);
void  clrErrorInt(uint8 motorIdx);

#endif	/* STATE_H */

