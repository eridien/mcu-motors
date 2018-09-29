

#include "types.h"
#include "state.h"
#include "i2c.h"
#include "motor.h"
#include "stop.h"

volatile int dummy = 0; // used for reading register and ignoring value

struct motorState mState[NUM_MOTORS];

void setStateBit(uint8 mask, uint8 set){
  disableAllInts;
  ms->stateByte = (ms->stateByte & ~mask) | (set ? mask : 0);
  enableAllInts;
}

void setError(uint8 err) {
  if(err == CLEAR_ERROR) {
    disableAllInts;
    ms->stateByte = ms->stateByte & (BUSY_BIT | MOTOR_ON_BIT | HOMED_BIT);
    enableAllInts;
    I2C_WCOL = 0;           // clear WCOL
    dummy = I2C_BUF_BYTE;   // clear SSPOV
  }
  else {
#ifdef DEBUG
    disableAllInts;
    while(true);
#endif
    // an error in one motor sets error bit in all motors
    // but error code is only in motor that reported error
    uint8 motIdx;
    for(motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
      mState[motIdx].stateByte = ERROR_BIT;
    }
    ms->stateByte = (err | ERROR_BIT);
    // reset all motors
    resetMotor(true);
  }
}

volatile uint8 errorIntMot;
volatile uint8 errorIntCode;

// used in interrupt
void setErrorInt(uint8 motIdx, uint8 err) {
  errorIntMot  = motIdx;
  errorIntCode = err;
}
