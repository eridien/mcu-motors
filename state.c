

#include "types.h"
#include "state.h"
#include "i2c.h"
#include "motor.h"
#include "stop.h"

bool nextStateTestPos;

void setStateBit(uint8 mask, uint8 set){
  GIE=0;
  ms->stateByte = (ms->stateByte & ~mask) | (set ? mask : 0);
  GIE=1;
}

void setError(uint8 err) {
  if(err == CLEAR_ERROR) {
    GIE=0;
    ms->stateByte = ms->stateByte & 0x07;
    GIE=1;
    I2C_WCOL = 0;                    // clear WCOL
    volatile int x = I2C_BUF_BYTE;   // clear SSPOV
  }
  else {
    // an error in one motor sets error bits in all motors
    // but error code is only in motor that caused error
    for(uint8 motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
      mState[motIdx].stateByte = ERROR_BIT;
    }
    ms->stateByte = (err | ERROR_BIT);
    // reset all motors
    resetMotor(true);
  }
}

volatile bool errorIntMot;
volatile bool errorIntCode;

// used in interrupt
void setErrorInt(uint8 motIdx, uint8 err) {
  errorIntMot  = motIdx;
  errorIntCode = err;
}
