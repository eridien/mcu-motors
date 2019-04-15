

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
    ms->stateByte = ms->stateByte & ~ERR_CODE;
    enableAllInts;
    dummy = I2C_BUF_BYTE;   // clear SSPOV
  }
  else {
    ms->stateByte = err;
    resetMotor();
  }
}

volatile uint8 errorIntMot;
volatile uint8 errorIntCode;

// used in interrupt
void setErrorInt(uint8 motIdx, uint8 err) {
  errorIntMot  = motIdx;
  errorIntCode = err;
}
