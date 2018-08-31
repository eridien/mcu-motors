

#include "types.h"
#include "state.h"
#include "i2c.h"
#include "motor.h"

// do not use with interrupts off
void setCurState(uint8 newState) {
  // v bit (version is zero)
  GIE = 0;
  ms->stateByte = newState;
  setI2cCkSum();
  GIE = 1;
}

void setStateBit(uint8 mask, uint8 set){
  setCurState((ms->stateByte & ~mask) | (set ? mask : 0));
}

// do not use from interrupt
void setError(uint8 err) {
  setCurState(err);
  for(uint8 motIdx = 0; motIdx < NUM_MOTORS; motIdx++)
    setStateBit(ERROR_BIT, true);
  resetMotor();
}

volatile bool errorIntMot;
volatile bool errorIntCode;

// use from interrupt
void setErrorInt(uint8 motIdx, uint8 err) {
  errorIntMot  = motIdx;
  errorIntCode = err;
}
