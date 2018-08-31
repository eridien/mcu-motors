

#include "types.h"
#include "state.h"
#include "i2c.h"
#include "motor.h"

void setStateBit(uint8 mask, uint8 set){
  ms->stateByte = (ms->stateByte & ~mask) | (set ? mask : 0);
}

void setError(uint8 err) {
  if(err == CLEAR_ERROR) {
    ms->stateByte = ms->stateByte & 0x07;
  }
  else {
    ms->stateByte = err;
#ifdef BM
    // an error in one bipolar motor sets error bits in all motors
    // but error code is only in motor that caused error
    for(uint8 motIdx = 0; motIdx < NUM_MOTORS; motIdx++) {
      mState[motIdx].stateByte = (mState[motIdx].stateByte | ERROR_BIT);
    }
    resetAllBiMotors();
#else
    // an error in a uni motor only affects that motor
    setStateBit(ERROR_BIT, true);
    resetUniMotor();
#endif
  }
}

volatile bool errorIntMot;
volatile bool errorIntCode;

// used in interrupt
void setErrorInt(uint8 motIdx, uint8 err) {
  errorIntMot  = motIdx;
  errorIntCode = err;
}
