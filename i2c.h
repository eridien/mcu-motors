#ifndef I2C_H
#define	I2C_H

#include <xc.h>
#include "types.h"
#include "motor.h"

// move/home commands start immediately even when moving
// all position and distance is in steps
// all speed is in steps/sec (except speed-move cmd)
// minimum system speed is 0.1 mm/sec => 8 1/80 mm
// all accelleration is in 1/80 mm/sec/sec
// homing on unit with no limit switch just sets current position to 150 mm

// steps are in 1/8 ustep (bipolar) or one phase (unipolar)
//    for bipolar:
//       steps/rev:        1600
//       dist/rev:           40 mm
//       max distance:      800 mm
//       max step count: 32,000
//
//    for unipolar:
//       steps/rev:        2048
//       dist/rev:           40 mm
//       max distance:      625 mm
//       max step count: 32,000
//
// writes ...
//   1aaa aaaa  goto command, top 7 bits of goto addr
//      aaaa aaaa followed by bottom 8 bits
//   01ss ssss (speed-move cmd) set max speed = s*256 steps/sec, move to a
//     0aaa aaaa top 7 bits of move addr
//     aaaa aaaa bottom 8 bits
//   0001 0001  start homing
//   0001 0010  soft stop, deccelerates, no reset
//   0001 0011  soft stop, deccelerates first, then reset
//   0001 0100  hard stop (immediate reset)
//   0001 0101  motor on (reset off)
//   0001 0110  set curpos to home pos value setting
//   0001 0111  set regs, 16-bit values, set only when idle
//      max speed
//      no-accelleration speed limit (and start speed)
//      accelleration rate
//      homing speed
//      homing back-up speed
//      home offset distance
//      home pos value
//
// Error codes 
//      1: fault
//      2: i2c buffer overflow
//      3: i2c cksum error
//      4: command not done
//      5: unexpected limit switch
//      6: move cmd when not homed
//      7: bad command data (first byte invalid or length wrong)
//
// state bytes
//   vccc eboz  state byte
//      v: version (1-bit)
//    ccc: error code (see above)
//      e: error bit
//      b: busy state
//      o: motor on (not in reset)
//      z: at home
//   aaaa aaaa  current position, top 8 bits of signed 16-bit word
//   aaaa aaaa  followed by bottom 8 bits
//   hhhh hhhh  home position test val, top 8 bits of signed 16-bit word
//   hhhh hhhh  home position test val, bottom 8 bits
//   cccc cccc  8-bit cksum, sum of first 5 bytes


#define NUM_RECV_BYTES NUM_SETTING_WORDS*2+4
#define NUM_SEND_BYTES   4  //  state, posH, posL, cksum

#define I2C_ADDR_MASK 0xf0 // motor idx in d3-d1 (d2-d0 in real addr)

#ifdef B1
#define I2C_ADDR      0x10  // real addr:0x08, ext box mcu, motor is always 0
#endif
#ifdef B3
#define I2C_ADDR      0x20  // real addr:0x10, head mcu for bipolar motors
#endif
#ifdef U6
#define I2C_ADDR      0x30  // real addr:0x18, head mcu for unipolar motors
#endif

extern volatile uint8 i2cRecvBytes[NUM_MOTORS][NUM_RECV_BYTES + 1];
extern volatile uint8 i2cRecvBytesPtr;
extern volatile uint8 i2cSendBytes[NUM_MOTORS][NUM_SEND_BYTES];
extern volatile uint8 i2cSendBytesPtr;

void i2cInit(void);
void checkI2c(void);
void setI2cCkSum(void);
void setI2cCkSumInt(uint8 motIdx);
void i2cInterrupt(void);

#endif	/* I2C_H */
