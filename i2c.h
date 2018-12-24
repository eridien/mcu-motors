#ifndef I2C_H
#define	I2C_H

#include <xc.h>
#include "types.h"
#include "motor.h"

#define RECV_BUF_SIZE   (NUM_SETTING_WORDS*2 + 1) // + opcode byte
#define NUM_SEND_BYTES   3  //  state, posH, posL

#define I2C_ADDR_MASK 0xf0 // motor idx in d3-d1 (d2-d0 in real addr)

// motor is bottom 2 bits in addr
// addr is set based on ID input pin
#define I2C_ADDR_0   0x10  // real addr: 0x08+motor
#define I2C_ADDR_1   0x18  // real addr: 0x0c+motor

#define RdNotWrite SSP1STATbits.I2C_READ
#define NotAddr    SSP1STATbits.NOT_ADDRESS
#define NotStretch SSP1CON1bits.CKP

extern volatile uint8 i2cRecvBytes[NUM_MOTORS][RECV_BUF_SIZE + 1];
extern volatile uint8 i2cRecvBytesPtr;
extern volatile uint8 i2cSendBytes[NUM_SEND_BYTES];
extern volatile uint8 i2cSendBytesPtr;

void i2cInit(void);
void i2cInterrupt(void);

#endif	/* I2C_H */
