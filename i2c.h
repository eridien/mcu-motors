#ifndef I2C_H
#define	I2C_H

#include <xc.h>
#include "types.h"
#include "motor.h"

#define RECV_BUF_SIZE   (NUM_SETTING_WORDS*2 + 1) // + opcode byte
#define NUM_SEND_BYTES   4  //  state, posH, posL, cksum

#define I2C_ADDR_MASK 0xf0 // motor idx in d3-d1 (d2-d0 in real addr)

// motor (or leds) is bottom 3 bits in addr
#ifdef B1
#define I2C_ADDR      0x10  // real addr:0x08, box mcu (B1), motor always 0
#endif
#ifdef B4
#define I2C_ADDR      0x20  // real addr:0x10, head mcu (B4) for bipolar motors
#endif
#ifdef U5
#define I2C_ADDR      0x30  // real addr:0x18, head mcu (U5) for unipolar motors
#endif


#ifdef B1
    #define RdNotWrite SSP1STATbits.I2C_READ
    #define NotAddr    SSP1STATbits.DA
    #define NotStretch SSP1CON1bits.CKP1
#endif
#ifdef B4
    #define RdNotWrite SSP1STATbits.I2C_READ
    #define NotAddr    SSP1STATbits.NOT_ADDRESS
    #define NotStretch SSP1CON1bits.CKP
#endif
#ifdef U5
    #define RdNotWrite SSP2STATbits.I2C_READ
    #define NotAddr    SSP2STATbits.NOT_ADDRESS
    #define NotStretch SSP2CON1bits.CKP
#endif

extern volatile uint8 i2cRecvBytes[NUM_MOTORS][RECV_BUF_SIZE + 1];
extern volatile uint8 i2cRecvBytesPtr;
extern volatile uint8 i2cSendBytes[NUM_SEND_BYTES];
extern volatile uint8 i2cSendBytesPtr;

void i2cInit(void);
void i2cInterrupt(void);

#endif	/* I2C_H */
