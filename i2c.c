
#include <xc.h>
#include "types.h"
#include "state.h"
#include "pins.h"
#include "i2c.h"
#include "state.h"
#include "motor.h"

volatile uint8 i2cRecvBytes[NUM_MOTORS][RECV_BUF_SIZE+1]; // added len byte
volatile uint8 i2cRecvBytesPtr;
volatile uint8 i2cSendBytes[NUM_SEND_BYTES];
volatile uint8 i2cSendBytesPtr;
volatile bool  inPacket;

#ifdef BM

void i2cInit() { 
    SSP1CON1bits.SSPM = 0x0e;          // slave mode, 7-bit, S & P ints enabled 
    SSP1MSK           = I2C_ADDR_MASK; // address mask, check top 5 bits
    SSP1ADD           = I2C_ADDR;      // slave address (7-bit addr)
    SSP1STATbits.SMP  = 0;             // slew-rate enabled
    SSP1STATbits.CKE  = 1;             // smb voltage levels
    SSP1CON2bits.SEN  = 1;             // enable clock stretching 
    SSP1CON3bits.AHEN = 0;             // no clock stretch before addr ack
    SSP1CON3bits.DHEN = 0;             // no clock stretch before data ack
    SSP1CON3bits.BOEN = 1;             // enable buffer overwrite check
    NotStretch        = 0;             // stretch clk of first start bit
    
#ifdef B1
    SSP1CLKPPS = 0x10;           // RC0
    SSP1DATPPS = 0x11;           // RC1
    RC0PPS     = 0x15;           // SCL1
    RC1PPS     = 0x16;           // SDA1
    
    SSP1IF = 0;                        // nothing received yet
    SSP1IE = 1;                        // Enable ints
#else
    // no pps
    _SSP1IF = 0;                       // nothing received yet
    _SSP1IE = 1;                       // Enable ints
#endif
    SSP1CON1bits.SSPEN = 1;            // Enable the serial port
}

#else

void i2cInit() { 
    SSP2CON1bits.SSPM = 0x0e;          // slave mode, 7-bit, S & P ints enabled 
    SSP2MSK           = I2C_ADDR_MASK; // address mask, check top 5 bits
    SSP2ADD           = I2C_ADDR;      // slave address (7-bit addr)
    SSP2STATbits.SMP  = 0;             // slew-rate enabled
    SSP2STATbits.CKE  = 1;             // smb voltage levels
    SSP2CON2bits.SEN  = 1;             // enable clock stretching 
    SSP2CON3bits.AHEN = 0;             // no clock stretch before addr ack
    SSP2CON3bits.DHEN = 0;             // no clock stretch before data ack
    SSP2CON3bits.BOEN = 1;             // enable buffer overwrite check
    NotStretch        = 0;             // stretch clk of first start bit
#endif
    
#ifdef BM
#ifdef B1
    SSP1CLKPPS = 0x10;           // RC0
    SSP1DATPPS = 0x11;           // RC1
    RC0PPS     = 0x15;           // SCL1
    RC1PPS     = 0x16;           // SDA1
    
    SSP1IF = 0;                        // nothing received yet
    SSP1IE = 1;                        // Enable ints
#else
    // no pps
    _SSP1IF = 0;                       // nothing received yet
    _SSP1IE = 1;                       // Enable ints
#endif
    SSP1CON1bits.SSPEN = 1;            // Enable the serial port
#else
    // no pps
    _SSP2IF = 0;                       // nothing received yet
    _SSP2IE = 1;                       // Enable ints
    SSP2CON1bits.SSPEN = 1;            // Enable the serial port
#endif
}

// all words are big-endian
void setSendBytesInt(uint8 motIdx) {
  struct motorState *p = &mState[motIdx];
  if(!ms->nextStateTestPos) {
    i2cSendBytes[0] = (p->stateByte | MCU_VERSION);
    i2cSendBytes[1] =  p->curPos >> 8;
    i2cSendBytes[2] =  p->curPos & 0x00ff;
  }
  else {
    ms->nextStateTestPos = false;
    i2cSendBytes[0]  = (TEST_POS_STATE | MCU_VERSION);
    i2cSendBytes[1]  = p->homeTestPos >> 8;
    i2cSendBytes[2]  = p->homeTestPos & 0x00ff;
  }
  i2cSendBytes[3] = i2cSendBytes[0] + i2cSendBytes[1] + i2cSendBytes[2];
}

volatile uint8 motIdxInPacket;

#ifdef B1
void i2cInterrupt(void) {
#endif
#ifdef B3
void __attribute__ ((interrupt,shadow,auto_psv)) _MSSP1Interrupt(void) {
  _SSP1IF = 0;
#endif
#ifdef U6
void __attribute__ ((interrupt,shadow,auto_psv)) _MSSP2Interrupt(void) {
  _SSP2IF = 0;
#endif
  // SSPxSTATbits.S is set during entire packet
  if(I2C_START_BIT && !inPacket) { 
    // received start bit, prepare for packet
    i2cRecvBytesPtr = 1;    // skip over length byte
    i2cSendBytesPtr = 1;    // first is hard-wired to zero
    I2C_WCOL = 0;           // clear WCOL
    dummy = I2C_BUF_BYTE;   // clear SSPOV
    inPacket = true;
  }
  else if(I2C_STOP_BIT) { 
    // received stop bit
    inPacket = false;
    if (I2C_WCOL || I2C_SSPOV) {
      setErrorInt(motIdxInPacket, I2C_OVERFLOW_ERROR);
    }
    else {
      if(!RdNotWrite) {
        // total length of recv is stored in first byte
        i2cRecvBytes[motIdxInPacket][0] = i2cRecvBytesPtr-1;
        // tell event loop that data is available
        mState[motIdxInPacket].haveCommand = true;
      } else {
        // master just read status,  clear any error
        setErrorInt(motIdxInPacket, CLEAR_ERROR);
      }
    }
  }
  else {
    if(!NotAddr) { 
      // received addr byte, extract motor number
      motIdxInPacket = (I2C_BUF_BYTE & 0x0e) >> 1;
      if(RdNotWrite) {
        // prepare all send data
        setSendBytesInt(motIdxInPacket);
        // send packet (i2c read from slave), load buffer for first byte
        I2C_BUF_BYTE = i2cSendBytes[0];
      }
    }
    else {
      if(!RdNotWrite) {
        // received byte (i2c write to slave)
        if(mState[motIdxInPacket].haveCommand) {
          // last command for this motor not handled yet by event loop
          setErrorInt(motIdxInPacket, CMD_NOT_DONE_ERROR);
        } else {
          if(i2cRecvBytesPtr < RECV_BUF_SIZE + 1) 
            i2cRecvBytes[motIdxInPacket][i2cRecvBytesPtr++] = I2C_BUF_BYTE;
        }
      }
      else {
        // sent byte (i2c read from slave), load buffer for next send
        I2C_BUF_BYTE = i2cSendBytes[i2cSendBytesPtr++];
      }
    }
  }
  // in packet: set ckp to end stretch after ack
  // stop bit:  clr ckp so next start bit will stretch
  NotStretch = !I2C_STOP_BIT; 
}
