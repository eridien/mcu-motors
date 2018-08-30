
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "i2c.h"
#include "state.h"
#include "motor.h"

#define CLK LATC0
#define SDA LATC1

volatile uint8 subTime;

volatile uint8 i2cRecvBytes[NUM_MOTORS][NUM_RECV_BYTES + 1];
volatile uint8 i2cRecvBytesPtr;
volatile uint8 i2cSendBytes[NUM_MOTORS][NUM_SEND_BYTES];
volatile uint8 i2cSendBytesPtr;
volatile bool  haveRecvData;
volatile bool  inPacket;

void i2cInit() {    
    SCL_TRIS = 1;
    SDA_TRIS = 1;
    
    SSP1CLKPPS = 0x10;           // RC0
    SSP1DATPPS = 0x11;           // RC1
    RC0PPS     = 0x15;           // SCL1
    RC1PPS     = 0x16;           // SDA1

    SSP1CON1bits.SSPM = 0x0e;          // slave mode, 7-bit, S & P ints enabled 
    SSP1MSK           = I2C_ADDR_MASK; // address mask, check all top 5 bits
    SSP1ADD           = I2C_ADDR;      // slave address (7-bit addr is 8 or 12)
    SSP1STATbits.SMP  = 0;             // slew-rate enabled
    SSP1STATbits.CKE  = 1;             // smb voltage levels
    SSP1CON2bits.SEN  = 1;             // enable clock stretching 
    SSP1CON3bits.AHEN = 0;             // no clock stretch for addr ack
    SSP1CON3bits.DHEN = 0;             // no clock stretch for data ack
    SSP1CON3bits.BOEN = 1;             // enable buffer overwrite check
           
    SSP1CON1bits.SSPEN = 1;            // Enable the serial port
    SSP1IF = 0;                        // nothing received yet
    SSP1IE = 1;                        // Enable ints
}

void setI2cCkSum() {
  // interrupts must be off when called
  uint8 *b = (uint8 *) &i2cSendBytes[motorIdx];
  b[0] = ms->stateByte;
  b[1] = ms->curPos >> 8;
  b[2] = ms->curPos & 0x00ff;
  b[3] = b[0] + b[1] + b[2];
}

// called from interrupt
void setI2cCkSumInt(uint8 motIdx) {
  struct motorState *p = &mState[motIdx];
  uint8 *b = (uint8 *) &i2cSendBytes[motIdx];
  b[0] = p->stateByte;
  b[1] = p->curPos >> 8;
  b[2] = p->curPos & 0x00ff;
  b[3] = b[0] + b[1] + b[2];
}

void checkI2c() {
  if(haveError() && 
      ((ms->stateByte & 0x0c) == (BUSY_MOVING << 2) || 
      ((ms->stateByte & 0x0c) == (BUSY_HOMING << 2)))) {
    // have error and motor moving, stop and reset
    softStopCommand(true);
  }
  if(ms->i2cCmdBusy) {
    if(!haveError()) {
      processMotorCmd();
    }
    // don't clear until done with data
    ms->i2cCmdBusy = false;
  }
}

volatile uint8 motIdxInPacket;

void i2cInterrupt(void) {
  // SSP1STATbits.S is set during entire packet
  if(SSP1STATbits.S && !inPacket) { 
    // received start bit, prepare for packet
    i2cRecvBytesPtr = 1; // skip over length byte
    i2cSendBytesPtr = 0;
    WCOL = 0;                   // clear WCOL
    volatile int x = SSP1BUF;   // clear SSPOV
    inPacket = true;
  }
  else if(SSP1STATbits.P) { 
    // received stop bit, on read tell loop that data is available
    inPacket = false;
    if (WCOL || SSPOV) {
      setErrorInt(motIdxInPacket, I2C_OVERFLOW_ERROR);
    }
    else {
      if(!SSP1STATbits.RW) {
        // total length of recv is stored in first byte
        i2cRecvBytes[motIdxInPacket][0] = i2cRecvBytesPtr-1;
        mState[motIdxInPacket].i2cCmdBusy = true;
      } else {
        // master just read status,  clear error
        ms->stateByte = (ms->stateByte & 0x8f);
        setI2cCkSumInt(motIdxInPacket);
      }
    }
  }
  else {
    if(!SSP1STATbits.DA) { 
      // received addr byte, extract motor number
      motIdxInPacket = (SSP1BUF & 0x0c) >> 1;
      if(SSP1STATbits.RW) {
        // send packet (i2c read from slave), load buffer for first byte
        SSP1BUF = i2cSendBytes[motIdxInPacket][i2cSendBytesPtr++]; // allways byte 0
      }
    }
    else {
      if(!SSP1STATbits.RW) {
        if(mState[motIdxInPacket].i2cCmdBusy) {
          // oops, last recv not handled yet by main loop
          setErrorInt(motIdxInPacket, CMD_NOT_DONE_ERROR);
        } else {
          // received byte (i2c write to slave)
          if(i2cRecvBytesPtr < NUM_RECV_BYTES + 1) 
            i2cRecvBytes[motIdxInPacket][i2cRecvBytesPtr++] = SSP1BUF;
        }
      }
      else {
        // sent byte (i2c read from slave), load buffer for next send
        SSP1BUF = i2cSendBytes[motIdxInPacket][i2cSendBytesPtr++];
      }
    }
  }
  CKP = 1; // end stretch
  volatile int z = SSP1BUF;  // clear BF  
}
