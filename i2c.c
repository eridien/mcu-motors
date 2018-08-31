
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
volatile uint8 i2cSendBytes[NUM_SEND_BYTES];
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

void updateSendBytesInt(uint8 motIdx) {
  struct motorState *p = &mState[motIdx];
  i2cSendBytes[0] = p->stateByte;
  i2cSendBytes[1] = p->curPos >> 8;
  i2cSendBytes[2] = p->curPos & 0x00ff;
  i2cSendBytes[3] = p->homeTestPos >> 8;
  i2cSendBytes[4] = p->homeTestPos & 0x00ff;
  i2cSendBytes[5] = i2cSendBytes[0] + i2cSendBytes[1] + 
                    i2cSendBytes[2] + i2cSendBytes[3] + i2cSendBytes[4];
}

void checkI2c() {
  if(haveError() && (ms->stateByte & BUSY_BIT)) {
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
    WCOL1 = 0;                   // clear WCOL
    volatile int x = SSP1BUF;   // clear SSPOV
    inPacket = true;
  }
  else if(SSP1STATbits.P) { 
    // received stop bit, on read tell loop that data is available
    inPacket = false;
    if (WCOL1 || SSPOV1) {
      setErrorInt(motIdxInPacket, I2C_OVERFLOW_ERROR);
    }
    else {
      if(!SSP1STATbits.RW) {
        // total length of recv is stored in first byte
        i2cRecvBytes[motIdxInPacket][0] = i2cRecvBytesPtr-1;
        mState[motIdxInPacket].i2cCmdBusy = true;
      } else {
        // master just read status,  clear error
        setErrorInt(motIdxInPacket, 0x55); // magic err code means clear
      }
    }
  }
  else {
    if(!SSP1STATbits.DA) { 
      // received addr byte, extract motor number
      motIdxInPacket = (SSP1BUF & 0x0e) >> 1;
      if(SSP1STATbits.RW) {
        // prepare all send data
        updateSendBytesInt(motIdxInPacket);
        // send packet (i2c read from slave), load buffer for first byte
        SSP1BUF = i2cSendBytes[i2cSendBytesPtr++]; // allways byte 0
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
        SSP1BUF = i2cSendBytes[i2cSendBytesPtr++];
      }
    }
  }
  CKP1 = 1; // end stretch
  volatile int z = SSP1BUF;  // clear BF  
}
