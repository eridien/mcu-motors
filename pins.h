  
#ifndef PINS_H
#define	PINS_H

#define I2C_START_BIT SSP1STATbits.S
#define I2C_STOP_BIT  SSP1STATbits.P
#define I2C_BUF_BYTE  SSP1BUF
#define I2C_SSPIF     SSP1IF

#define IDTRIS    _TRISB1   // mcu ID, sets i2c base addr
#define IDPORT    _RB1     // 0: mcuA, 1: mcuB, only valid at startup

#define dirTRIS   _TRISA6
#define ms1TRIS   _TRISA7
#define ms2TRIS   _TRISB7

#define dirLAT    _LATA6
#define ms1LAT    _LATA7
#define ms2LAT    _LATB7

#define resetATRIS _TRISB15
#define resetBTRIS _TRISB14
#define resetCTRIS _TRISB13
#define resetDTRIS _TRISB12

#define resetALAT  _LATB15
#define resetBLAT  _LATB14
#define resetCLAT  _LATB13
#define resetDLAT  _LATB12

#define resetAPORT  PORTB
#define resetBPORT  PORTB
#define resetCPORT  PORTB
#define resetDPORT  PORTB

#define resetABIT   0x8000
#define resetBBIT   0x4000
#define resetCBIT   0x2000
#define resetDBIT   0x1000

#define stepATRIS  _TRISB1
#define stepBTRIS  _TRISB2
#define stepCTRIS  _TRISB3
#define stepDTRIS  _TRISA2

#define stepALAT   _LATB1
#define stepBLAT   _LATB2
#define stepCLAT   _LATB3
#define stepDLAT   _LATA2

#define stepAPORT  PORTB
#define stepBPORT  PORTB
#define stepCPORT  PORTB
#define stepDPORT  PORTA

#define stepABIT   0x0002
#define stepBBIT   0x0004
#define stepCBIT   0x0008
#define stepDBIT   0x0004

#define faultATRIS  _TRISA3
#define faultBTRIS  _TRISB4
#define faultCTRIS  _TRISA4
#define faultDTRIS  _TRISB5

#define faultALAT  _LATA3
#define faultBLAT  _LATB4
#define faultCLAT  _LATA4
#define faultDLAT  _LATB5

#define faultAPORT  PORTA
#define faultBPORT  PORTB
#define faultCPORT  PORTA
#define faultDPORT  PORTB

#define faultABIT   0x0008
#define faultBBIT   0x0010
#define faultCBIT   0x0010
#define faultDBIT   0x0020

#define limALAT   _LATA0
#define limBLAT   _LATA1
#define limCLAT   _LATB0
#define limDLAT   _LATB6

#define limATRIS  _TRISA0
#define limBTRIS  _TRISA1
#define limCTRIS  _TRISB0
#define limDTRIS  _TRISB6

#define limAPORT  PORTA
#define limBPORT  PORTB
#define limCPORT  PORTB
#define limDPORT  PORTA

#define limABIT   0x0001
#define limBBIT   0x0001
#define limCBIT   0x0040
#define limDBIT   0x0002

#ifdef DEBUG
#define tp1TRIS faultATRIS
#define tp2TRIS faultBTRIS
#define tp3TRIS faultCTRIS
#define tp4TRIS faultDTRIS

#define tp1LAT  faultALAT
#define tp2LAT  faultBLAT
#define tp3LAT  faultCLAT
#define tp4LAT  faultDLAT

#define dbg10 tp1LAT = 0;
#define dbg11 tp1LAT = 1;
#define dbg20 tp2LAT = 0;
#define dbg21 tp2LAT = 1;
#define dbg30 tp3LAT = 0;
#define dbg31 tp3LAT = 1;
#define dbg40 tp4LAT = 0;
#define dbg41 tp4LAT = 1;
#endif

#endif	/* PINS_H */

