  
#ifndef PINS_H
#define	PINS_H

#define I2C_START_BIT SSP1STATbits.S
#define I2C_STOP_BIT  SSP1STATbits.P
#define I2C_BUF_BYTE  SSP1BUF
#define I2C_SSPIF     SSP1IF

#define dirTRIS   _TRISA6
#define ms1TRIS   _TRISA7
#define ms2TRIS   _TRISB7
#define ms3TRIS   _TRISB6

#define dirLAT    _LATA6
#define ms1LAT    _LATA7
#define ms2LAT    _LATB7
#define ms3LAT    _LATB6

#define resetTRIS _TRISB15
#define resetLAT  _LATB15
#define resetPORT  PORTB
#define resetBIT   0x8000

#define stepRTRIS  _TRISB1
#define stepETRIS  _TRISB2
#define stepXTRIS  _TRISB3
#define stepFTRIS  _TRISB13
#define stepZTRIS  _TRISA2
#define stepRLAT   _LATB1
#define stepELAT   _LATB2
#define stepXLAT   _LATB3
#define stepFLAT   _LATB13
#define stepZLAT   _LATA2
#define stepRPORT  PORTB
#define stepRBIT   0x0002
#define stepEPORT  PORTB
#define stepEBIT   0x0004
#define stepXPORT  PORTB
#define stepXBIT   0x0008
#define stepFPORT  PORTB
#define stepFBIT   0x2000
#define stepZPORT  PORTA
#define stepZBIT   0x0004

#define faultRTRIS  _TRISB4
#define faultETRIS  _TRISA4
#define faultXTRIS  _TRISB5
#define faultFTRIS  _TRISB12
#define faultZTRIS  _TRISA3
#define faultRLAT  _LATB4
#define faultELAT  _LATA4
#define faultXLAT  _LATB5
#define faultFLAT  _LATB12
#define faultZLAT  _LATA3
#define faultRPORT  PORTB
#define faultRBIT   0x0010
#define faultEPORT  PORTA
#define faultEBIT   0x0010
#define faultXPORT  PORTB
#define faultXBIT   0x0020
#define faultFPORT  PORTB
#define faultFBIT   0x1000
#define faultZPORT  PORTA
#define faultZBIT   0x0008

#define limitRTRIS  _TRISA0
#define limitXTRIS  _TRISA1
#define limitFTRIS  _TRISB14
#define limitZTRIS  _TRISB0
#define limitRPORT  PORTA
#define limitRBIT   0x0001
#define limitXPORT  PORTA
#define limitXBIT   0x0002
#define limitFPORT  PORTB
#define limitFBIT   0x4000
#define limitZPORT  PORTB
#define limitZBIT   0x0001

#ifdef DEBUG
#define tp1TRIS     faultRTRIS
#define tp2TRIS     faultETRIS
#define tp3TRIS     faultXTRIS

#define tp1LAT      faultRLAT
#define tp2LAT      faultELAT
#define tp3LAT      faultXLAT

#define dbg10 tp1LAT = 0;
#define dbg11 tp1LAT = 1;
#define dbg20 tp2LAT = 0;
#define dbg21 tp2LAT = 1;
#define dbg30 tp3LAT = 0;
#define dbg31 tp3LAT = 1;
#endif

#endif	/* PINS_H */

