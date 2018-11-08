  
#ifndef PINS_H
#define	PINS_H

#define I2C_START_BIT SSP1STATbits.S
#define I2C_STOP_BIT  SSP1STATbits.P
#define I2C_BUF_BYTE  SSP1BUF
#define I2C_SSPIF     SSP1IF

#ifdef B1
#define I2C_WCOL      WCOL1
#define I2C_SSPOV     SSPOV1
#else
#define I2C_WCOL      SSP1CON1bits.WCOL
#define I2C_SSPOV     SSP1CON1bits.SSPOV
#endif

#ifdef B1
#define dirTRIS   TRISA2
#define ms1TRIS   TRISC2
#define ms2TRIS   TRISB4
#define ms3TRIS   TRISB5

#define dirLAT    LATA2
#define ms1LAT    LATC2
#define ms2LAT    LATB4
#define ms3LAT    LATB5

#define resetTRIS TRISB6
#define resetLAT  LATB6

#define adcTRIS   TRISB7

#define resetPORT  PORTB
#define resetMASK  0x40

#define stepTRIS  TRISC5
#define stepLAT   LATC5
#define stepPORT  PORTC
#define stepMASK  0x20

#define faultTRIS TRISA5
#define faultPORT PORTA
#define faultMASK  0x20

#define limitTRIS TRISA4 
#define limitPORT PORTA
#define limitMASK 0x10

#define tp1TRIS TRISC4
#define tp2TRIS TRISC3
#define tp3TRIS TRISC6
#define tp4TRIS TRISA4

#define tp1LAT  LATC4
#define tp2LAT  LATC3
#define tp3LAT  LATC6
#define tp4LAT  LATA4

#define dbg10   tp1LAT = 0;
#define dbg11   tp1LAT = 1;
#define dbg20   tp2LAT = 0;
#define dbg21   tp2LAT = 1;
#define dbg30   tp3LAT = 0;
#define dbg31   tp3LAT = 1;
#define dbg40   tp4LAT = 0;
#define dbg41   tp4LAT = 1;

#endif /* B1 */

#ifdef B5
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
#define stepZBIT   0x0002

#define faultRTRIS  _TRISB4
#define faultETRIS  _TRISA4
#define faultXTRIS  _TRISB5
#define faultFTRIS  _TRISB12
#define faultZTRIS  _TRISA3
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

#endif	/* B5 */

#ifdef U3
#define motAPORT   PORTA  // tool A
#define motBPORT   PORTB  // tool B
#define motPPORT   PORTB  // paster

#define motATRIS   TRISA  // tool A
#define motBTRIS   TRISB  // tool B
#define motPTRIS   TRISB  // paster

#define motAOFS    0
#define motBOFS    0
#define motPOFS    8

#define limitATRIS  _TRISA6
#define limitBTRIS  _TRISA7
#define limitAPORT  PORTA
#define limitBPORT  PORTA
#define limitABIT   0x0040
#define limitBBIT   0x0080

#define led1TRIS   _TRISB12
#define led2TRIS   _TRISB13
#define led3TRIS   _TRISB14
#define led4TRIS   _TRISB15
 
#define led1LAT    _LATB12
#define led2LAT    _LATB13
#define led3LAT    _LATB14
#define led4LAT    _LATB15

#define tp1TRIS    _TRISA4
#define tp2TRIS    led1TRIS
#define tp3TRIS    led2TRIS
#define tp4TRIS    led3TRIS

#define tp1LAT     _LATA4
#define tp2LAT     led1LAT
#define tp3LAT     led2LAT
#define tp4LAT     led3LAT

#define dbg10 tp1LAT = 0;
#define dbg11 tp1LAT = 1;
#define dbg20 tp2LAT = 0;
#define dbg21 tp2LAT = 1;
#define dbg30 tp3LAT = 0;
#define dbg31 tp3LAT = 1;
#define dbg40 tp4LAT = 0;
#define dbg41 tp4LAT = 1;
#endif /* U3 */

#endif	/* PINS_H */

