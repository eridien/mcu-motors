  
#ifndef PINS_H
#define	PINS_H

#ifdef BM
#define I2C_START_BIT SSP1STATbits.S
#define I2C_STOP_BIT  SSP1STATbits.P
#define I2C_BUF_BYTE  SSP1BUF
#define I2C_SSPIF     SSP1IF
#endif /* BM */

#ifdef B1
#define I2C_WCOL  WCOL1
#define I2C_SSPOV     SSPOV1

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

#ifdef B4
#define I2C_WCOL  SSP1CON1bits.WCOL
#define I2C_SSPOV SSP1CON1bits.SSPOV

#define dirTRIS   _TRISA6
#define ms1TRIS   _TRISA7
#define ms2TRIS   _TRISB7
#define ms3TRIS   _TRISB6

#define dirLAT    _LATA6
#define ms1LAT    _LATA7
#define ms2LAT    _LATB7
#define ms3LAT    _LATB6

#define resetRTRIS _TRISB15
#define resetETRIS _TRISB14
#define resetXTRIS _TRISB13

#define resetRLAT _LATB15
#define resetELAT _LATB14
#define resetXLAT _LATB13

#define resetRPORT  PORTB
#define resetRBIT   0x8000
#define resetEPORT  PORTB
#define resetEBIT   0x4000
#define resetXPORT  PORTB
#define resetXBIT   0x2000

#define stepRTRIS  _TRISB1
#define stepETRIS  _TRISB2
#define stepXTRIS  _TRISB3

#define stepRLAT   _LATB1
#define stepELAT   _LATB2
#define stepXLAT   _LATB3

#define stepRPORT  PORTB
#define stepRBIT   0x02
#define stepEPORT  PORTB
#define stepEBIT   0x04
#define stepXPORT  PORTB
#define stepXBIT   0x08

#define faultRTRIS  _TRISB4
#define faultETRIS  _TRISA4
#define faultXTRIS  _TRISB5

#define faultRPORT  PORTB
#define faultRBIT   0x10
#define faultEPORT  PORTA
#define faultEBIT   0x10
#define faultXPORT  PORTB
#define faultXBIT   0x20

#define limitRTRIS  _TRISA0
#define limitXTRIS  _TRISA1

#define limitRPORT  PORTA
#define limitRBIT   0x01
#define limitXPORT  PORTA
#define limitXBIT   0x02

#define tp1TRIS   _TRISB0
#define tp2TRIS   _TRISB12
#define tp3TRIS   _TRISA2
#define tp4TRIS   _TRISA3

#define tp1LAT     _LATB0
#define tp2LAT     _LATB12
#define tp3LAT     _LATA2
#define tp4LAT     _LATA3

// only tp2 works in B4 ???
#define dbg10 tp1LAT = 0;
#define dbg11 tp1LAT = 1;
#define dbg20 tp2LAT = 0;
#define dbg21 tp2LAT = 1;
#define dbg30 tp3LAT = 0;
#define dbg31 tp3LAT = 1;
#define dbg40 tp4LAT = 0;
#define dbg41 tp4LAT = 1;

#endif	/* B4 */

#ifdef U5
#define I2C_START_BIT SSP2STATbits.S
#define I2C_STOP_BIT  SSP2STATbits.P
#define I2C_WCOL      SSP2CON1bits.WCOL
#define I2C_SSPOV     SSP2CON1bits.SSPOV
#define I2C_BUF_BYTE  SSP2BUF
#define I2C_SSPIF     _SSP2IF

#define motAPORT   PORTA  // tool A
#define motBPORT   PORTB  // tool B
#define motCPORT   PORTC  // tool C
#define motPPORT   PORTB  // paster
#define motFPORT   PORTC  // focus

#define motATRIS   TRISA  // tool A
#define motBTRIS   TRISB  // tool B
#define motCTRIS   TRISC  // tool C
#define motPTRIS   TRISB  // paster
#define motFTRIS   TRISC  // focus

#define motAOFS    0
#define motBOFS   12
#define motCOFS    0
#define motPOFS    8
#define motFOFS    4

#define led1TRIS   _TRISA6
#define led2TRIS   _TRISA7
#define led3TRIS   _TRISA4
#define led4TRIS   _TRISA9
 
#define led1LAT    _LATA6
#define led2LAT    _LATA7
#define led3LAT    _LATA4
#define led4LAT    _LATA9

#define tp1TRIS    _TRISC8
#define tp2TRIS    _TRISC9
#define tp3TRIS    _TRISA10
#define tp4TRIS    _TRISA11

#define tp1LAT     _LATC8
#define tp2LAT     _LATC9
#define tp3LAT     _LATA10
#define tp4LAT     _LATA11

// tp3 doesn't work ???
#define dbg10 tp1LAT = 0;
#define dbg11 tp1LAT = 1;
#define dbg20 tp2LAT = 0;
#define dbg21 tp2LAT = 1;
#define dbg30 tp3LAT = 0;
#define dbg31 tp3LAT = 1;
#define dbg40 tp4LAT = 0;
#define dbg41 tp4LAT = 1;
#endif /* U5 */

#endif	/* PINS_H */

