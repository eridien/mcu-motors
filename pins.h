  
#ifndef PINS_H
#define	PINS_H

#ifdef BM
#define I2C_START_BIT SSP1STATbits.S
#define I2C_STOP_BIT  SSP1STATbits.P
#define I2C_WCOL      WCOL1
#define I2C_SSPOV     SSPOV1
#define I2C_BUF_BYTE  SSP1BUF
#define I2C_SSPIF     SSP1IF

#else

#define I2C_START_BIT SSP2STATbits.S
#define I2C_STOP_BIT  SSP2STATbits.P
#define I2C_WCOL      WCOL2
#define I2C_SSPOV     SSPOV2
#define I2C_BUF_BYTE  SSP2BUF
#define I2C_SSPIF     SSP2IF
#endif /* BM */

#ifdef B1
#define dirTRIS   TRISA2
#define ms1TRIS   TRISC2
#define ms2TRIS   TRISB4
#define ms3TRIS   TRISB5
#define resetTRIS TRISB6

#define dirLAT    LATA2
#define ms1LAT    LATC2
#define ms2LAT    LATB4
#define ms3LAT    LATB5
#define resetLAT  LATB6

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

#define debug1TRIS   TRISC4
//#define debug2TRIS   TRISC3
//#define debug3TRIS   TRISC6
//#define debug4TRIS   TRISA4
//
#define dbg1         LATC4 // test point TP1
//#define dbg2         LATC3 // test point TP2
//#define dbg3         LATC6 // test point TP3
//#define dbg4         LATA4 // usually lim y switch

#endif /* B1 */

#ifdef B3
#define dirTRIS   TRISA6
#define ms1TRIS   TRISA7
#define ms2TRIS   TRISB7
#define ms3TRIS   TRISB6
#define resetRTRIS TRISB15
#define resetETRIS TRISB14
#define resetXTRIS TRISB13

#define dirLAT    LATA6
#define ms1LAT    LATA7
#define ms2LAT    LATB7
#define ms3LAT    LATB6
#define resetRLAT LATB15
#define resetELAT LATB14
#define resetXLAT LATB13

#define stepRTRIS  TRISB1
#define stepETRIS  TRISB2
#define stepXTRIS  TRISB3

#define stepRLAT   LATB1
#define stepELAT   LATB2
#define stepXLAT   LATB3

#define stepRPORT  PORTB
#define stepRBIT   0x02
#define stepEPORT  PORTB
#define stepEBIT   0x04
#define stepXPORT  PORTB
#define stepXBIT   0x08

#define faultRTRIS  TRISB4
#define faultETRIS  TRISA4
#define faultXTRIS  TRISB5

#define faultRLAT   LATB4
#define faultELAT   LATA4
#define faultXLAT   LATB5

#define faultRPORT  PORTB
#define faultRBIT   0x10
#define faultEPORT  PORTA
#define faultEBIT   0x10
#define faultXPORT  PORTB
#define faultXBIT   0x20

#define limitRTRIS  TRISA0
#define limitXTRIS  TRISA1

#define limitRPORT  PORTA
#define limitRBIT   0x01
#define limitXPORT  PORTA
#define limitXBIT   0x02

#define tp1TRIS   TRISB0
#define tp2TRIS   TRISB12
#define tp3TRIS   TRISA2
#define tp4TRIS   TRISA3

#define tp1LAT     LATB0
#define tp2LAT     LATB12
#define tp3LAT     LATA2
#define tp4LAT     LATA3

#endif	/* B3 */

#define debug1TRIS   TRISC7
//#define debug2TRIS   TRISB7
//
#define dbg1         LATC7 // test point TP1
//#define dbg2         LATB7 // test point TP2

#ifdef U6
#define motAPORT   PORTA  // tube 1
#define motBPORT   PORTB  // tube 2
#define motCPORT   PORTC  // tube 3
#define motPPORT   PORTD  // paster
#define motZPORT   PORTA  // camera Z
#define motFPORT   PORTC  // focus

#define motAOFS    0
#define motBOFS    0
#define motCOFS    0
#define motPOFS    4
#define motZOFS    4
#define motFOFS    4

#define limitZPORT PORTE
#define limitZBIT  0x01

#define led1TRIS   TRISE1
#define led2TRIS   TRISE2
#define led3TRIS   TRISD2
#define led4TRIS   TRISD3
 
#define led1LAT    LATE1
#define led2LAT    LATE2
#define led3LAT    LATD2
#define led4LAT    LATD3
 
#define tp1TRIS    TRISB4
#define tp1LAT     LATB4
#define tp2TRIS    TRISB5
#define tp2LAT     LATB5
#endif

#endif	/* PINS_H */

