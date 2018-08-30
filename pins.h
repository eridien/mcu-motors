  
#ifndef PINS_H
#define	PINS_H

#define SCL_TRIS  TRISC0
#define SDA_TRIS  TRISC1

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

#ifdef B1

#define stepTRIS  TRISC5
#define stepLAT   LATC5
#define stepPORT  PORTC
#define stepBIT   0x20

#define faultTRIS TRISA5
#define faultPORT PORTA
#define faultBIT  0x80

#define limitTRIS TRISA4
#define limitPORT PORTA
#define limitBIT  0x10

#else


#define stepRTRIS  TRISC5
#define stepETRIS  TRISC4
#define stepXTRIS  TRISC3

#define stepRLAT   LATC5
#define stepELAT   LATC4
#define stepXLAT   LATC3

#define stepRPORT  PORTC
#define stepRBIT   0x20
#define stepEPORT  PORTC
#define stepEBIT   0x10
#define stepXPORT  PORTC
#define stepXBIT   0x08

#define faultRTRIS  TRISC6
#define faultETRIS  TRISC7
#define faultXTRIS  TRISB7

#define faultRLAT   LATC6
#define faultELAT   LATC7
#define faultXLAT   LATB7

#define faultRPORT  PORTC
#define faultRBIT   0x40
#define faultEPORT  PORTC
#define faultEBIT   0x80
#define faultXPORT  PORTB
#define faultXBIT   0x80

#define limitRTRIS  TRISA5
#define limitXTRIS  TRISA4

#define limitRPORT  PORTA
#define limitRBIT   0x20
#define limitXPORT  PORTA
#define limitXBIT   0x10

#endif	/* M1 */

#endif	/* PINS_H */

