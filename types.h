
#ifndef TYPES_H
#define	TYPES_H

// todo
//   set homing dir for rotation motor
//   status returns home pos after homing test
//   new busy and err bits
//   stop all motors on error
//   keep pos accurate in recvbytes
//   chg speed to steps/sec (8x for bipolar)  
//   chg max ustep to 1/8

#define BM  // bipolar motor
#define B1  // one bipolar motor
//#define B3  // three bipolar motors
//#define U6  // six unipolar motors

typedef signed char int8;
typedef unsigned char uint8;
typedef int int16;
typedef unsigned int uint16;
typedef __int24 int24;
typedef __uint24 uint24;
typedef long int32;
typedef unsigned long uint32;
typedef char bool;

#define true  1
#define false 0

#endif	/* TYPES_H */

/*
 * interrupt functions that require motIdx
 * 
 * setErrorInt
 * setI2cSendBytesInt
 */