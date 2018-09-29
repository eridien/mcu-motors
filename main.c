#ifdef B1

// PIC16F15375 Configuration Bit Settings

#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = OFF     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:0x10000; software control of WDTPS)
#pragma config WDTE = SWDTEN     // WDT operating mode (WDT enabled/disabled by SWDTEN bit in WDTCON0)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)
#pragma config BBSIZE = BB512   //  (512 char boot block size) 
#pragma config BBEN = ON        //  (Boot Block enabled)
#pragma config SAFEN = OFF      //  (SAF disabled)
#pragma config WRTAPP = OFF     //  (Application Block not write protected)
#pragma config WRTB = OFF       //  (Boot Block not write protected)
#pragma config WRTC = OFF       //  (Configuration Register not write protected)
#pragma config WRTSAF = OFF     //  (SAF not write protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)
#else

// PIC24F16KM202 (& PIC24F16KM204) Configuration Bit Settings

// FBS
#pragma config BWRP = OFF               // Boot Segment Write Protect (Disabled)
#pragma config BSS = OFF                // Boot segment Protect (No boot program flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Write Protect (General segment may be written)
#pragma config GCP = OFF                // General Segment Code Protect (No Protection)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Select (Fast RC Oscillator with Postscaler and PLL Module (FRCDIV+PLL))
#pragma config SOSCSRC = ANA            // SOSC Source Type (Analog Mode for use with crystal)
#pragma config LPRCSEL = HP             // LPRC Oscillator Power and Accuracy (High Power, High Accuracy Mode)
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-speed Start-up enabled))

// FOSC
#pragma config POSCMOD = HS             // Primary Oscillator Configuration bits (HS oscillator mode selected)
#pragma config OSCIOFNC = IO            // CLKO Enable Configuration bit (Port I/O enabled (CLKO disabled))
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32768)
#pragma config FWPSA = PR128            // WDT Prescaler bit (WDT prescaler ratio of 1:128)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bits (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected(windowed WDT disabled))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware, SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Use Default SCL1/SDA1 Pins For I2C1)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (RA5 input pin disabled, MCLR pin enabled)

#ifdef B3
// FICD
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (EMUC/EMUD share PGC2/PGD2)
#endif

#ifdef U6
#pragma config ICS = PGx1               // ICD Pin Placement Select bits (EMUC/EMUD share PGC1/PGD1)
#endif

#endif

#include <xc.h>
#include "types.h"
#include "i2c.h" 
#include "pins.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "dist-table.h"

int main(void) {
#ifdef B1
 ANSELA = 0; // no analog inputs
 ANSELB = 0; // these &^%$&^ regs cause a lot of trouble
 ANSELC = 0; // they should not default to on and override everything else
//  debug1TRIS    = 0;
//  debug2TRIS    = 0;
//  debug3TRIS    = 0;
//  debug4TRIS    = 0;
#else
 _RCDIV = 0;
 ANSA = 0;
 ANSB = 0;
 
#ifdef U6
 ANSC = 0;
#endif

#ifdef DEBUG
 tp1TRIS = 0;
 tp2TRIS = 0;
 tp3TRIS = 0;
 tp4TRIS = 0;
 tp1LAT = 0;
 tp2LAT = 0;
 tp3LAT = 0;
 tp4LAT = 0;
#else
 tp1TRIS = 1;
 tp2TRIS = 1;
 tp3TRIS = 1;
 tp4TRIS = 1; 
#endif
 
 _NSTDIS = 1;  // nested interrupts disabled
#endif

  
 #ifdef B1
// show speed on DAC output pin A0 (ICP Data)
// always leave enabled, doesn't hurt anything
// remove pickit debugger to see output
// see setDacToSpeed() to use
  DAC1PSS0      = 0;  // DAC top ref is VDD
  DAC1NSS       = 0;  // DAC bot ref is GND
  DAC1OE1       = 1;  // enable DAC 1 pin
  DAC1OE2       = 1;  // enable DAC 2 pin
  DAC1CON1      = 0;  // DAC input value
  DAC1EN        = 1;  // turn DAC on
#endif
  
  i2cInit();
  clkInit();
  motorInit();

#ifdef B1
  initDistTable();
  PEIE =  1;   // enable peripheral ints
#endif
  
  enableAllInts;
  
  // main event loop -- never ends
  while(true) {
    // motorIdx, mp, mm, ms, and sv are globals
    for(motorIdx=0; motorIdx < NUM_MOTORS; motorIdx++) {
//      dbg11
      
      mp = stepPort[motorIdx]; // (&PORT)
      mm = stepMask[motorIdx]; // 0x000f, 0x00f0, 0x0f00, 0xf000, or step bit
      ms = &mState[motorIdx];
      sv = &(mSet[motorIdx].val);
      if(errorIntCode && errorIntMot == motorIdx) {
        // error happened during interrupt
        setError(errorIntCode);
        errorIntCode = 0;
      }
      if(ms->haveCommand) {
        processCommand();
        ms->haveCommand = false;
      }
      checkAll();
//      dbg10
    }
  }
}

 #ifdef B1
// global interrupt routine
void __interrupt() globalInt() {
  // motor (clock) interrupts every 20 usecs (50 KHz))
  if(TMR0IF) {
    TMR0IF = 0;
    clockInterrupt();
  }
  // i2c interrupts
  if(I2C_SSPIF) {
    I2C_SSPIF = 0;
    i2cInterrupt();
  }
}
#endif


