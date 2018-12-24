// PIC24F16KM202 Configuration Bit Settings

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
// FICD
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (EMUC/EMUD share PGC2/PGD2)

#include <xc.h>
#include "types.h"
#include "i2c.h" 
#include "pins.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "dist-table.h"

int main(void) {
 _RCDIV  = 0; // switch instruction clock from 4 MHz to 8 MHz
 ANSA    = 0;   // no analog inputs
 ANSB    = 0;
 AUXLAT  = 0;  // aux pin (fan or buzzer in P3))
 AUXTRIS = 0;  // aux pin output, shared with ICSPCLK2

#ifdef DEBUG
 tp1TRIS = 0;
 tp2TRIS = 0;
 tp3TRIS = 0;
 tp4TRIS = 0;
 
 tp1LAT  = 0;
 tp2LAT  = 0;
 tp3LAT  = 0;
 tp4LAT  = 0;
#endif
    
  i2cInit();
  clkInit();
  motorInit();

 _NSTDIS = 1;  // nested interrupts disabled
  enableAllInts;
  
  // main event loop -- never ends
  while(true) {
    // motorIdx, mp, mm, ms, and sv are globals
    for(motorIdx=0; motorIdx < NUM_MOTORS; motorIdx++) {
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
    }
  }
}
