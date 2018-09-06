
#include <xc.h>
#include "dist-table.h"
#include "types.h"
#include "pins.h"

// disttable is defined in disttable.asm
extern const uint16 disttable;

uint16 distTableAddr;

void initDistTable(void){
    distTableAddr = (uint16) &disttable;
}

// table is 8 (accel) by 256 (speed)
// entry is dist of decel in 1/8 steps

// accel steps/sec/sec (assuming 40 steps/mm): 
//     1500, 1250, 1000, 800, 600, 400, 200, 0 (off)
// const uint16 accelTab[] = [0, 8000, 16000, 24000, 32000, 40000, 50000, 60000];

// speed resolution of 3.2 mm/sec (128/40)  (assuming 40 steps/mm)
// 256 speed values, 128 delta, (6.4 to 819.2 mm/sec)
// up to 32,767 steps/sec (4 kHz pps)
uint16 calcDist(uint16 accel, uint16 speed) {
  if(speed >= 0x8000) speed = 0x7fff;
  uint16 addr = distTableAddr + ((accel << 8) | (speed >> 7));
  NVMCON1bits.NVMREGS = 0;
  NVMADRL = addr & 0xff;
  NVMADRH = addr >> 8;
  NVMCON1bits.RD = 1;
  uint16 dist = (NVMDATH << 8) | NVMDATL;
  return dist;
}
