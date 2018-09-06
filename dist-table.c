
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

// table is 4 (ustep) by 8 (accel) by 64 (speed) -> 2048 14-bit entries
// return is dist of decel
uint16 calcDist(uint8 ustep, uint8 accel, uint16 speed) {
  if(speed >= 0x4000) speed = 0x3fff;
                dbg1 = 1;
  uint16 addr = distTableAddr + ((ustep << 9) | (accel << 6) | (speed >> 8));
                dbg1 = 0;
  NVMCON1bits.NVMREGS = 0;
  NVMADRL = addr & 0xff;
  NVMADRH = addr >> 8;
  NVMCON1bits.RD = 1;
  uint16 dist = (NVMDATH << 8) | NVMDATL;
  return dist;
}
