
#include <xc.h>
#include "dist-table.h"
#include "types.h"

// disttable is defined in disttable.asm
extern const uint16_t disttable;

uint16 distTableAddr;

void initDistTable(){
    distTableAddr = (uint16) &disttable;
}

// table is 4 (ustep) by 8 (accel) by 64 (speed) -> 2048 14-bit entries
// return is dist of decel
void calcDist(uint8 ustep, uint8 accel, uint16 speed) {
  uint16 addr = (ustep << 9 | accel << 6 | (speed / 200));
}

void pps2usecs(uint16_t pps)
{
  uint8_t exp = 0;
  if (pps < 128)
  { // 16 pulses/sec is min legal value
    ppsUsecs.val = 0xffffff;
    return;
  }
  // exp will be 2 to 10
  for (exp = 2; !(pps & 0x8000); pps <<= 1, exp++)
    ;

  // linker maxes out at 2048 words so table is split in two
  // 2nd table addr is already offset by 2048 (0x0800)
  uint16_t ofs = (pps & 0x7ff8) >> 3;
  uint16_t addr = (ofs < 0x0800 ? invTableAddr1 : invTableAddr2) + ofs;
  NVMCON1bits.NVMREGS = 0;
  NVMADRL = addr & 0xff;
  NVMADRH = addr >> 8;
  NVMCON1bits.RD = 1;
  if (exp >= 8)
  {
    ppsUsecs.bytes[2] = NVMDATH;
    ppsUsecs.bytes[1] = NVMDATL;
    ppsUsecs.bytes[0] = 0;
    ppsUsecs.val <<= exp - 8;
  }
  else
  {
    ppsUsecs.bytes[2] = 0;
    ppsUsecs.bytes[1] = NVMDATH;
    ppsUsecs.bytes[0] = NVMDATL;
    ppsUsecs.val <<= exp;
  }
}
