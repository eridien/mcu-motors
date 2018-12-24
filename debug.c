
#include "types.h"
#include "debug.h"

uint16 dbgHistory[DBG_HIST_LEN];
uint16 dbgHistIdx = 0;

void initDebug(void) {
  for(dbgHistIdx = 0; dbgHistIdx < DBG_HIST_LEN; dbgHistIdx++)
    dbgHistory[dbgHistIdx] = 0xeeee;
  dbgHistIdx = 0;
}

void dbgHist(uint16 val) {
  if(dbgHistIdx < DBG_HIST_LEN)
    dbgHistory[dbgHistIdx++] = val;
  else {
    setBrkptHere();
  }
}

