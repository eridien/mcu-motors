
#ifndef DEBUG_H
#define	DEBUG_H

#define DBG_HIST_LEN 512
#define setBrkptHere() volatile bool x = 0

void initDebug(void);
void dbgHist(uint16 val);

#endif	/* DEBUG_H */

