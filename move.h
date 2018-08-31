#ifndef MOVE_H
#define	MOVE_H

#include <xc.h>
#include "types.h"
#include "motor.h"

void chkMoving(void);
void moveCommand(void);
bool underAccellLimit(void);

#endif	/* MOVE_H */

