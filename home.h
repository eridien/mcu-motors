#ifndef HOME_H
#define	HOME_H

#include <xc.h>
#include "types.h"
#include "motor.h"

// homing states
#define movingToFwdSide 0
#define goingHome       1
#define homeReversing   2
#define homingToOfs     3

void chkHoming(void);
void homeCommand(bool start);


#endif	/* HOME_H */

