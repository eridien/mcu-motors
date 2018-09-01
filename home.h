#ifndef HOME_H
#define	HOME_H

#include <xc.h>
#include "types.h"
#include "motor.h"

#define homeStarting  0
#define goingHome     1
#define homeReversing 2
#define homingToOfs   3

void chkHoming(void);
void homeCommand(void);


#endif	/* HOME_H */

