#ifndef HOME_H
#define	HOME_H

#include <xc.h>
#include "types.h"
#include "motor.h"

#define homingIdle   0
#define homeStarting 1
#define homingIn     2
#define homingSwitch 3
#define homingOut    4
#define homingOfs    5

void chkHoming(void);
void homeCommand(void);


#endif	/* HOME_H */

