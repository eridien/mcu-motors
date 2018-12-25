#ifndef STOP_H
#define	STOP_H

void resetMotor(void);
void stopStepping(void);
void softStopCommand(bool reset);
void chkStopping(void);

#endif	/* STOP_H */

