// Contains the TC5 Handler declaration

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#include "stdint.h"

#define WAIT_TC16_REGS_SYNC(x) while (x->COUNT16.STATUS.bit.SYNCBUSY);

void TC5_Handler();
void TC5_Handler_Legacy();
void updateMotorCurrent();
void updateVelocity();
void updateAngle();
extern bool USE_EXTERNAL_INTERRUPTS;

#endif
