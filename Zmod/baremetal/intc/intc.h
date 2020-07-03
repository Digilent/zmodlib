/**
 * @file intc.h
 * @author Elod Gyorgy
 * @date 3 Jan 2015
 * @brief File containing function definitions for initialization of the interrupt system.
 */

#ifndef INTC_H_
#define INTC_H_

#include "xstatus.h"
#include "xil_exception.h"

XStatus fnInitInterruptController();
void fnEnableInterrupt(int id, XInterruptHandler callback, void *data);


#endif /* INTC_H_ */
