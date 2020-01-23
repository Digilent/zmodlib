/**
 * @file intc.h
 * @author Elod Gyorgy
 * @date 3 Jan 2015
 * @brief File containing function definitions for initialization of the interrupt system.
 */

#ifndef INTC_H_
#define INTC_H_

#include "xstatus.h"
#include "xscugic.h"
#include "xil_exception.h"

/**
 * Structure for interrupt id, handler and callback reference.
 */
typedef struct {
	int id; ///< Interrupt ID
	XInterruptHandler handler; ///< Interrupt handler
	void *pvCallbackRef; ///< Interrupt callback
} ivt_t;

XStatus fnInitInterruptController(XScuGic *psIntc);
void fnEnableInterrupts(XScuGic *psIntc, const ivt_t *prgsIvt, unsigned int csIVectors);


#endif /* INTC_H_ */
