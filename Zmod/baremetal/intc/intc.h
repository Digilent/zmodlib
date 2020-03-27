/**
 * @file intc.h
 * @author Elod Gyorgy
 * @date 3 Jan 2015
 * @brief File containing function definitions for initialization of the interrupt system.
 */

#ifndef INTC_H_
#define INTC_H_

#include "xstatus.h"
#ifdef __ZYNQ__
#include "xscugic.h"
#define INTC XScuGic
#else
#include "xintc.h"
#define INTC XIntc
#endif

#include "xil_exception.h"

/**
 * Structure for interrupt id, handler and callback reference.
 */
typedef struct {
	int id; ///< Interrupt ID
	XInterruptHandler handler; ///< Interrupt handler
	void *pvCallbackRef; ///< Interrupt callback
} ivt_t;

XStatus fnInitInterruptController(INTC *psIntc);
void fnEnableInterrupts(INTC *psIntc, const ivt_t *prgsIvt, unsigned int csIVectors);


#endif /* INTC_H_ */
