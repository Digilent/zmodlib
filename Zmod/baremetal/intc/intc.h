/**
 * @file intc.h
 * @author Elod Gyorgy
 * @date 3 Jan 2015
 * @brief File containing function definitions for initialization of the interrupt system.
 */

#ifndef INTC_H_
#define INTC_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include "xparameters.h"
#include "xstatus.h"
#ifdef PLATFORM_ZYNQ
#include "xscugic.h"
#define INTC XScuGic
#define INTC_DEVICE_ID XPAR_PS7_SCUGIC_0_DEVICE_ID
#else
#include "xintc.h"
#define INTC XIntc
#define INTC_DEVICE_ID XPAR_INTC_0_DEVICE_ID
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


#if defined(__cplusplus)
}
#endif

#endif /* INTC_H_ */
