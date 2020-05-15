/**
 * @file intc.c
 * @author Elod Gyorgy
 * @date 3 Jan 2015
 * @brief File containing function for initialization of the interrupt system.
 */

#ifndef LINUX_APP
#include "intc.h"

INTC Intc;
bool fIntCInit = false;

/**
 * Initialize an interrupt controller.
 *
 * @param psIntc a pointer to the XScuGic driver instance data
 *
 * @return XST_SUCCESS on success,
 *  XST_FAILURE on failure
 */
XStatus fnInitInterruptController(INTC *pIntc)
{

	// Init driver instance
#ifdef PLATFORM_ZYNQ
	XScuGic_Config *IntcConfig;

	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (IntcConfig == NULL)
	{
		return XST_FAILURE;
	}
	if (XScuGic_CfgInitialize(pIntc, IntcConfig,
				IntcConfig->CpuBaseAddress) != XST_SUCCESS)
		return XST_FAILURE;
	Xil_ExceptionInit();
	// Register the interrupt controller handler with the exception table.
	// This is in fact the ISR dispatch routine, which calls our ISRs

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
				pIntc);
#else
	RETURN_ON_FAILURE(XIntc_Initialize(pIntc, INTC_DEVICE_ID));
	XIntc_Start(pIntc, XIN_REAL_MODE);
	Xil_ExceptionInit();
	// Register the interrupt controller handler with the exception table.
	// This is in fact the ISR dispatch routine, which calls our ISRs

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler)XIntc_InterruptHandler,
				pIntc);
#endif

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/**
 * Enable interrupts and connect interrupt service routines.
 *
 * @param psIntc a pointer to the XScuGic driver instance data
 * @param prgsIvt a pointer to a vector of ivt_t structs that will be
 *  used to enable and connect the interrupts to their callback functions
 * @param csIVectors the number of interrupts that need to be connected
 */
void fnEnableInterrupts(INTC *pIntc, const ivt_t *prgsIvt, unsigned int csIVectors)
{
	unsigned int isIVector;

	Xil_AssertVoid(pIntc != NULL);
	Xil_AssertVoid(pIntc->IsReady == XIL_COMPONENT_IS_READY);

	/* Hook up interrupt service routines from IVT */
	for (isIVector = 0; isIVector < csIVectors; isIVector++)
	{
		/* Connect & Enable the interrupt vector at the interrupt controller */
#ifdef PLATFORM_ZYNQ
		XScuGic_Connect(pIntc, prgsIvt[isIVector].id, prgsIvt[isIVector].handler, prgsIvt[isIVector].pvCallbackRef);
		XScuGic_Enable(pIntc, prgsIvt[isIVector].id);
#else
		XIntc_Connect(pIntc, prgsIvt[isIVector].id, prgsIvt[isIVector].handler, prgsIvt[isIVector].pvCallbackRef);
		XIntc_Enable(pIntc, prgsIvt[isIVector].id);
#endif
	}
}
#endif // LINUX_APP
