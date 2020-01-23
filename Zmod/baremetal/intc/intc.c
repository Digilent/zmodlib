/**
 * @file intc.c
 * @author Elod Gyorgy
 * @date 3 Jan 2015
 * @brief File containing function for initialization of the interrupt system.
 */

#ifndef LINUX_APP
#include "intc.h"
#include "xparameters.h"

#define INTC_DEVICE_ID XPAR_SCUGIC_0_DEVICE_ID	///< Interrupt controller device ID


XScuGic sIntc;	///< Interrupt controller instance
bool fIntCInit = false; ///< Interrupt controller initialized flag

/**
 * Initialize an interrupt controller.
 *
 * @param psIntc a pointer to the XScuGic driver instance data
 *
 * @return XST_SUCCESS on success,
 *  XST_FAILURE on failure
 */
XStatus fnInitInterruptController(XScuGic *psIntc)
{
	XScuGic_Config *psIntcConfig;

	psIntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (psIntcConfig == NULL)
	{
		return XST_FAILURE;
	}

	// Init driver instance
	if (XScuGic_CfgInitialize(psIntc, psIntcConfig,
			psIntcConfig->CpuBaseAddress) != XST_SUCCESS)
		return XST_FAILURE;

	// Start interrupt controller ????

	Xil_ExceptionInit();
	// Register the interrupt controller handler with the exception table.
	// This is in fact the ISR dispatch routine, which calls our ISRs
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
				psIntc);
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
void fnEnableInterrupts(XScuGic *psIntc, const ivt_t *prgsIvt, unsigned int csIVectors)
{
	unsigned int isIVector;

	Xil_AssertVoid(psIntc != NULL);
	Xil_AssertVoid(psIntc->IsReady == XIL_COMPONENT_IS_READY);

	/* Hook up interrupt service routines from IVT */
	for (isIVector = 0; isIVector < csIVectors; isIVector++)
	{
		XScuGic_Connect(psIntc, prgsIvt[isIVector].id, prgsIvt[isIVector].handler, prgsIvt[isIVector].pvCallbackRef);

		/* Enable the interrupt vector at the interrupt controller */
		XScuGic_Enable(psIntc, prgsIvt[isIVector].id);
	}
}
#endif // LINUX_APP
