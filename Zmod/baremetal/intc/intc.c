/**
 * @file intc.c
 * @author Elod Gyorgy
 * @date 3 Jan 2015
 * @brief File containing function for initialization of the interrupt system.
 */

#ifndef LINUX_APP

#include "intc.h"
#include "xparameters.h"

#ifdef __MICROBLAZE__
#include "xintc.h"
#define Intc XIntc
#define Intc_InterruptHandler XIntc_InterruptHandler
#define Intc_Config XIntc_Config
#define Intc_LookupConfig XIntc_LookupConfig
#define Intc_Connect XIntc_Connect
#define Intc_Enable XIntc_Enable

#define INTC_DEVICE_ID XPAR_INTC_0_DEVICE_ID
#else
#include "xscugic.h"
#define Intc XScuGic
#define Intc_InterruptHandler XScuGic_InterruptHandler
#define Intc_Config XScuGic_Config
#define Intc_Connect XScuGic_Connect
#define Intc_Enable XScuGic_Enable

#define INTC_DEVICE_ID XPAR_SCUGIC_0_DEVICE_ID
#endif

Intc sIntc;	///< Interrupt controller instance
bool fIntcInit = false; ///< Interrupt controller initialized flag

/**
 * Initialize an interrupt controller.
 *
 * @return XST_SUCCESS on success,
 *  XST_FAILURE on failure
 */
XStatus fnInitInterruptController()
{
	s32 status;

	if (fIntcInit) {
		return XST_SUCCESS;
	}

#ifdef __MICROBLAZE__
	status = XIntc_Initialize(&sIntc, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
		return status;

	status = XIntc_Start(&sIntc, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
		return status;
#else
	Intc_Config *psIntcConfig;

	psIntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (psIntcConfig == NULL)
		return XST_FAILURE;

	status = XScuGic_CfgInitialize(sIntc, psIntcConfig,
			psIntcConfig->CpuBaseAddress);
	if (status != XST_SUCCESS)
		return status;
#endif

	Xil_ExceptionInit();
	// Register the interrupt controller handler with the exception table.
	// This is in fact the ISR dispatch routine, which calls our ISRs
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler)Intc_InterruptHandler,
				&sIntc);
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/**
 * Enable interrupts and connect interrupt service routines.
 *
 * @param id the interrupt id
 * @param callback function to call when interrupt occurs
 * @param data data to pass to function called when interrupt occurs
 */
void fnEnableInterrupt(int id, XInterruptHandler callback, void *data)
{
	Xil_AssertVoid(sIntc.IsReady == XIL_COMPONENT_IS_READY);

	/* Hook up interrupt callback */
	Intc_Connect(&sIntc, id, callback, data);

	/* Enable the interrupt */
	Intc_Enable(&sIntc, id);
}
#endif // LINUX_APP
