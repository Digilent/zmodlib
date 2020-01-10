/******************************************************************************
 * @file iic.c
 * Interrupt system initialization.
 *
 * @author Elod Gyorgy
 *
 * @date 2015-Jan-3
 *
 * @copyright
 * (c) 2015 Copyright Digilent Incorporated
 * All Rights Reserved
 *
 * This program is free software; distributed under the terms of BSD 3-clause
 * license ("Revised BSD License", "New BSD License", or "Modified BSD License")
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name(s) of the above-listed copyright holder(s) nor the names
 *    of its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * @desciption
 * Contains interrupt controller initialization function.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who          Date     Changes
 * ----- ------------ ----------- -----------------------------------------------
 * 1.00  Elod Gyorgy  2015-Jan-3 First release
 *
 * </pre>
 *
 *****************************************************************************/
#ifndef LINUX_APP
#include "intc.h"
#include "xparameters.h"

#define INTC_DEVICE_ID XPAR_SCUGIC_0_DEVICE_ID

XScuGic sIntc;
bool fIntCInit = false;

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
	RETURN_ON_FAILURE(XScuGic_CfgInitialize(psIntc, psIntcConfig, psIntcConfig->CpuBaseAddress));

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
