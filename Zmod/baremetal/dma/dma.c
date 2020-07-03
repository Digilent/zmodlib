/**
 * @file baremetal/dma/dma.c
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing implementations of platform-specific methods for DMA devices.
 */
#ifndef LINUX_APP

#include "xparameters.h"
#include "xil_printf.h"
#include "xaxidma.h"

#include "xstatus.h"

#include "../../dma.h"
#include "../intc/intc.h"

#define AXIDMA_REG_ADDR_MM2S_DMACR 		0x00 ///< MM2S DMACR register
#define AXIDMA_REG_ADDR_MM2S_SA 		0x18 ///< MM2S SA register
#define AXIDMA_REG_ADDR_MM2S_SA_LENGTH 	0x28 ///< MM2S SA length register
#define AXIDMA_REG_ADDR_S2MM_DMACR 		0x30 ///< S2MM DMACR register
#define AXIDMA_REG_ADDR_S2MM_DA 		0x48 ///< S2MM DA register
#define AXIDMA_REG_ADDR_S2MM_DA_LENGTH	0x58 ///< S2MM DA length register
#define AXIDMA_REGFLD_MM2S_DMACR_RUNSTOP 		AXIDMA_REG_ADDR_MM2S_DMACR, 0, 1 ///< RUNSTOP field of MM2S_DMACR DMA register
#define AXIDMA_REGFLD_MM2S_DMACR_IOC_IRQ 		AXIDMA_REG_ADDR_MM2S_DMACR, 12, 1 ///< IOC_IRQ field of MM2S_DMACR DMA register
#define AXIDMA_REGFLD_S2MM_DMACR_RUNSTOP 		AXIDMA_REG_ADDR_S2MM_DMACR, 0, 1 ///< RUNSTOP field of S2MM_DMACR DMA register
#define AXIDMA_REGFLD_S2MM_DMACR_IOC_IRQ 		AXIDMA_REG_ADDR_S2MM_DMACR, 12, 1 ///< IOC_IRQ field of S2MM_DMACR DMA register

/**
 * Struct containing data specific to this DMA instance.
 */
typedef struct _dmaEnv {
	enum dma_direction direction; ///< the direction of the DMA transfer
	XAxiDma *xAxiDma; ///< a pointer to the XAxiDma driver instance data
	uint32_t base_addr; ///< the physical address of the DMA device
	uint8_t complete_flag; ///< whether the current DMA transfer is complete or not
} DMAEnv;

/**
 * Write a DMA register.
 *
 * @param baseAddr the base address of the DMA device
 * @param regAddr the offset address of the register
 * @param value the value to write
 */
void writeDMAReg(uintptr_t baseAddr, uint8_t regAddr, uint32_t value)
{
	XAxiDma_WriteReg(baseAddr, regAddr, value);
}

/**
 * Read a DMA register.
 *
 * @param baseAddr the base address of the DMA device
 * @param regAddr the offset address of the register
 *
 * @return the value read
 */
uint32_t readDMAReg(uintptr_t baseAddr, uint8_t regAddr) {
	return XAxiDma_ReadReg(baseAddr, regAddr);
}

/**
 * Write a register field.
 *
 * @param baseAddr the base address of the DMA device
 * @param regAddr the offset address of the register
 * @param lsbBit the index of the first bit to write out of the register
 * @param noBits the number of bits to write
 * @param value the value to write in the register, will only be written to the
 *  bits starting at lsbBit (inclusive) and ending at lsbBit + noBits (exclusive)
 */
void writeDMARegFld(uintptr_t baseAddr, uint8_t regAddr, uint8_t lsbBit, uint8_t noBits, uint32_t value) {
	uint32_t regMask = ((1 << noBits) - 1) << lsbBit;
	uint32_t regValue = readDMAReg(baseAddr, regAddr);

	// align value to bit lsb_bit
	value <<= lsbBit;

	// mask out any bits outside specified field
	value &= regMask;

	// mask out bits corresponding to the specified field
	regValue &= ~regMask;

	// set the values for the field bits
	regValue |= value;

	writeDMAReg(baseAddr, regAddr, regValue);
}

/**
 * Call when a Stream to MemoryMap interrupt is triggered for the DMA.
 *
 * @param Callback a pointer to the S2MM channel of the DMA engine
 */
void fnDMAInterruptHandler(void *Callback) {
	DMAEnv *dmaEnv = (DMAEnv *)Callback;
	XAxiDma *AxiDmaInst;
	uint32_t IrqStatus;
	int TimeOut;

	if (!dmaEnv)
		return;

	AxiDmaInst = dmaEnv->xAxiDma;

	if (dmaEnv->direction == DMA_DIRECTION_RX) {
		// Read all the pending DMA interrupts
		IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);

		// Acknowledge pending interrupts
		XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);
	} else {// DMA_DIRECTION_TX
		// Read all the pending DMA interrupts
		IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DMA_TO_DEVICE);

		// Acknowledge pending interrupts
		XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DMA_TO_DEVICE);

	}
	// If there are no interrupts we exit the Handler
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}

	// If error interrupt is asserted, raise error flag, reset the
	// hardware to recover from the error, and return with no further
	// processing.
	if (IrqStatus & XAXIDMA_IRQ_ERROR_MASK) {
		XAxiDma_Reset(AxiDmaInst);
		TimeOut = 100;
		while (TimeOut) {
			if(XAxiDma_ResetIsDone(AxiDmaInst)) {
				break;
			}

			TimeOut -= 1;
		}
		return;
	}

    // Clear Cache
    Xil_DCacheFlush();

	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {
		dmaEnv->complete_flag = 1;
	}
}

/**
 * Configure the DMA in interrupt mode.
 *
 * This implies that the scatter gather function is disabled.
 * Prior to calling this function the user must make sure that the Interrupts
 * and the Interrupt Handlers have been configured.
 *
 * @param AxiDma a pointer to the XAxiDma driver instance data
 * @param dmaBaseAddr the base address of the DMA device
 *
 * @return base address of the DMA device on success, a negative number on failure
 */
int fnConfigDma(XAxiDma *AxiDma, uintptr_t dmaBaseAddr) {
	XAxiDma_Config *pCfgPtr;
	int Status;

	// Make sure the DMA hardware is present in the project
	// Ensures that the DMA hardware has been loaded
	pCfgPtr = XAxiDma_LookupConfigBaseAddr(dmaBaseAddr);
	if (!pCfgPtr) {
		xil_printf("No config found for %X\n", dmaBaseAddr);
		return -1;
	}

	// Initialize DMA
	// Read and set all the available information
	// about the DMA to the AxiDma variable
	Status = XAxiDma_CfgInitialize(AxiDma, pCfgPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %X\n");
		return -1;
	}

	// Ensures that the Scatter Gather mode is not active
	if (XAxiDma_HasSg(AxiDma)) {
		xil_printf("Device cannot be configured as SG mode\n");
		return -1;
	}

	// Enable all the DMA Interrupts
	XAxiDma_IntrEnable(AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrEnable(AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

    // Disable cache
    Xil_DCacheDisable();

	return pCfgPtr->BaseAddr;
}

/**
 * Initialize the DMA.
 *
 * @param dmaBaseAddr the base address of the DMA device
 * @param direction the direction of the DMA transfers
 * @param dma_interrupt_id the interrupt number of the DMA device
 *
 * @return a pointer to an instance of DMAEnv
 */
uint32_t fnInitDMA(uintptr_t dmaBaseAddr, enum dma_direction direction,
		int dma_interrupt_id) {
	DMAEnv *dmaEnv = (DMAEnv *)malloc(sizeof(DMAEnv));
	if (!dmaEnv) {
		xil_printf("Can't allocate DMAEnv for %X\n", dmaBaseAddr);
		return 0;
	}

	// Init interrupt controller
	fnInitInterruptController();

	dmaEnv->xAxiDma = (XAxiDma *)malloc(sizeof(XAxiDma));
	if (!dmaEnv->xAxiDma) {
		xil_printf("Can't allocate XAxiDma for %X\n", dmaBaseAddr);
		return 0;
	}

	dmaEnv->base_addr = fnConfigDma(dmaEnv->xAxiDma, dmaBaseAddr);
	if (dmaEnv->base_addr < 0) {
		xil_printf("DMA configuration failure for %X\n", dmaBaseAddr);
		fnDestroyDMA((uint32_t)dmaEnv);
		return 0;
	}

	// Enable all interrupts in the interrupt vector table
	fnEnableInterrupt(dma_interrupt_id, (XInterruptHandler)fnDMAInterruptHandler, dmaEnv);

	dmaEnv->direction = direction;
	if (dmaEnv->direction == DMA_DIRECTION_RX) {
		// enable AXIDMA S2MM IOC interrupt
		writeDMARegFld(dmaEnv->base_addr, AXIDMA_REGFLD_S2MM_DMACR_IOC_IRQ, 1);
	}
	else{	// DMA_DIRECTION_TX
		// enable AXIDMA MM2S IOC interrupt
		writeDMARegFld(dmaEnv->base_addr, AXIDMA_REGFLD_MM2S_DMACR_IOC_IRQ, 1);

	}

    return (uint32_t)dmaEnv;
}

/**
 * Destroy a DMAEnv instance.
 *
 * @param addr the address of the DMAEnv instance returned by fnInitDMA
 */
void fnDestroyDMA(uintptr_t addr) {
	DMAEnv *dmaEnv = (DMAEnv *)addr;
	if (!dmaEnv)
		return;

	if(dmaEnv->xAxiDma)
	{
		free(dmaEnv->xAxiDma);
	}
	free(dmaEnv);
}

/**
 * Start a one-way DMA transfer.
 *
 * @param addr the address of the DMAEnv instance returned by fnInitDMA
 * @param buf the buffer to receive the acquired data,
 *  must be previously allocated to a dimension large enough to accommodate the requested transfer
 * @param transfer_size the size of the transfer in bytes
 *
 * @return 0 on success, any other number on failure
 */
int fnOneWayDMATransfer(uintptr_t addr, uint32_t *buf, size_t transfer_size){
	DMAEnv *dmaEnv = (DMAEnv *)addr;
	if (!dmaEnv)
		return -1;

	dmaEnv->complete_flag = 0;

	// DMA Setup
	if (dmaEnv->direction == DMA_DIRECTION_RX) {
		// S2MM
		// Associate data buffer
		writeDMAReg(dmaEnv->base_addr, AXIDMA_REG_ADDR_S2MM_DA, (uint32_t)buf);

		// Set DMA RX Run bit, value 1, DMA register
		writeDMARegFld(dmaEnv->base_addr, AXIDMA_REGFLD_S2MM_DMACR_RUNSTOP, 1);

		// Start DMA Transfer
		writeDMAReg(dmaEnv->base_addr, AXIDMA_REG_ADDR_S2MM_DA_LENGTH, transfer_size);
	}
	else
	{
		// MM2S
		// Associate data buffer
		writeDMAReg(dmaEnv->base_addr, AXIDMA_REG_ADDR_MM2S_SA, (uint32_t)buf);

		// Set DMA RX Run bit, value 1, DMA register
		writeDMARegFld(dmaEnv->base_addr, AXIDMA_REGFLD_MM2S_DMACR_RUNSTOP, 1);

		// Start DMA Transfer
		writeDMAReg(dmaEnv->base_addr, AXIDMA_REG_ADDR_MM2S_SA_LENGTH, transfer_size);
	}

	return 0;
}

/**
 * Check if the DMA transfer previously started has completed.
 *
 * @param addr the address of the DMAEnv instance returned by fnInitDMA
 *
 * @return 1 if the DMA transfer completed, 0 if it is still running
 */
uint8_t fnIsDMATransferComplete(uintptr_t addr) {
	DMAEnv *dmaEnv = (DMAEnv *)addr;
	if (!dmaEnv)
		return 0;

	return dmaEnv->complete_flag;
}

/**
 * Check if the DMA transfer previously started has completed by polling
 * a register.
 *
 * @param addr the physical address of the DMA device
 */
uint8_t fnIsDMATransferCompletePoll(uintptr_t addr) {
	DMAEnv *dmaEnv = (DMAEnv *)addr;
	if (!dmaEnv)
		return 0;
	uint8_t val = readDMAReg(dmaEnv->base_addr, 4);
	return (val & 2);
}

/**
 * Allocate a DMA buffer.
 *
 * @param addr the physical address of the DMA device
 * @param size the size of the DMA buffer
 *
 * @return the address of the newly allocated DMA buffer
 */
void* fnAllocBuffer(uintptr_t addr, size_t size) {

	uint32_t *buf = (uint32_t *)malloc(size);

	return buf;
}

/**
 * Free a DMA buffer.
 *
 * @param addr the physical address of the DMA device
 * @param buf the address of the DMA buffer
 * @param size the size of the DMA buffer
 */
void fnFreeBuffer(uintptr_t addr, void *buf, size_t size) {
	if (buf)
	{
		free(buf);
	}
}
#endif // LINUX_APP
