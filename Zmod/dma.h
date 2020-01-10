/******************************************************************************
 * @file dma.c
 * Function declarations used for DMA transfer.
 *
 * @authors Cristian Fatu
 *
 * @date 2015-Jan-20
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
 *
 * @note
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who             Date        Changes
 * ----- --------------- ----------- --------------------------------------------
 *
 * </pre>
 *
 *****************************************************************************/

#ifndef DMA_H_
#define DMA_H_

#include <stdint.h>

#define DMA_BUF_SIZE 0xFFF0	// 2^16 - 4

enum dma_direction {
	DMA_DIRECTION_TX,
	DMA_DIRECTION_RX,
};

uint32_t fnInitDMA(uintptr_t addr, enum dma_direction direction, int dmaInterrupt);
void fnDestroyDMA(uintptr_t addr);
int fnOneWayDMATransfer(uintptr_t addr, uint32_t *buf, size_t length);
uint8_t fnIsDMATransferComplete(uintptr_t addr);
void* fnAllocBuffer(uintptr_t addr, size_t size);
void fnFreeBuffer(uintptr_t addr, void *buf, size_t size);

#endif /* DMA_H_ */
