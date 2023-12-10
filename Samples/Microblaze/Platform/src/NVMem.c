/* Microsoft Reference Implementation for TPM 2.0
 *
 *  The copyright in this software is being made available under the BSD License,
 *  included below. This software may be subject to other third party and
 *  contributor rights, including patent rights, and no such rights are granted
 *  under this license.
 *
 *  Copyright (c) Microsoft Corporation
 *
 *  All rights reserved.
 *
 *  BSD License
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this list
 *  of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or
 *  other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ""AS IS""
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//** Description
//
//    This file contains the NV read and write access methods.  This implementation
//    uses RAM/file and does not manage the RAM/file as NV blocks.
//    The implementation may become more sophisticated over time.
//

//** Includes and Local
#include <memory.h>
#include <string.h>
#include <assert.h>
#include "Platform.h"
#if FILE_BACKED_NV
#  include <stdio.h>
static FILE* s_NvFile           = NULL;
static int   s_NeedsManufacture = FALSE;
#endif
#if SPI_BACKED_NV
#ifndef SDT
#include "xintc.h"		/* Interrupt controller device driver */
#else
#include "xinterrupt_wrap.h"
#endif
#include "xspi.h"
#include "main.h"
static int   s_NeedsManufacture = FALSE;
#ifndef SDT
static XIntc InterruptController;
#endif
static XSpi Spi;
#endif

#include <wolfssl/options.h>
#include <wolfssl/wolfcrypt/aes.h>

#if SPI_BACKED_NV

#define ATMEL_PAGE_SIZE   264  /* Page Size */
//#define ATMEL_PAGE_SIZE   256  /* Page Size */

#ifndef SDT
#define SPI_DEVICE_ID   XPAR_SPI_0_DEVICE_ID
#define INTC_DEVICE_ID  XPAR_INTC_0_DEVICE_ID
#define SPI_INTR_ID     XPAR_INTC_0_SPI_0_VEC_ID
#endif

volatile int TransferInProgress;
XSpi_Config *config = NULL;	/* Pointer to Configuration data */
int ErrorCount;

#ifndef SDT
static int SetupInterruptSystem(XSpi *SpiPtr)
{

	int Status;

	Status = XIntc_Initialize(&InterruptController, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect a device driver handler that will be called when an interrupt
	 * for the device occurs, the device driver handler performs the
	 * specific interrupt processing for the device
	 */
	Status = XIntc_Connect(&InterruptController,
			       SPI_INTR_ID,
			       (XInterruptHandler)XSpi_InterruptHandler,
			       (void *)SpiPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Start the interrupt controller such that interrupts are enabled for
	 * all devices that cause interrupts, specific real mode so that
	 * the SPI can cause interrupts through the interrupt controller.
	 */
	Status = XIntc_Start(&InterruptController, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	XIntc_Enable(&InterruptController, SPI_INTR_ID);


	/*
	 * Initialize the exception table.
	 */
	Xil_ExceptionInit();

	/*
	 * Register the interrupt controller handler with the exception table.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				     (Xil_ExceptionHandler) XIntc_InterruptHandler,
				     &InterruptController);

	/*
	 * Enable non-critical exceptions.
	 */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}
#endif

void SpiHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount)
{
	/*
	 * Indicate the transfer on the SPI bus is no longer in progress
	 * regardless of the status event.
	 */
	TransferInProgress = FALSE;

	/*
	 * If the event was not transfer done, then track it as an error.
	 */
	if (StatusEvent != XST_SPI_TRANSFER_DONE) {
		ErrorCount++;
	}
}


static int SpiAtmelFlashWaitForFlashNotBusy(XSpi *SpiPtr)
{
	u8 StatusReg;
    u8 WriteBuffer[2];
    u8 ReadBuffer[2];

	/*
	 * Prepare the Write Buffer.
	 */
	WriteBuffer[0] = 0xD7; /* ATMEL_COMMAND_STATUSREG_READ */
	WriteBuffer[1] = 0xFF;

	/*
	 * Prepare the Read Buffer.
	 */
	ReadBuffer[0] = 0;//ATMEL_INITBYTE;
	ReadBuffer[1] = 0;//ATMEL_INITBYTE;

	while (1) {

		/*
		 * Transmit the data.
		 */
		TransferInProgress = TRUE;
		XSpi_Transfer(&Spi, WriteBuffer, ReadBuffer,
			      0x2);//ATMEL_STATUS_READ_BYTES);

		/*
		 * Wait for the transmission to be complete and
		 * check if there are any errors in the transaction.
		 */
		while (TransferInProgress);
		if (ErrorCount != 0) {
			ErrorCount = 0;
			return XST_FAILURE;
		}

		StatusReg = ReadBuffer[1];

        /* mask and check the flash SR is ready bit */
		if ((StatusReg & 0x80)) {
			break;
		}
	}

	return XST_SUCCESS;
}


/* returns 0 on success, -1 on failure */
static int SPIReadID(int* id1, int* id2, int* id3)
{
    int ret = 0;
    u8 id[4];
    u8 WriteBuffer[4];

    WriteBuffer[0]     = 0x9F;
    TransferInProgress = TRUE;
    XSpi_Transfer(&Spi, WriteBuffer, id, 4);
	while (TransferInProgress);
	if (ErrorCount != 0) {
	    ErrorCount = 0;
	    return XST_FAILURE;
	}

    if (id1) {
        *id1 = id[0];
    }
    if (id2) {
        *id2 = id[1];
    }
    if (id3) {
        *id3 = id[2];
    }
    return ret;
}


#define AES_KEY_SIZE 16
byte key[] = {
};
byte iv[]  = {
};


/* returns 0 on success, if 'set' is hot then the decrypted buffer is stored in
 * global s_NV */
static int SPIDecrypt(const byte* in, int sz, byte* tag, int tagSz, int set)
{
    u8 decrypted[NV_MEMORY_SIZE];
    int ret;
    Aes aes;

    if (sz > NV_MEMORY_SIZE) {
        return -1;
    }

    wc_AesInit(&aes, NULL, INVALID_DEVID);

    ret = wc_AesGcmSetKey(&aes, key, AES_KEY_SIZE);
#ifdef DEBUG_SPI
    xil_printf("ret of aes gcm set key = %d\n\r", ret);
    {
        int z;
        xil_printf("tag found : ");
        for (z = 0; z < 16; z++) {
            xil_printf("%02x", tag[z]);
        }
        xil_printf("\n\r");
    }
#endif

    if (ret == 0) {
        ret = wc_AesGcmDecrypt(&aes, decrypted, in, sz, iv, sizeof(iv), tag,
            tagSz, NULL, 0);
#ifdef DEBUG_SPI
        xil_printf("\n\rresults of aes gcm decrypt = %d\n\r", ret);
#endif
    }
    wc_AesFree(&aes);

    if (set && ret == 0) {
        memcpy(s_NV, decrypted, sz);
    }

    return ret;
}

static long NvSPIRead(int set)
{
	u16 Index;
    u8 ReadBfrPtr[NV_MEMORY_SIZE + AES_BLOCK_SIZE];
    u8 ReadPagePtr[ATMEL_PAGE_SIZE + 4];
    u8 WriteBuffer[4];
    u32 Page = 0;
    u32 Ofst = 0;
    int ret;

#ifdef DEBUG_SPI
    xil_printf("Calling nv SPI read\n\r");
#endif

    EnableSPI();
    for (Index = 0; Index < NV_MEMORY_SIZE + AES_BLOCK_SIZE;
            Index += ATMEL_PAGE_SIZE) {

        /* Read a page at a time using the default 264 page address */
	    WriteBuffer[0] = 0x03; /* ATMEL_COMMAND_READ */
	    WriteBuffer[1] = (u8) (Page >> 7);
	    WriteBuffer[2] = (u8) ((Page << 1) | (Ofst >> 8));
	    WriteBuffer[3] = (u8) (Ofst & 0xFF);
        Page++;

        #ifdef DEBUG_SPI
        {
            int z;
            xil_printf("READ COMMAND : ");
            for (z = 0; z < 4; z++)
                xil_printf("%02X", WriteBuffer[z]);
            xil_printf("\n\r");
            memset(ReadPagePtr, 0, ATMEL_PAGE_SIZE + 4);
        }
        #endif

	    TransferInProgress = TRUE;
	    XSpi_Transfer(&Spi, WriteBuffer, ReadPagePtr,
		      ATMEL_PAGE_SIZE + 4);

	    while (TransferInProgress);
	    if (ErrorCount != 0) {
		    ErrorCount = 0;
		    return XST_FAILURE;
	    }

    #ifdef DEBUG_SPI
        if (Index < 1000) {
            int z;
            xil_printf("NV SPI READ Page (index = %d / %d) : ", Index,
                NV_MEMORY_SIZE);
            for (z = 0; z < ATMEL_PAGE_SIZE; z++)
                xil_printf("%02x", ReadPagePtr[z + 4]);
            xil_printf("\n\r");
        }
    #endif
        memcpy(ReadBfrPtr + Index, ReadPagePtr + 4, ATMEL_PAGE_SIZE);
    }

    ret = SPIDecrypt(ReadBfrPtr, NV_MEMORY_SIZE,
        ReadBfrPtr + NV_MEMORY_SIZE, AES_BLOCK_SIZE, set);

    EnableUart();
	return ret;
}


/* returns null (0) on failure and pointer to SPI device on success */
XSpi* NvInitSPI()
{
    int Status;

    config = XSpi_LookupConfig(XPAR_MB0_AXI_QUAD_SPI_0_DEVICE_ID);
	if (config == NULL) {
    #ifdef DEBUG_SPI
        xil_printf("config device not found\n\r");
    #endif
		return NULL;
	}

	Status = XSpi_CfgInitialize(&Spi, config,
				    config->BaseAddress);
	if (Status != XST_SUCCESS) {
        if (Status == XST_DEVICE_IS_STARTED) {
        #ifdef DEBUG_SPI
            xil_printf("Device was already started\n\r");
        #endif
            return &Spi;
        }
    #ifdef DEBUG_SPI
        xil_printf("config initialize failed\n\r");
    #endif
		return NULL;
	}
#ifdef DEBUG_SPI
    xil_printf("SPI initialized, base address = %X\n\r", config->BaseAddress);
#endif
    return &Spi;
}


//*** NvSPIOpen()
// This function opens the SPI connection used to hold the NV image.
//  Return Type: int
//  >= 0        success
//  -1          error
static int NvSPIOpen(void)
{
    int ret = 0;
	int Status;
	u32 Index;
	u32 Address;

#ifdef DEBUG_SPI
    xil_printf("Calling NvSPIOpen()\n\r");
#endif
    if (NvInitSPI() == NULL) {
        return XST_FAILURE;
    }

	/*
	 * Setup the handler for the SPI that will be called from the interrupt
	 * context when an SPI status occurs, specify a pointer to the SPI
	 * driver instance as the callback reference so the handler is able to
	 * access the instance data.
	 */
	XSpi_SetStatusHandler(&Spi, &Spi, (XSpi_StatusHandler)SpiHandler);

	/*
	 * Set the SPI device as a master and in manual slave select mode such
	 * that the slave select signal does not toggle for every byte of a
	 * transfer, this must be done before the slave select is set.
	 */
	Status = XSpi_SetOptions(&Spi, XSP_MASTER_OPTION |
				 XSP_MANUAL_SSELECT_OPTION |
				 XSP_CLK_PHASE_1_OPTION |
				 XSP_CLK_ACTIVE_LOW_OPTION);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Select the slave on the SPI bus so that the Atmel Flash device can be
	 * read and written using the SPI bus.
	 */
	Status = XSpi_SetSlaveSelect(&Spi, 0x01);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Start the SPI driver so that interrupts and the device are enabled.
	 */
	Status = XSpi_Start(&Spi);
	if (Status != XST_SUCCESS && Status != XST_DEVICE_IS_STARTED) {
        xil_printf("XSpi_Start did not return XST_SUCCESS, %d\r\n", Status);
        ret = -1;
    }

    EnableSPI();
    if (ret == 0) {
        int id1, id2, id3;

        SPIReadID(&id1, &id2, &id3);
        xil_printf("SPI ID found = 0x%x, 0x%x, 0x%x\n\r", id1, id2, id3);
    }
    EnableUart();

    return ret;
}


/* returns 0 on success, encrypts in buffer and places result in out and tag */
static int SPIEncrypt(const byte* in, byte* out, int sz, byte* tag, int tagSz)
{
    Aes aes;
    int ret;

    wc_AesInit(&aes, NULL, INVALID_DEVID);
    ret = wc_AesGcmSetKey(&aes, key, AES_KEY_SIZE);
    if (ret == 0) {
        ret = wc_AesGcmEncrypt(&aes, out, in, sz, iv, sizeof(iv), tag, tagSz,
            NULL, 0);
    }

#ifdef DEBUG_SPI
    xil_printf("ret of aes gcm encrypt = %d\n\r", ret);
    {
        int z;
        xil_printf("tag created : ");
        for (z = 0; z < tagSz; z++) {
            xil_printf("%02x", tag[z]);
        }
        xil_printf("\n\r");
    }
#endif

    wc_AesFree(&aes);
    return ret;
}


/* erase the SPI contents, returns 0 on success */
static int SPIErase(void)
{
    int ret = 0;
    int Status;
    u8 WriteBuffer[4];
    u32 Page;

    for (Page = 0; (Page * ATMEL_PAGE_SIZE) < NV_MEMORY_SIZE + 16; Page++) {
	    WriteBuffer[0] = 0x81; /* ATMEL_COMMAND_PAGE_ERASE */;
	    WriteBuffer[1] = (u8) (Page >> 7);
	    WriteBuffer[2] = (u8) (Page << 1);
	    WriteBuffer[3] = 0xFF; /* padding */

        TransferInProgress = TRUE;
	    XSpi_Transfer(&Spi, WriteBuffer, NULL, 0x4);

	    while (TransferInProgress);
	    if (ErrorCount != 0) {
		    ErrorCount = 0;
		    return -1;
	    }

    	Status = SpiAtmelFlashWaitForFlashNotBusy(&Spi);
    	if (Status != XST_SUCCESS) {
    		return -1;
    	}
    }

    return ret;
}


//*** NvSPICommit()
// Write all of the contents of the NV image to SPI device.
//  Return Type: int
//      TRUE(1)         success
//      FALSE(0)        failure
static int NvSPICommit(void)
{
    int OK = 1;
	u8 WriteCmd[5];
	int Status;
    u8 WriteBuffer[NV_MEMORY_SIZE + AES_BLOCK_SIZE + 4];
    u32 Page = 0;
    u32 Ofst = 0;
    int ofst;
    u8 cipher[NV_MEMORY_SIZE + AES_BLOCK_SIZE];
    int ret;

    xil_printf("NV SPI Commit\n\r");
    // If NV SPI is not available, return failure
    if(config == NULL)
        return 0;

    // Write RAM data to NV
#ifdef DEBUG_SPI
    xil_printf("NV SPI Commit\n\r");
#endif

    /* take over interrupts for use with SPI */
    EnableSPI();

    //Encrypt the state before writing out
    ret = SPIEncrypt(s_NV, cipher, NV_MEMORY_SIZE, cipher + NV_MEMORY_SIZE,
        AES_BLOCK_SIZE);
    if (ret != 0) {
    #ifdef DEBUG_SPI
        xil_printf("Error %d encrypting buffer for SPI\n\r", ret);
    #endif
        return 0;
    }

    ret = SPIErase();
    if (ret != 0) {
        return 0;
    }

    for (ofst = 0; ofst < NV_MEMORY_SIZE + 16;) {
        int toCopy = ATMEL_PAGE_SIZE;
	    u16 idx = 0;

        /* Set upper bound on amount of data to copy */
        if (toCopy > (NV_MEMORY_SIZE + 16) - ofst) {
            toCopy = (NV_MEMORY_SIZE + 16) - ofst;
        }

        /* Setting address based on default 264 page size */
    	WriteBuffer[idx++] = 0x82; /* ATMEL_COMMAND_WRITE */
    	WriteBuffer[idx++] = (u8) (Page >> 7);
    	WriteBuffer[idx++] = (u8) ((Page << 1) | (Ofst >> 8));
    	WriteBuffer[idx++] = (u8) (Ofst & 0xFF);
    	Page++;

        /* append data to be written to SPI device */
        memcpy(WriteBuffer + idx, cipher + ofst, toCopy);

    #ifdef DEBUG_SPI
        if (ofst < 1000) {
            int z;
            xil_printf("NV SPI Writing to address 0x%02x%02x%02x toCopy = %d: ",
                WriteBuffer[1], WriteBuffer[2], WriteBuffer[3], toCopy);
            for (z = 0; z < ATMEL_PAGE_SIZE; z++) {
                xil_printf("%02X", WriteBuffer[4+z]);
            }
            xil_printf("\n\r");
        }
    #endif

        TransferInProgress = TRUE;
        XSpi_Transfer(&Spi, WriteBuffer, NULL, 4 + toCopy);

    	while (TransferInProgress);
    	if (ErrorCount != 0) {
    		ErrorCount = 0;
    		return XST_FAILURE;
    	}

    	Status = SpiAtmelFlashWaitForFlashNotBusy(&Spi);
    	if (Status != XST_SUCCESS) {
    		return XST_FAILURE;
    	}

        /* increase offset and loop to write next page */
        ofst += toCopy;
    }

    EnableUart();
    xil_printf("NV SPI Commit return OK\n\r");
    return OK;
}

#endif //SPI_BACKED_NV

//*** _plat__NvErrors()
// This function is used by the simulator to set the error flags in the NV
// subsystem to simulate an error in the NV loading process
LIB_EXPORT void _plat__NvErrors(int recoverable, int unrecoverable)
{
    s_NV_unrecoverable = unrecoverable;
    s_NV_recoverable   = recoverable;
}

//***_plat__NVEnable()
// Enable NV memory.
//
// This version just pulls in data from a file. In a real TPM, with NV on chip,
// this function would verify the integrity of the saved context. If the NV
// memory was not on chip but was in something like RPMB, the NV state would be
// read in, decrypted and integrity checked.
//
// The recovery from an integrity failure depends on where the error occurred. It
// it was in the state that is discarded by TPM Reset, then the error is
// recoverable if the TPM is reset. Otherwise, the TPM must go into failure mode.
//  Return Type: int
//      0           if success
//      > 0         if receive recoverable error
//      <0          if unrecoverable error
LIB_EXPORT int _plat__NVEnable(
    void* platParameter  // IN: platform specific parameters
)
{
    NOT_REFERENCED(platParameter);  // to keep compiler quiet
                                    //
    // Start assuming everything is OK
    s_NV_unrecoverable = FALSE;
    s_NV_recoverable   = FALSE;
#if SPI_BACKED_NV
#ifdef DEBUG_SPI
     xil_printf("enabling NVM config = %p\n\r", config);
#endif

    // Initialize all the bytes in the ram copy of the NV
    _plat__NvMemoryClear(0, NV_MEMORY_SIZE);

    if (NvSPIOpen() == 0) {
        int ret;

        /* if able to decrypt data from SPI then set it, otherwise start new */
        ret = NvSPIRead(1);
        if(ret != 0) {
            NvSPICommit();
            s_NeedsManufacture = TRUE;
        #ifdef DEBUG_SPI
            xil_printf("Need to manufacture NV\n\r");
        #endif
        }
    #ifdef DEBUG_SPI
        else {
            xil_printf("Using existing NV state\n\r");
        }
    #endif
    }
    else {
        xil_printf("Failed to open SPI\n\r");
        return 1;
    }
    assert(NULL != config);  // Just in case we are broken for some reason.
#endif
    // NV contents have been initialized and the error checks have been performed. For
    // simulation purposes, use the signaling interface to indicate if an error is
    // to be simulated and the type of the error.
    if(s_NV_unrecoverable)
        return -1;
    return s_NV_recoverable;
}

//***_plat__NVDisable()
// Disable NV memory
LIB_EXPORT void _plat__NVDisable(int delete  // IN: If TRUE, delete the NV contents.
)
{
    if (delete)
        (void)SPIErase();
    return;
}

//***_plat__IsNvAvailable()
// Check if NV is available
//  Return Type: int
//      0               NV is available
//      1               NV is not available due to write failure
//      2               NV is not available due to rate limit
LIB_EXPORT int _plat__IsNvAvailable(void)
{
    int retVal = 0;
    // NV is not available if the TPM is in failure mode
    if(!s_NvIsAvailable)
        retVal = 1;
    else
        retVal = (config == NULL);
    return retVal;
}

//***_plat__NvMemoryRead()
// Function: Read a chunk of NV memory
LIB_EXPORT void _plat__NvMemoryRead(unsigned int startOffset,  // IN: read start
                                    unsigned int size,  // IN: size of bytes to read
                                    void*        data   // OUT: data buffer
)
{
    assert(startOffset + size <= NV_MEMORY_SIZE);
    memcpy(data, &s_NV[startOffset], size);  // Copy data from RAM
    return;
}

//*** _plat__NvIsDifferent()
// This function checks to see if the NV is different from the test value. This is
// so that NV will not be written if it has not changed.
//  Return Type: int
//      TRUE(1)         the NV location is different from the test value
//      FALSE(0)        the NV location is the same as the test value
LIB_EXPORT int _plat__NvIsDifferent(unsigned int startOffset,  // IN: read start
                                    unsigned int size,  // IN: size of bytes to read
                                    void*        data   // IN: data buffer
)
{
    return (memcmp(&s_NV[startOffset], data, size) != 0);
}

//***_plat__NvMemoryWrite()
// This function is used to update NV memory. The "write" is to a memory copy of
// NV. At the end of the current command, any changes are written to
// the actual NV memory.
// NOTE: A useful optimization would be for this code to compare the current
// contents of NV with the local copy and note the blocks that have changed. Then
// only write those blocks when _plat__NvCommit() is called.
LIB_EXPORT int _plat__NvMemoryWrite(unsigned int startOffset,  // IN: write start
                                    unsigned int size,  // IN: size of bytes to write
                                    void*        data   // OUT: data buffer
)
{
    if(startOffset + size <= NV_MEMORY_SIZE)
    {
        memcpy(&s_NV[startOffset], data, size);  // Copy the data to the NV image
        return TRUE;
    }
    return FALSE;
}

//***_plat__NvMemoryClear()
// Function is used to set a range of NV memory bytes to an implementation-dependent
// value. The value represents the erase state of the memory.
LIB_EXPORT void _plat__NvMemoryClear(
    unsigned int start,  // IN: clear start
    unsigned int size    // IN: number of bytes to clear
)
{
    assert(start + size <= NV_MEMORY_SIZE);
    // In this implementation, assume that the erase value for NV is all 1s
    memset(&s_NV[start], 0xff, size);
}

//***_plat__NvMemoryMove()
// Function: Move a chunk of NV memory from source to destination
//      This function should ensure that if there overlap, the original data is
//      copied before it is written
LIB_EXPORT void _plat__NvMemoryMove(
    unsigned int sourceOffset,  // IN: source offset
    unsigned int destOffset,    // IN: destination offset
    unsigned int size           // IN: size of data being moved
)
{
    assert(sourceOffset + size <= NV_MEMORY_SIZE);
    assert(destOffset + size <= NV_MEMORY_SIZE);
    memmove(&s_NV[destOffset], &s_NV[sourceOffset], size);  // Move data in RAM
    return;
}

//***_plat__NvCommit()
// This function writes the local copy of NV to NV for permanent store. It will write
// NV_MEMORY_SIZE bytes to NV. If a file is use, the entire file is written.
//  Return Type: int
//  0       NV write success
//  non-0   NV write fail
LIB_EXPORT int _plat__NvCommit(void)
{
    return (NvSPICommit() ? 0 : 1);
}

//***_plat__SetNvAvail()
// Set the current NV state to available.  This function is for testing purpose
// only.  It is not part of the platform NV logic
LIB_EXPORT void _plat__SetNvAvail(void)
{
    s_NvIsAvailable = TRUE;
    return;
}

//***_plat__ClearNvAvail()
// Set the current NV state to unavailable.  This function is for testing purpose
// only.  It is not part of the platform NV logic
LIB_EXPORT void _plat__ClearNvAvail(void)
{
    s_NvIsAvailable = FALSE;
    return;
}

//*** _plat__NVNeedsManufacture()
// This function is used by the simulator to determine when the TPM's NV state
// needs to be manufactured.
LIB_EXPORT int _plat__NVNeedsManufacture(void)
{
    return s_NeedsManufacture;
}
