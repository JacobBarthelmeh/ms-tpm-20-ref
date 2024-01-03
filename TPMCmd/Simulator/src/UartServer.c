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
// This file contains the socket interface to a TPM simulator.
//

#if defined(USE_UART_TRANSPORT) || defined(LINUX_UART)
#ifdef LINUX_UART
    #include <stdio.h>
    #include <stdbool.h>
    #include <stdlib.h>
    #include <stdint.h>

    #ifndef TPM2_SWTPM_HOST
    #define TPM2_SWTPM_HOST "/dev/ttyUSB0"
    #endif
    #ifndef TPM2_SWTPM_BAUD
    #define TPM2_SWTPM_BAUD 115200
    #endif
    #ifndef TPM2_TIMEOUT_SECONDS
    #define TPM2_TIMEOUT_SECONDS    60
    #endif

    #if defined(__unix__) || defined(__APPLE__)
        #define _XOPEN_SOURCE 600
        #include <netdb.h>
        #include <sys/socket.h>         /* used for all socket calls */
        #include <netinet/in.h>         /* used for sockaddr_in6 */
        #include <arpa/inet.h>
        #include <fcntl.h>
        #include <sys/stat.h>
        #include <termios.h>
        #include <signal.h>
        #include <errno.h>
    #else
        #error UART transport not supported
    #endif

    #ifdef _MSC_VER
    #  pragma warning(push, 3)
    #  include <windows.h>
    #  include <winsock.h>
    #  pragma warning(pop)
    typedef int socklen_t;
    #elif defined(__unix__) || defined(__APPLE__)
    #  include <string.h>
    #  include <unistd.h>
    #  include <errno.h>
    #  include <stdint.h>
    #  include <netinet/in.h>
    #  include <sys/socket.h>
    #  include <pthread.h>
    #  define ZeroMemory(ptr, sz) (memset((ptr), 0, (sz)))
    #  define closesocket(x)      close(x)
    #  define INVALID_SOCKET      (-1)
    #  define SOCKET_ERROR        (-1)
    #  define WSAGetLastError()   (errno)
    #  define INT_PTR             intptr_t
    typedef int SOCKET;
    #else
    #  error "Unsupported platform."
    #endif

#else
    // Xilinx includes
    #include <stdio.h>

#ifdef USE_UARTLITE
    #include "xuartlite.h"
    #define SOCKET XUartLite*

    #error device ID's not set
    #define INTC_VEC_ID 
    #define CONSOLE_UART_ID 
    #define APU_UART_ID 
#else
    #include "xuartns550.h"
    #define SOCKET XUartNs550*

    /* used to map to Xilinx device ID's */
    #define INTC_VEC_ID XPAR_INTC_0_UARTNS550_1_VEC_ID
    #define CONSOLE_UART_ID XPAR_MB0_AXI_UART16550_0_DEVICE_ID
    #define APU_UART_ID XPAR_MB0_AXI_UART16550_1_DEVICE_ID
#endif
    #include "xintc.h"
    #include "xparameters.h"
    #include "xil_cache.h"
    #include "xil_testmem.h"

    #include "xspi.h"

    // custom includes
    #include "main.h"
#endif

#include "TpmBuildSwitches.h"
#include <stdbool.h>


#include "TpmTcpProtocol.h"
#include "Manufacture_fp.h"
#include "TpmProfile.h"

#include "Simulator_fp.h"
#include "Platform_fp.h"

#include <wolfssl/options.h>
#include <wolfssl/wolfcrypt/wc_port.h>

#undef  BUFF_SIZE
#define BUFF_SIZE 3000

#ifdef DO_FUZZING
unsigned char inputBuffer[16000];
int  inputBufferIdx = 0;
int  inputBufferSz;
#endif

#ifdef LINUX_UART
    static SOCKET uartConsole;
#else
    // UARTs
#ifdef USE_UARTLITE
    static XUartLite uartConsole;
    static XUartLite uartApuIF;
    extern int XUartLite_SetBaudRate(XUartLite *InstancePtr, u32 BaudRate);
#else
    static XUartNs550 uartConsole;
    static XUartNs550 uartApuIF;
    extern int XUartNs550_SetBaudRate(XUartNs550 *InstancePtr, u32 BaudRate);
#endif

    static volatile int uartApuIFSent = 0;


    // interrupt controller
    static XIntc intc;

    // receive buffer for APU interface serial port
    static uint8_t rxBuff[ BUFF_SIZE ];
    static int rxIdx = 0;

    static unsigned char controlFlow = 0; /* use acks for control flow */
    unsigned char controlFlowOff = 0;
#endif


#ifndef LINUX_UART
static unsigned char SendBuf[BUFF_SIZE];
static int SendBufSz = 0;

static bool DoRead(SOCKET s, char* buffer, int NumBytes)
{
    int res = 0;
    int numGot = 0;

    if (NumBytes > (s->ReceiveBuffer.RequestedBytes - rxIdx)) {
    #ifdef DEBUG_UART
        xil_printf("rxBuf is too small\n\r");
    #endif
        return false;
    }

    do {
        res = s->ReceiveBuffer.RequestedBytes -
            (s->ReceiveBuffer.RemainingBytes + rxIdx);
        if (res == 0) usleep(1); /* allow interupts to happen */

        if (s->ReceiveBuffer.RemainingBytes == 0) {
            xil_printf("Exahusted rxBuff\r\n");
            return false;
        }
    }
    while (res < NumBytes);

    memcpy(buffer, rxBuff + rxIdx, NumBytes);
    rxIdx += NumBytes;
#ifdef DEBUG_UART
    {
        int z;
        xil_printf("[rxIdx %d] Read [%d] : ", rxIdx, NumBytes);
        for (z = 0; z < NumBytes; z++) xil_printf("%02X ", buffer[z] & 0xFF);
        xil_printf("\n\r");
     }
#endif

    return true;
}

static bool FlushWrite(SOCKET s)
{
    int numSent   = 0;
    int remaining = SendBufSz;

    if (SendBufSz > 0) {
        int ChunkSize = 16;
        char buffer[1];
        int  NumBytes = 1;
    #ifdef DEBUG_UART
        xil_printf("Flushing out write buffer\n\r");
    #endif
        do {
            int toSend = (remaining < ChunkSize)? remaining : ChunkSize;

        #ifdef DEBUG_UART
            xil_printf("Sending a burst of %d bytes\n\r", toSend);
        #endif
        #ifdef USE_UARTLITE
            XUartLite_Send(s, SendBuf + numSent, toSend);
        #else
            XUartNs550_Send(s, SendBuf + numSent, toSend);
        #endif
            do {
    	        /* wait for all data to be sent */
            } while (uartApuIFSent != toSend);//s->SendBuffer.RemainingBytes > 0);
        #ifdef DEBUG_UART
            {
                 int z;
                 xil_printf("Sending : ");
                 for (z = 0; z < toSend; z++)
                     xil_printf("%02X ", SendBuf[z + numSent] & 0xFF);
                    xil_printf("\n\r");
            }
        #endif
            numSent   += toSend;
            remaining -= toSend;

            /* get ack */
            usleep(1000); /* the default rx buffer is 16,
                          give it a moment to pull down data */
            DoRead(s, buffer, NumBytes);

            if (remaining <= 0 || numSent >= SendBufSz) break;
        } while (1);

        SendBufSz = 0;
    }
    return true;
}
#endif

//*** ReadBytes()
// This function reads the indicated number of bytes ('NumBytes') into buffer
// from the indicated socket.
bool ReadBytes(SOCKET s, char* buffer, int NumBytes)
{
#ifdef DO_FUZZING
    if (inputBufferIdx + NumBytes >= inputBufferSz)
        return false;

    memcpy(buffer, inputBuffer + inputBufferIdx, NumBytes);
    inputBufferIdx += NumBytes;
#elif defined(LINUX_UART)
    int res = 0;
    int numGot = 0;

    while(numGot < NumBytes)
    {
        fd_set rfds;
        struct timeval tv = { TPM2_TIMEOUT_SECONDS, 0};

        FD_ZERO(&rfds);
        FD_SET(s, &rfds);
        /* use select to wait for data */
        res = select(s + 1, &rfds, NULL, NULL, &tv);
        if (res == 0) {
            printf("Receive timeout\n");
            return false; /* timeout */
        }

        res = read(s, buffer + numGot, NumBytes - numGot);

	if (res == -1) {
		printf("read error\n");
	}
	else {
		printf("read %d bytes\n", res);
	}
        numGot += res;

    }
#else
    if (controlFlow) {
        FlushWrite(s);
    }

    DoRead(s, buffer, NumBytes);
#endif

#if (DBG_LEVEL > 0)
    printf("Read %d bytes\n\r", numGot);
#endif

    return true;
}

#ifndef __IGNORE_STATE__

static uint32_t ServerVersion = 1;

#  define MAX_BUFFER 2000
char InputBuffer[MAX_BUFFER];   //The input data buffer for the simulator.
char OutputBuffer[MAX_BUFFER];  //The output data buffer for the simulator.

struct
{
    uint32_t largestCommandSize;
    uint32_t largestCommand;
    uint32_t largestResponseSize;
    uint32_t largestResponse;
} CommandResponseSizes = {0};

#endif  // __IGNORE_STATE___

//*** WriteBytes()
// This function will send the indicated number of bytes ('NumBytes') to the
// indicated socket
bool WriteBytes(SOCKET s, char* buffer, int NumBytes)
{
    int res;
    int numSent = 0;

#if (DBG_LEVEL > 0)
    printf("Writing %d bytes\n\r", NumBytes);
#endif

#ifdef DO_FUZZING

#elif defined(LINUX_UART)
    while(numSent < NumBytes)
    {
        res = write(s, buffer + numSent, NumBytes - numSent);
        if (res == -1)
        {
        #if (DBG_LEVEL > 0)
            printf("Issue with write\n");
        #endif
            return false;
        }
        numSent += res;
    }
#else
     /* clear receive buffer and then send */
     if (rxIdx > 0) {
         rxIdx = 0;
     #ifdef USE_UARTLITE
         XUartLite_Recv(&uartApuIF, rxBuff, BUFF_SIZE);
     #else
         XUartNs550_Recv(&uartApuIF, rxBuff, BUFF_SIZE);
     #endif
     }

    if (controlFlow) {
        if (NumBytes + SendBufSz > BUFF_SIZE) {
            xil_printf("ran out of room, flushing out\n\r");
            FlushWrite(s);
            sleep(1);
        }
    #ifdef DEBUG_UART
        {
            int z;
            xil_printf("Adding: ");
            for (z = 0; z < NumBytes; z++)
                xil_printf("%02X ", buffer[z] & 0xFF);
            xil_printf("\n\r");
        }
    #endif
        memcpy(SendBuf + SendBufSz, buffer, NumBytes);
        SendBufSz += NumBytes;
    }
    else {
    #ifdef USE_UARTLITE
        XUartLite_Send(s, buffer, NumBytes);
    #else
        XUartNs550_Send(s, buffer, NumBytes);
    #endif
        do {
	        /* wait for all data to be sent */
        } while (uartApuIFSent != NumBytes);//s->SendBuffer.RemainingBytes > 0);
    #ifdef DEBUG_UART
        {
             int z;
             xil_printf("Sending : ");
             for (z = 0; z < NumBytes; z++)
                 xil_printf("%02X ", buffer[z] & 0xFF);
                xil_printf("\n\r");
        }
    #endif
    }

#endif
    return true;
}

#if 0 /* used to test SHA256 operations, from wolfcrypt/test/test.c, can be removed */
#include <wolfssl/wolfcrypt/sha256.h>
#define HEAP_HINT NULL
#define ERROR_OUT(err, eLabel) do { ret = (err); goto eLabel; } while (0)
typedef struct testVector {
    const char*  input;
    const char*  output;
    size_t inLen;
    size_t outLen;
} testVector;

static int sha256_test(void)
{
    wc_Sha256 sha, shaCopy;
    byte      hash[WC_SHA256_DIGEST_SIZE];
    byte      hashcopy[WC_SHA256_DIGEST_SIZE];
    int ret = 0;
    int devId = -1;

    testVector a, b, c;
    testVector test_sha[3];
    int times = sizeof(test_sha) / sizeof(struct testVector), i;

    a.input  = "";
    a.output = "\xe3\xb0\xc4\x42\x98\xfc\x1c\x14\x9a\xfb\xf4\xc8\x99\x6f\xb9"
               "\x24\x27\xae\x41\xe4\x64\x9b\x93\x4c\xa4\x95\x99\x1b\x78\x52"
               "\xb8\x55";
    a.inLen  = XSTRLEN(a.input);
    a.outLen = WC_SHA256_DIGEST_SIZE;

    b.input  = "abc";
    b.output = "\xBA\x78\x16\xBF\x8F\x01\xCF\xEA\x41\x41\x40\xDE\x5D\xAE\x22"
               "\x23\xB0\x03\x61\xA3\x96\x17\x7A\x9C\xB4\x10\xFF\x61\xF2\x00"
               "\x15\xAD";
    b.inLen  = XSTRLEN(b.input);
    b.outLen = WC_SHA256_DIGEST_SIZE;

    c.input  = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    c.output = "\x24\x8D\x6A\x61\xD2\x06\x38\xB8\xE5\xC0\x26\x93\x0C\x3E\x60"
               "\x39\xA3\x3C\xE4\x59\x64\xFF\x21\x67\xF6\xEC\xED\xD4\x19\xDB"
               "\x06\xC1";
    c.inLen  = XSTRLEN(c.input);
    c.outLen = WC_SHA256_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;
    test_sha[2] = c;

    ret = wc_InitSha256_ex(&sha, HEAP_HINT, devId);
    if (ret != 0)
        return -1;
    ret = wc_InitSha256_ex(&shaCopy, HEAP_HINT, devId);
    if (ret != 0) {
        wc_Sha256Free(&sha);
        return -1;
    }

    for (i = 0; i < times; ++i) {
        ret = wc_Sha256Update(&sha, (byte*)test_sha[i].input,
            (word32)test_sha[i].inLen);
        if (ret != 0) {
            ERROR_OUT(i, exit);
        }
        ret = wc_Sha256GetHash(&sha, hashcopy);
        if (ret != 0)
            ERROR_OUT(i, exit);
        ret = wc_Sha256Copy(&sha, &shaCopy);
        if (ret != 0)
            ERROR_OUT(i, exit);
        ret = wc_Sha256Final(&sha, hash);
        if (ret != 0)
            ERROR_OUT(i, exit);
        wc_Sha256Free(&shaCopy);

        if (XMEMCMP(hash, test_sha[i].output, WC_SHA256_DIGEST_SIZE) != 0)
            ERROR_OUT(i, exit);
        if (XMEMCMP(hash, hashcopy, WC_SHA256_DIGEST_SIZE) != 0)
            ERROR_OUT(i, exit);
	xil_printf("\ttest i %d was ok\n\r", i);
    }


exit:

    wc_Sha256Free(&sha);
    wc_Sha256Free(&shaCopy);

    return ret;
}
//    xil_printf("Sha256_test rest = %d\n\r", sha256_test());
#endif


// This is the main function for the sample application.
int StartUartServer(void)
{
    bool continueServing;

    // initialize the uart transport
    wolfCrypt_Init();
    _rpc__Signal_PowerOn(false); /* simulate power on of TPM */
    _rpc__Signal_NvOn();
    _plat__NVEnable(NULL);

#ifdef LINUX_UART
    printf("Started fTPM version %d, listening for commands\n\r",
        ServerVersion);
#else
    xil_printf("Started fTPM version %d, listening for commands\n\r",
        ServerVersion);

    /* open receive buffer for input */
    rxIdx = 0;
#ifdef USE_UARTLITE
    XUartLite_Recv(&uartApuIF, rxBuff, BUFF_SIZE);
#else
    XUartNs550_Recv(&uartApuIF, rxBuff, BUFF_SIZE);
#endif
#endif

    do
    {

#ifdef LINUX_UART
        continueServing = TpmServer(uartConsole);
#else
        if (controlFlowOff) {
        #ifdef DEBUG_UART
            xil_printf("Turning off control flow of send\n\r");
        #endif
            controlFlow = 0;
        }

        continueServing = TpmServer(&uartApuIF);
#endif
    } while(continueServing);

    TpmEndSimulation();
    wolfCrypt_Cleanup();

    // never reach
    return 0;
}

/* convert opaque to 32 bit integer */
static void ato32(const unsigned char* c, unsigned int* wc_u32)
{
    *wc_u32 = ((unsigned int)c[0] << 24) |
              ((unsigned int)c[1] << 16) |
              ((unsigned int)c[2] << 8) |
               (unsigned int)c[3];
}

/* convert 32 bit integer to opaque */
static void c32toa(unsigned int wc_u32, unsigned char* c)
{
    c[0] = (unsigned char)((wc_u32 >> 24) & 0xff);
    c[1] = (unsigned char)((wc_u32 >> 16) & 0xff);
    c[2] = (unsigned char)((wc_u32 >>  8) & 0xff);
    c[3] =  (unsigned char)(wc_u32        & 0xff);
}

//*** WriteUINT32()
// Send 4 byte integer
bool WriteUINT32(SOCKET s, uint32_t val)
{
    unsigned char netVal[4];

    c32toa(val, netVal);
    return WriteBytes(s, (char*)netVal, 4);
}

//*** ReadUINT32()
// Function to read 4 byte integer from socket.
bool ReadUINT32(SOCKET s, uint32_t* val)
{
    unsigned char netVal[4];

    if(!ReadBytes(s, (char*)netVal, 4))
        return false;
    ato32(netVal, val);
    return true;
}

//*** ReadVarBytes()
// Get a uint32-length-prepended binary array.  Note that the 4-byte length is
// in network byte order (big-endian).
bool ReadVarBytes(SOCKET s, char* buffer, uint32_t* BytesReceived, int MaxLen)
{
    unsigned char tmp[4];
    unsigned int  length;
    bool res;

    res = ReadBytes(s, (char*)tmp, 4);
    if(!res)
        return res;
    ato32(tmp, &length);
    *BytesReceived = length;
    if(length > MaxLen)
    {
        return false;
    }
    if(length == 0)
        return true;
    res = ReadBytes(s, buffer, length);
    if(!res)
        return res;
    return true;
}

//*** WriteVarBytes()
// Send a uint32-length-prepended binary array.  Note that the 4-byte length is
// in network byte order (big-endian).
bool WriteVarBytes(SOCKET s, char* buffer, int BytesToSend)
{
    unsigned char tmp[4];
    bool     res;

    c32toa(BytesToSend, tmp);
    res = WriteBytes(s, (char*)tmp, 4);
    if(!res)
        return res;
    res = WriteBytes(s, buffer, BytesToSend);
    if(!res)
        return res;
    return true;
}


void EnableSPI(void)
{
    XIntc_Enable(&intc, XPAR_INTC_0_SPI_0_VEC_ID);
}


void EnableUart(void)
{
    XIntc_Enable(&intc, INTC_VEC_ID);
}

// This function initializes the test system
void InitSystem( void )
{
#if defined(__MICROBLAZE__)
    int ret;
    XSpi* spi;

    // ensure that the caches are disabled, at least for now
    Xil_DCacheDisable();
    Xil_ICacheDisable();

#ifdef USE_UARTLITE
    // initialize the console UART in polling mode
    XUartLite_Initialize( &uartConsole, CONSOLE_UART_ID);

    // initialize the APU interface UART in interrupt mode
    XUartLite_Initialize( &uartApuIF, APU_UART_ID);
    if (XUartLite_SelfTest(&uartApuIF) != XST_SUCCESS) {
        xil_printf("Failed self test for APU IF UART\n\r");
    }
    XUartLite_SetSendHandler( &uartApuIF, SerialHandler, ( void* ) &uartApuIF );
#else
    XUartNs550_Initialize( &uartConsole, CONSOLE_UART_ID);
    XUartNs550_SetBaudRate( &uartConsole, 115200 );

    // initialize the APU interface UART in interrupt mode
    XUartNs550_Initialize( &uartApuIF, APU_UART_ID);
    if (XUartNs550_SelfTest(&uartApuIF) != XST_SUCCESS) {
        xil_printf("Failed self test for APU IF UART\n\r");
    }
    XUartNs550_SetBaudRate( &uartApuIF, 115200 );
    XUartNs550_SetHandler( &uartApuIF, SerialHandler, ( void* ) &uartApuIF );
    XUartNs550_SetOptions( &uartApuIF, ( XUN_OPTION_DATA_INTR | XUN_OPTION_MODEM_INTR | XUN_OPTION_FIFOS_ENABLE ) );
    XUartNs550_SetFifoThreshold( &uartApuIF, XUN_FIFO_TRIGGER_01 );

#endif

#if SPI_BACKED_NV
    spi = NvInitSPI();
#endif

    // initialize the interrupt controller
    ret = XIntc_Initialize( &intc, XPAR_INTC_0_DEVICE_ID);

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler( XIL_EXCEPTION_ID_INT, ( Xil_ExceptionHandler ) XIntc_InterruptHandler, &intc );
    Xil_ExceptionEnable();
    ret = XIntc_Start( &intc, XIN_REAL_MODE );

    // connect the interrupt handler for the APU interface UART, and enable the interrupt
#ifdef USE_UARTLITE
    ret = XIntc_Connect( &intc, INTC_VEC_ID, ( XInterruptHandler ) XUartLite_InterruptHandler, ( void* ) &uartApuIF );
#else
    ret = XIntc_Connect( &intc, INTC_VEC_ID, ( XInterruptHandler ) XUartNs550_InterruptHandler, ( void* ) &uartApuIF );
#endif

    // try connecting up interrupt for SPI too
#if SPI_BACKED_NV
	ret = XIntc_Connect(&intc,
			       XPAR_INTC_0_SPI_0_VEC_ID,
			       (XInterruptHandler)XSpi_InterruptHandler,
			       (void *)spi);
#endif
    EnableUart();

#else
    struct termios tty;
    const char *dev = TPM2_SWTPM_HOST;
    /* Open UART file descriptor */
    uartConsole = open(dev, O_RDWR | O_NOCTTY);
    if (uartConsole < 0) {
        printf("Error opening %s: Error %i (%s)\n",
    	dev, errno, strerror(errno));
        return;
    }
    tcgetattr(uartConsole, &tty);
    cfsetospeed(&tty, TPM2_SWTPM_BAUD);
    cfsetispeed(&tty, TPM2_SWTPM_BAUD);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | (CS8);
    tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY| INLCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~(ONLCR|OCRNL);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~ISTRIP;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tcsetattr(uartConsole, TCSANOW, &tty);

    /* Flush any data in the RX buffer */
    tcflush(uartConsole, TCIOFLUSH);

#endif
}

#ifndef LINUX_UART
// to prevent an assertion
void SerialHandler( void *CallBackRef, u32 Event, unsigned int ByteCount )
{

#ifndef USE_UARTLITE
    if (Event == XUN_EVENT_SENT_DATA) {
        uartApuIFSent = ByteCount;
    }
#endif
}
#endif


//*** TpmServer()
// Processing incoming TPM command requests using the protocol / interface
// defined above.
bool TpmServer(SOCKET s)
{
    uint32_t    length;
    uint32_t    Command;
    uint8_t     locality;
    bool        OK;
    int         result;
    int         clientVersion;
    _IN_BUFFER  InBuffer;
    _OUT_BUFFER OutBuffer;
    //

    EnableUart();
    for(;;)
    {
        char tmp[4];
        OK = ReadBytes(s, tmp, 4);
        // client disconnected (or other error).  We stop processing this client
        // and return to our caller who can stop the server or listen for another
        // connection.
#ifdef DO_FUZZING
        if(!OK)
            return false;
#else
        if(!OK)
            return true;
#endif

        ato32((const unsigned char*)tmp, &Command);
        switch(Command)
        {
            case TPM_SIGNAL_HASH_START:
                _rpc__Signal_Hash_Start();
                break;
            case TPM_SIGNAL_HASH_END:
                _rpc__Signal_HashEnd();
                break;
            case TPM_SIGNAL_HASH_DATA:
                OK = ReadVarBytes(s, InputBuffer, &length, MAX_BUFFER);
                if(!OK)
                    return true;
                InBuffer.Buffer     = (uint8_t*)InputBuffer;
                InBuffer.BufferSize = length;
                _rpc__Signal_Hash_Data(InBuffer);
                break;
            case TPM_SEND_COMMAND:
                OK = ReadBytes(s, (char*)&locality, 1);
                if(!OK)
                    return true;
                OK = ReadVarBytes(s, InputBuffer, &length, MAX_BUFFER);
                if(!OK)
                    return true;
                InBuffer.Buffer      = (uint8_t*)InputBuffer;
                InBuffer.BufferSize  = length;
                OutBuffer.BufferSize = MAX_BUFFER;
                OutBuffer.Buffer     = (_OUTPUT_BUFFER)OutputBuffer;
                // record the number of bytes in the command if it is the largest
                // we have seen so far.
                if(InBuffer.BufferSize > CommandResponseSizes.largestCommandSize)
                {
                    CommandResponseSizes.largestCommandSize = InBuffer.BufferSize;
                    memcpy(&CommandResponseSizes.largestCommand,
                           &InputBuffer[6],
                           sizeof(uint32_t));
                }
                _rpc__Send_Command(locality, InBuffer, &OutBuffer);
                // record the number of bytes in the response if it is the largest
                // we have seen so far.
                if(OutBuffer.BufferSize > CommandResponseSizes.largestResponseSize)
                {
                    CommandResponseSizes.largestResponseSize = OutBuffer.BufferSize;
                    memcpy(&CommandResponseSizes.largestResponse,
                           &OutputBuffer[6],
                           sizeof(uint32_t));
                }
                OK = WriteVarBytes(s, (char*)OutBuffer.Buffer, OutBuffer.BufferSize);
                if(!OK)
                    return true;
                break;
            case TPM_REMOTE_HANDSHAKE:
                OK = ReadBytes(s, (char*)&clientVersion, 4);
                if(!OK)
                    return true;
                if(clientVersion == 0)
                {
                    return true;
                }
                OK &= WriteUINT32(s, ServerVersion);
                OK &= WriteUINT32(
                    s, tpmInRawMode | tpmPlatformAvailable | tpmSupportsPP);
                break;
            case TPM_SET_ALTERNATIVE_RESULT:
                OK = ReadBytes(s, (char*)&result, 4);
                if(!OK)
                    return true;
                // Alternative result is not applicable to the simulator.
                break;
            case TPM_SESSION_END:
                // Client signaled end-of-session
                return true;

            /* custom commands for resetting and saving state */
            case TPM_SIGNAL_RESET:
            #if 0
                TPM_TearDown();
                OK = TPM_Manufacture(1);
                if (OK != 0) {
                    xil_printf("Remanufacture failed\n\r");
                    return false;
                }
            #endif
                controlFlow = 1; /* turn on ack control flow with reset */
                break;
        #if 0
            case TPM_COMMIT_STATE:
                xil_printf("Commiting fTPM state\n\r");
                _plat__NvCommit();
                break;
        #endif
            default:
                return true;
        }
        OK = WriteUINT32(s, 0);
        if(!OK)
            return true;
    }
}
#endif // USE_UART_TRANSPORT || LINUX_UART
// end of main.c
