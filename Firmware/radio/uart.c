// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Embedded Firmware Design, Inc.                               Copyright © 2002
// _____________________________________________________________________________
//
// File:                uart.c
//
// Author:              Mark A. Odell
//
// Description:
// ------------
// An example of a ring buffered, interrupt driven, UART driver for the 8051.
//
// Design Notes:
// -------------
// Define DBG for test only.
//
// Sets up the 8051 standard UART using Timer 1 for a baud rate generator. You
// pick the XTAL frequency by setting XTAL_FREQ to one of the listed speeds
// in the form of defines XTAL_XX_XXXMHZ.
//
// Call initUart(baudRate) with baudRate set to one of the enum's listed in
// uart.h as a type called UartBaudRates. The initUart() function will do
// everything for you except touch the global interrupt enable bit. You must
// enable this when you are ready to start your system.
//
// The UART interrupt preserve/disable/restore operation really needs high
// priority interrupt to provide completely safe interrupt posture control.
//
// All functions that return status return zero for "no failure" and non-zero
// for "some failure". The exception to this rule is for standard C functions
// that I override here (e.g. _getkey() and putchar()).
//
// Note that __RC51__ is unique to Amrai tools but __C51__ is defined by both
// Keil and Amrai. Thus, don't try to distinguish the two toolsets by __C51__.
//
// Thanks to Jon Young for pointing out the benefits of _testbit_().
//
// License:
// --------
// This software is copyrighted by Embedded Firmware Design, Inc. and comes
// with absolutely no warranty whatsoever. Use at your own risk. There are no
// use limitations on this software, commercial or otherwise, as long as
// Embedded Firmware Design, Inc. is fully indemnified from any and all harm
// incurred as a result of using this software.
//
// Revision Control:
// -----------------
// Last committed on  --> $Date: 2002/05/03 21:01:19 $
// This file based on --> $Revision: 1.15 $
// _____________________________________________________________________________
// _____________________________________________________________________________

//
// Port to SDCC by Michael Smith.
//

//
// Header files
//
#include "radio.h"

#include "uart.h"

// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Private Constants
//
// Common XTAL frequencies (Hz).
#define XTAL_11_059MHZ  11059200
#define XTAL_12_000MHZ  12000000
#define XTAL_12_288MHZ  12288000
#define XTAL_16_000MHZ  16000000
#define XTAL_20_000MHZ  20000000
#define XTAL_22_118MHZ  22118400
#define XTAL_24_000MHZ  24000000
#define XTAL_24_500MHZ  24500000

// Choose your frequency.
#define XTAL_FREQ       XTAL_24_500MHZ

// Choice of memory space will affect ring buffer sizes. Usually, RX >= TX size.
//      0 - data
//      1 - idata
//      2 - pdata
//      3 - xdata
#define RING_MEM_SPACE  3
#define RX_RING_SIZE    256
#define TX_RING_SIZE    128

#if RING_MEM_SPACE == 0
#   undef  RING_MEM_SPACE
#   define RING_MEM_SPACE __data
#   if RX_RING_SIZE + TX_RING_SIZE > 0x7F - 0x20
#       error Suggest you drop the size of UART ring buffers or switch to another memory space.
#   endif
#elif RING_MEM_SPACE == 1
#   undef  RING_MEM_SPACE
#   define RING_MEM_SPACE __idata
#   if RX_RING_SIZE + TX_RING_SIZE > 0xFF - 0x20
#       error Suggest you drop the size of UART ring buffers or switch to another memory space.
#   endif
#elif RING_MEM_SPACE == 2
#   undef  RING_MEM_SPACE
#   define RING_MEM_SPACE __pdata
#   if RX_RING_SIZE + TX_RING_SIZE > 0xFF
#       error Suggest you drop the size of UART ring buffers or switch to another memory space.
#   endif
#elif RING_MEM_SPACE == 3
#   undef  RING_MEM_SPACE
#   define RING_MEM_SPACE __xdata
#   if RX_RING_SIZE + TX_RING_SIZE > 0xFFFF - 0x20
#       error Suggest you drop the size of UART ring buffers or switch to another memory space.
#   endif
#else
#   error You must pick a UART ring buffer memory space of xdata, idata, or data.
#endif

// Handy debug control macro.
// XXX won't work for SDCC...
#ifdef DBG
#   define debug
#else
#   define debug while (0)
#endif

#define SCON_8N1            0x40 // SCON.SM1 = 1, SM0,2 = 0
#define SCON_REN            0x10

// Timer 1 only (Timer 0 not used here)
#define TMOD_GATE_T1        0x80
#define TMOD_C_T_T1         0x40
#define TMOD_M1_T1          0x20
#define TMOD_M0_T1          0x10
#define T1_MODE_MASK        (TMOD_GATE_T1 | TMOD_C_T_T1 | TMOD_M1_T1 | TMOD_M0_T1)
#define T1_8BIT_AUTO_RELOAD (TMOD_M1_T1)

#define PCON_SMOD           0x80 // Doubles baud rate timing of Timer 1
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Private Objects
//
typedef __bit		Bool;

// Clip indexes at 2^16 since the 8051 only supports a 16-bit XDATA space.
#if RX_RING_SIZE > 256
    typedef unsigned int  RxRingIndex;
#else
    typedef unsigned char RxRingIndex;
#endif

#if TX_RING_SIZE > 256
    typedef unsigned int  TxRingIndex;
#else
    typedef unsigned char TxRingIndex;
#endif
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Private module functions (prototyped as 'static')
//
static Bool setUartBaudRate(UartBaudRates baudRate);
static Bool putUartChar(U8 outChar);
static Bool getUartChar(U8 *pInChar);
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Private module variables (static module global 's_' prefixed for 's'tatic)
//
static const char __code s_moduleInfo[] = __FILE__ " modified " __DATE__ " at " __TIME__;

// Ring variables.
static          U8            RING_MEM_SPACE s_rxRing[RX_RING_SIZE];
static          U8            RING_MEM_SPACE s_txRing[TX_RING_SIZE];
static          Bool                           s_translateEol;
static volatile Bool                           s_rxRingEmpty;
static volatile Bool                           s_txRingEmpty;
static volatile RxRingIndex   __data           s_rxWrIdx;
static volatile RxRingIndex   __data           s_rxRdIdx;
static volatile TxRingIndex   __data           s_txWrIdx;
static volatile TxRingIndex   __data           s_txRdIdx;
static          UartBaudRates __data           s_currentBaudRate;
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// CPU resouces (for this module, 'r_' prefixed for 'r'egister)
//
#define r_ieEs		ES0
#define r_ieEt1		ET1

#define	r_scon		SCON0
#define r_sconRen	REN0
#define r_sconTi	TI0
#define r_sconRi	RI0
#define r_sbufRx	SBUF0
#define r_sbufTx	SBUF0

#define r_tmod		TMOD

#define r_tcon		TCON
#define r_tconTf1	TF1
#define r_tconTr1	TR1
#define r_tconIe1	IE1
#define r_tconIt1	IT1

#define	r_tl1		TL1
#define r_th1		TH1

#define	r_pcon		PCON

// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  putchar
//
// Description:
// ------------
// Required by the library for any form of console output.
//
// Design Notes:
// -------------
// Do not prototype, prototyped in stdio.h.
//
// Called by library verion of printf(), allows printf() to use our driver.
//
// Simply blocks until there is room in the Tx Ring since printf() may not
// know what to do if putchar() fails.
// _____________________________________________________________________________
//
void putchar(char outChar)
{
    while (putUartChar((U8) outChar));

    if (s_translateEol && '\n' == outChar)
    {
        while (putUartChar('\r'));
    }
}

// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  getchar
//
// Description:
// ------------
// Required by the library for any form of console input.
//
// Design Notes:
// -------------
// Do not prototype, prototyped in stdio.h.
// _____________________________________________________________________________
//
char getchar(void)
{
    U8 __data inChar;

    while (getUartChar(&inChar));

    return inChar;
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  initUart
//
// Description:
// ------------
// Sets up the 8051 Serial Port for UART (SCON.SM1 = 1, Mode 1) operation and
// initializes the ring buffers.
//
// Design Notes:
// -------------
// Byte framing is 8-N-1.
// _____________________________________________________________________________
//
__bit initUart(UartBaudRates baudRate)
{
    // For restartability.
    r_ieEs = 0;

    // Setup the baud rate generator.
    if (setUartBaudRate(baudRate)) return 1;

    // Setup the serial port.
    r_scon = SCON_8N1 | SCON_REN;

    // Setup ring buffers.
    s_rxRingEmpty = 1;
    s_txRingEmpty = 1;
    s_rxWrIdx     = 0;
    s_rxRdIdx     = 0;
    s_txWrIdx     = 0;
    s_txRdIdx     = 0;

    // Printf() translates for us so no need for it here.
    s_translateEol = 0;

    // Start driver.
    r_ieEs = 1;

    return 0;
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  setUartBaudRate
//
// Description:
// ------------
// Attempts to setup Timer 1 to generate baud for the disired <baudRate>. Will
// try TH1 values for both PCON.SMON = 1 and 0. Returns zero for no failure
// in setting the desired baud rate.
//
// Design Notes:
// -------------
// The absolute value of the negative entries are the actual number of Timer
// 1 ticks required for a given baud rate. The 2x table assumes PCON.SMOD is set
// to double the baud rate generated by TH1. Zeros indicate a baud rate could
// not be generated with 4% or less difference. The 1x table assumes PCON.SMOD
// is clear.
// _____________________________________________________________________________
//
static Bool setUartBaudRate(UartBaudRates baudRate)
{
    // XXX TODO this table not applicable for this application

    static const S8 __code baudRateTable2x[NUM_BAUD_RATES] =
    {
    //  1200  2400  9600  19200  38400  57600  115200
#if   XTAL_FREQ == XTAL_11_059MHZ
         -48,  -24,   -6,    -3,     0,    -1,      0
#elif XTAL_FREQ == XTAL_12_000MHZ
         -52,  -26,    0,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_12_288MHZ
         -53,  -27,    0,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_16_000MHZ
         -69,  -35,   -9,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_20_000MHZ
         -87,  -43,  -11,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_22_118MHZ
         -96,  -48,  -12,    -5,    -3,    -2,     -1
#elif XTAL_FREQ == XTAL_24_000MHZ
        -104,  -52,  -13,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_24_500MHZ
           0,    0,    0,     0,     0,     0,      0
#else // Add more if needed.
#   error Pick a known XTAL frequency from list provided in this file.
#endif
    };
    static const S8 __code baudRateTable1x[NUM_BAUD_RATES] =
    {
    //  1200  2400  9600  19200  38400  57600  115200
#if   XTAL_FREQ == XTAL_11_059MHZ
         -24,  -12,   -3,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_12_000MHZ
         -26,  -13,    0,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_12_288MHZ
         -27,  -13,    0,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_16_000MHZ
         -35,  -17,    0,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_20_000MHZ
         -43,  -22,    0,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_22_118MHZ
         -48,  -24,   -6,    -3,     0,    -1,      0
#elif XTAL_FREQ == XTAL_24_000MHZ
         -52,  -26,    0,     0,     0,     0,      0
#elif XTAL_FREQ == XTAL_24_500MHZ
           0,    0,    0,     0,     0,     0,      0
#else
#   error Pick a known XTAL frequency from list provided in this file.
#endif
    };
    S8   __data th1Value;
    Bool ieEs;

    // Look for cases where we don't want to or can't change the baud rate.
    if (BAUD_RATE_NO_CHANGE == baudRate || s_currentBaudRate == baudRate)
    {
        return 0;
    }

    if ((unsigned int) baudRate > NUM_BAUD_RATES)
    {
        return 1;
    }

    // See if we *can* set the desired baud rate.
    r_pcon  |= PCON_SMOD;
    th1Value = baudRateTable2x[baudRate];
    if (!th1Value)
    {
        r_pcon  &= ~PCON_SMOD;
        th1Value = baudRateTable1x[baudRate];
        if (!th1Value) return 1;
    }

    // Preserve and clear serial interrupt enable.
    ieEs = r_ieEs;
    r_ieEs = 0;

    // Must ensure that Timer 1 overflow can never cause an interrupt.
    r_ieEt1 = 0;

    // Save this for getUartOption().
    s_currentBaudRate = baudRate;

    // Config. Timer 1 for baud rate generation.
    r_tconTr1 = 0;
    r_tmod   &= T1_MODE_MASK;
    r_tmod   |= T1_8BIT_AUTO_RELOAD;
    r_th1     = (U8) th1Value;
    r_tconTr1 = 1;

    // Restore previous setting.
    r_ieEs    = ieEs;

    return 0;
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  setUartOption
//
// Description:
// ------------
// Allows driver options to be configured by external users.
//
// Design Notes:
// -------------
// Returns zero for no failure, non-zero otherwise. Baud rate changes during
// UART activity may result in corrupt characters sent or received.
// _____________________________________________________________________________
//
__bit setUartOption(UartOptions option, int value)
{
    switch (option)
    {
        case UART_TRANSLATE_EOL:
            s_translateEol = (Bool) value;
            return 0;

        case UART_BAUD_RATE:
            return setUartBaudRate((UartBaudRates) value);
    }

    return 1;
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  getUartOption
//
// Description:
// ------------
// Allows driver options to be queried by external users.
//
// Design Notes:
// -------------
// Returns zero for no failure, non-zero otherwise..
// _____________________________________________________________________________
//
__bit getUartOption(UartOptions option, int *pValue)
{
    switch (option)
    {
        case UART_TRANSLATE_EOL:
            *pValue = s_translateEol;
            return 0;

        case UART_BAUD_RATE:
            *pValue = s_currentBaudRate;
            return 0;
    }

    return 1;
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  putUartChar
//
// Description:
// ------------
// Driver function to put a character out to the UART via the Tx Ring.
//
// Design Notes:
// -------------
// Will return zero for no failure or non-zero if the ring is full. The Tx
// interrupt will be forced if the Tx Ring is empty to begin a new transmit.
// _____________________________________________________________________________
//
static Bool putUartChar(U8 outChar)
{
    if (s_txRingEmpty || s_txWrIdx != s_txRdIdx)
    {
        Bool preservedIeEs;

        s_txRing[s_txWrIdx] = outChar;

        // Protect this section from the UART ISR.
        preservedIeEs = r_ieEs;
        r_ieEs = 0;

        ++s_txWrIdx;
#if TX_RING_SIZE != 256
        s_txWrIdx %= TX_RING_SIZE;
#endif

        // Set not empty and prime the transmitter.
        if (s_txRingEmpty)
        {
            s_txRingEmpty = 0;
            r_sconTi      = 1;
        }

        r_ieEs = preservedIeEs;

        return 0;
    }

    return 1;
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  getUartChar
//
// Description:
// ------------
// Driver function to get a character from the UART via the Rx Ring.
//
// Design Notes:
// -------------
// Use _getkey() to see if input is avaiable as this function will return
// failure if caller's <pInChar> is null.
// _____________________________________________________________________________
//
static Bool getUartChar(U8 *pInChar)
{
    if (s_rxRingEmpty) return 1;

    // Do not consume char if caller has no storage for it.
    if (pInChar)
    {
        Bool preservedIeEs;

        *pInChar = s_rxRing[s_rxRdIdx];

        // Protect this section from the UART ISR.
        preservedIeEs = r_ieEs;
        r_ieEs = 0;

        ++s_rxRdIdx;
#if RX_RING_SIZE != 256
        s_rxRdIdx %= RX_RING_SIZE;
#endif

        // Detect empty.
        if (s_rxRdIdx == s_rxWrIdx) s_rxRingEmpty = 1;

        r_ieEs = preservedIeEs;

        return 0;
    }

    return 1;
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  uartIsr
//
// Description:
// ------------
// UART Interrupt Service Routing (ISR). More like a task that is scheduled by
// hardware events such as RI or TI.
//
// Design Notes:
// -------------
// TI can be set optionally by the timer tick if there is pending data to be
// transmitted and you don't need or want putchar() to initiate transmission.
//
// Only one recv. char and one send char. per interrupt (worse case) for
// determinism reasons.
//
// If a ring size is exactly 256 bytes, this code will skip the modulo operation
// since the 8051 will wrap an unsigned 8-bit var from 0xFF ==> 0x00. Regardless,
// keep the ring sizes a nice even power of two an C51 will use a simple ANL
// instruction for the modulo.
// _____________________________________________________________________________
//
void uartIsr(void) __interrupt(INTERRUPT_UART0) __using(1)
{
    static U8 __data inChar;

    if (r_sconRi)
    {
        // Buffer the char in hopes that there is room left in the ring. Drop
        // the char if not. In any case, ack. the receive interrupt.
        r_sconRi = 0;
        inChar   = r_sbufRx;
        if (s_rxRingEmpty || s_rxWrIdx != s_rxRdIdx)
        {
            s_rxRing[s_rxWrIdx] = inChar;
            ++s_rxWrIdx;

#if RX_RING_SIZE != 256
            s_rxWrIdx %= RX_RING_SIZE;
#endif
            s_rxRingEmpty = 0;
        }
    }

    if (r_sconTi)
    {
        // If there are chars. left to send, send one and check for empty,
        // otherwise just ack. the transmit interrupt and leave.
        r_sconTi = 0;
        if (s_txRdIdx != s_txWrIdx)
        {
            r_sbufTx = s_txRing[s_txRdIdx];
            ++s_txRdIdx;

#if TX_RING_SIZE != 256
            s_txRdIdx %= TX_RING_SIZE;
#endif
        }
        else
        {
            // Only declare empty after the last SCON.TI event occurs.
            s_txRingEmpty = 1;
        }
    }
}
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Function:  main
//
// Description:
// ------------
// Test entry point for this UART driver.
//
// Design Notes:
// -------------
// This main is non-ISO C compliant since it does not return int.
//
// Do not define DBG when using this driver, this main() is only for driver test.
// _____________________________________________________________________________
//
#ifdef DBG
static const char * code s_pUartBaudRateStrings[] =
{
    "1200"  ,
    "2400"  ,
    "9600"  ,
    "19200" ,
    "38400" ,
    "57600" ,
    "115200"
};

sbit r_ieEa = 0xAF; // IE.EA

void main(void)
{
    static int data value;

    initUart(BAUD_RATE_2400);

    r_ieEa = 1;

    // To silence the linker overlay warning and to demonstrate function usage.
    value = 0;
    setUartOption(UART_TRANSLATE_EOL, value);

    // Status message.
    if (getUartOption(UART_BAUD_RATE, &value))
    {
        printf("Failed getting the baud rate.\n");
    }
    else
    {
        printf("The baud rate is %s.\n", s_pUartBaudRateStrings[value]);
        printf("%s started\n", s_moduleInfo);
    }

    // Echo back (loop-back chars).
    for (;;)
    {
        getchar();
    }
}
#endif
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Revisions:
// ----------
// $Log: uart.c,v $
// Revision 1.15  2002/05/03 21:01:19  mao
// - Now using _testbit_() macro for atomic preservation and clear of UART
//   interrupt enable (credit to Jon Young).
// - Now protect rx and tx ring variables in put and getUartChar(). This fixes
//   false buffer full bug (credit to Jon Young).
//
// Revision 1.14  2002/03/29 16:33:11  mao
// - Switched to proper toolchain constant to distinguish Amrai from Keil builds.
// - Removed unneeded header file.
//
// Revision 1.13  2002/03/28 04:36:02  mao
// - Updated comments.
//
// Revision 1.12  2002/03/28 02:52:57  mao
// - Placed s_moduleInfo into code space, was sucking up precious data space.
//
// Revision 1.11  2002/03/27 16:10:45  mao
// - Added support for AMRAI toolset. Builds but does not work properly yet in
//   simulator.
//
// Revision 1.10  2002/03/27 02:09:51  mao
// - Added license notice.
//
// Revision 1.9  2002/03/27 01:40:44  mao
// - Fixed function header for _getkey().
//
// Revision 1.8  2002/03/17 03:48:08  mao
// - Added some more file header comments about use of driver.
// - Slight style clean up.
// - No longer set EOL translation in initUart() since printf() already does this.
// - Fixed bug. Was not ever clearing s_txRingEmpty bit in putchar().
// - Fixed bug. Was setting s_txRingEmpty in uartIsr() when a char was enqueued
//   instead of transmitted.
// - Fixed bug. Table of baud rates was pointed to improperly.
// - Test main() function now demonstrates all public driver function calls.
//
// Revision 1.7  2002/03/16 04:46:03  mao
// - Freshened file comments.
// - Removed old ifdef C plus plus at end of file.
//
// Revision 1.6  2002/03/16 04:44:01  mao
// - Fixed baud rate typo for 22.118MHz.
// - Added set baud rate function and removed in-lined code in set uart options.
// - Added a non PCON.SMOD = 1 baud rate table and now try to set a baud
//   rate from either table.
// - Fixed table entry bugs for last two XTAL frequencies and moved tables into
//   set uart baud rate function.
// - Fixed recv. ISR bug, now clears Rx Ring empty bit after receiving a char.
// - Added debug table of baud rate strings to print.
//
// Revision 1.5  2002/03/15 04:51:56  mao
// - Fixed bug, debug main() now enables global interrupts.
// - ISR runs fine for both send and receive.
// - Receive seems to work but xmit doesn't show output on debugger screen.
//
// Revision 1.4  2002/03/15 04:29:40  mao
// - Added XTAL frequencies and baud rate table.
// - Added Timer 1 setup.
// - Added save of last set baud rate enum for getUartOptions().
// - Changed some prototypes to return bit instead of int.
// - Fixed putchar() and _getkey() to do as Keil specifies. Removed getchar().
//
// Revision 1.3  2002/03/14 04:21:12  mao
// - Moved ring buffer space and sizing to .c file.
// - Removed rprintf.
// - Implemented more functions.
// - Added set/get options.
// - Test main() now calls all functions, clean build.
// - Added some CPU resources.
// - Baud rate is not yet set.
//
// Revision 1.2  2002/03/13 05:24:20  mao
// - Removed blank line
//
// Revision 1.1.1.1  2002/03/13 05:20:10  mao
// - Initial import
// _____________________________________________________________________________

