#ifndef uart_H_INCLUDED
#define uart_H_INCLUDED
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Embedded Firmware Design, Inc.                               Copyright © 2002
// _____________________________________________________________________________
//
// File:                uart.h
//
// Author:              Mark A. Odell
//
// Description:
// ------------
// Contains external definitions for uart.c.
//
// Design Notes:
// -------------
// Only external declarations, constants, macros, etc. exist here.
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
// Last committed on  --> $Date: 2002/03/28 04:35:21 $
// This file based on --> $Revision: 1.7 $
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Header files
//
// (none)
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Public Constants
//
typedef enum UartOptions
{
    UART_TRANSLATE_EOL,
    UART_BAUD_RATE    ,

    NUM_UART_OPTIONS
} UartOptions;

typedef enum UartBaudRates
{
    BAUD_RATE_9600     ,
    BAUD_RATE_38400    ,
    BAUD_RATE_57600    ,
    BAUD_RATE_115200   ,
    BAUD_RATE_230400   ,

    NUM_BAUD_RATES     ,
    BAUD_RATE_NO_CHANGE
} UartBaudRates;

#define uartInitUart      initUart
#define uartSetUartOption setUartOption
#define uartGetUartOption getUartOption

// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Macro:  UART_ASSERT
//
// Description:
// ------------
// Assert macro that uses this UART driver.
//
// Design Notes:
// -------------
// This multi-statement safe macro is based upon the comp.lang.c C-FAQ,
// specifically, http://www.eskimo.com/~scs/C-faq/q10.4.html.
// _____________________________________________________________________________
//
#define UART_ASSERT(cond) do                                                 \
{                                                                            \
    if (!(cond))                                                             \
    {                                                                        \
        initUart(BAUD_RATE_NO_CHANGE);                                       \
        printf("\n%s %s%u%s", s_moduleInfo, "(line ", __LINE__);             \
        printf(") : assertion failed:\n    " #cond);                         \
        for (;;);                                                            \
    }                                                                        \
} while (0)
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Public Objects
//
// (none)
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Public functions (prototyped here only)
//
__bit uartInitUart(UartBaudRates baudRate);
__bit uartSetUartOption(UartOptions option, int value);
__bit uartGetUartOption(UartOptions option, int *pValue);
bool iskey(void);
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Public (Global) variables (minimize, 'g_' prefixed for 'g'lobal)
//
// (none)
// _____________________________________________________________________________
// _____________________________________________________________________________
//
// Revisions:
// ----------
// $Log: uart.h,v $
// Revision 1.7  2002/03/28 04:35:21  mao
// - Added header file inclusion enforcement defines. Callers now use module
//   abbr. prefixed named functions.
//
// Revision 1.6  2002/03/28 02:52:17  mao
// - No longer disable recv. in assert macro since it probably doesn't really help
//   and it would mean putting REN in this header file for the sole purpose to
//   allow external users to use uart assert.
//
// Revision 1.5  2002/03/27 02:14:38  mao
// - Added acknowledgement.
//
// Revision 1.4  2002/03/27 02:09:51  mao
// - Added license notice.
//
// Revision 1.3  2002/03/15 04:27:20  mao
// - Fixed bug in baud rates enum.
// - Changed prototypes to return bit instead of int.
//
// Revision 1.2  2002/03/14 04:19:30  mao
// - Moved ring buffer space and sizing to .c file.
// - Added UART options and values.
// - Added public function prototypes.
// - Removed rprintf.
//
// Revision 1.1.1.1  2002/03/13 05:20:10  mao
// Initial import
// _____________________________________________________________________________
//
#endif // uart_H_INCLUDED
