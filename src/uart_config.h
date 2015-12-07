/*
 * uart_config.h
 *
 *  Created on: 08/10/2014
 *      Author: a
 */

#ifndef UART_CONFIG_H_
#define UART_CONFIG_H_

#include <stddef.h>
#include "em_cmu.h"
#include "em_usart.h"
#include "em_leuart.h"
#include "board_rfd900e.h"

// Please don't touch settings below :)

/* Results of Disk Functions */
typedef enum {
	UART_Serial = 0,
  UART_LAST
} UART_SELECT_TypeDef;


typedef enum
{
  Databits8 = 8,
  Databits9 = 9
} Databits_TypeDef;

typedef enum
{
  NoParity   = 0,
  EvenParity ,
  OddParity
} Parity_TypeDef;

typedef enum
{
  Stopbits1 = 1,
  Stopbits2
} Stopbits_TypeDef;

typedef struct {
    uint32_t            BAUD;       // baud rate
    USART_TypeDef*      USART;      // usart, set to null if leuart
    LEUART_TypeDef*     LEUART;     // leuart , set to null if usart
    unsigned long       location;   /* Location of Usart - eg USART_ROUTE_LOCATION_LOC1 */
    Databits_TypeDef    Databits;   // bits
    Parity_TypeDef      Parity;     // parity
    Stopbits_TypeDef    Stopbits;   // stopbits
    CMU_Clock_TypeDef   clock;      // le/Us art clock - eg cmuClock_USART1
    GPIO_SELECT_TypeDef GPIOCTS;			// CTS pin
    GPIO_SELECT_TypeDef GPIORTS;			// RTS pin
} UART_Config_t;


// set up serial port usart1 loc #2

#define UART_SETUP  \
{57600, USART1   , NULL, USART_ROUTE_LOCATION_LOC2, Databits8 , NoParity , Stopbits1 , cmuClock_USART1,GPIO_PIN_CTS,GPIO_PIN_RTS}


#endif /* UART_CONFIG_H_ */
