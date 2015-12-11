/***************************************************************************//**
 * @file ezradio_plugin_manager.c
 * @brief This file contains the plug-in manager for the EZRadio and
 *        EZRadioPRO chip families.
 * @version 3.20.13
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "em_gpio.h"
#include "em_prs.h"
#include "em_cmu.h"
#include "gpiointerrupt.h"
#include "ustimer.h"

#include "ezradio_cmd.h"
#include "ezradio_prop.h"
#include "ezradio_hal.h"
#include "ezradio_api_lib.h"
#include "ezradio_api_lib_add.h"
#include "ezradio_plugin_manager.h"
#include "printfl.h"


/* Radio configuration data array. */
const uint8_t Radio_Configuration_Data_Array[]  = \
                        RADIO_CONFIGURATION_DATA_ARRAY;

/* Radio interrupt receive flag */
bool    ezradioIrqReceived = false;
static EZRADIODRV_Handle_t myradioHandle = NULL;

void ezradioPowerUp(void);

#if ( defined EZRADIO_PLUGIN_TRANSMIT )
Ecode_t ezradioHandleTransmitPlugin( EZRADIODRV_Handle_t radioHandle, EZRADIODRV_ReplyHandle_t radioReplyHandle );
#endif

#if ( defined EZRADIO_PLUGIN_RECEIVE )
Ecode_t ezradioHandleReceivePlugin( EZRADIODRV_Handle_t radioHandle, EZRADIODRV_ReplyHandle_t radioReplyHandle );
#endif

#if ( defined EZRADIO_PLUGIN_CRC_ERROR )
Ecode_t ezradioHandleCrcErrorPlugin( EZRADIODRV_Handle_t radioHandle, EZRADIODRV_ReplyHandle_t radioReplyHandle );
#endif

/**************************************************************************//**
 * @brief  Radio nIRQ GPIO interrupt.
 *****************************************************************************/
void GPIO_EZRadio_INT_IRQHandler( uint8_t pin )
{
  (void)pin;

  /* Sign radio interrupt received */
  ezradioIrqReceived = true;
}

/**************************************************************************//**
 * @brief  Power up the Radio.
 *****************************************************************************/
void ezradioPowerUp(void)
{
  /* Hardware reset the chip */
  ezradio_reset();

  /* Initialize ustimer */
  USTIMER_Init();
  /* Delay for preconfigured time */
  USTIMER_Delay( RADIO_CONFIG_DATA_RADIO_DELAY_AFTER_RESET_US );
  /* Deinit ustimer */
  USTIMER_DeInit();

}

/**************************************************************************//**
 * @brief  Radio Initialization.
 *
 * @param[in] handle EzRadio driver instance handler.
 *****************************************************************************/
void ezradioInit( EZRADIODRV_Handle_t handle )
{
  uint16_t wDelay;
  myradioHandle = handle;
  /* Initialize radio GPIOs and SPI port */

#if ( defined EZRADIO_PLUGIN_PTI )
  ezradio_hal_GpioInit( GPIO_EZRadio_INT_IRQHandler, true );
#else
  ezradio_hal_GpioInit( GPIO_EZRadio_INT_IRQHandler, false );
#endif
  ezradio_hal_SpiInit();

  /* Power Up the radio chip */
  ezradioPowerUp();

  /* Load radio configuration */
  while (EZRADIO_CONFIG_SUCCESS != ezradio_configuration_init(Radio_Configuration_Data_Array))
  {
    /* Error hook */
#ifdef ERROR_HOOK
    ERROR_HOOK;
#else
    printf("ERROR: Radio configuration failed!\n");
#endif
    for (wDelay = 0x7FFF; wDelay--; ) ;

    /* Power Up the radio chip */
    ezradioPowerUp();
  }

  /* Read ITs, clear pending ones */
  ezradio_get_int_status(0u, 0u, 0u, NULL);

}

/**************************************************************************//**
 * @brief  EzRadio plugin manager state machine handler. This function should
 *         be called in the application in an infinite loop, in order to
 *         manage the EzRadio plug-ins properly.
 *
 * @param[in] handle EzRadio driver instance handler.
 *****************************************************************************/
Ecode_t ezradioPluginManager( EZRADIODRV_Handle_t radioHandle )
{
  /* EZRadio response structure union */
  ezradio_cmd_reply_t radioReplyData;
  EZRADIODRV_ReplyHandle_t radioReplyHandle = &radioReplyData;

  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

  /* Check is radio interrupt is received. */
  if (ezradioIrqReceived)
  {
    /* Accept interrupt before clearing IT in the radio, so prevent race conditions. */
    ezradioIrqReceived = false;

    /* Read ITs, clear all pending ones */
    ezradio_get_int_status(0x0, 0x0, 0x0, radioReplyHandle);

#if ( defined EZRADIO_PLUGIN_TRANSMIT )
    ezradioHandleTransmitPlugin( radioHandle, radioReplyHandle );
#endif

#if ( defined EZRADIO_PLUGIN_RECEIVE )
    ezradioHandleReceivePlugin( radioHandle, radioReplyHandle );
#endif

#if ( defined EZRADIO_PLUGIN_CRC_ERROR )
    ezradioHandleCrcErrorPlugin( radioHandle, radioReplyHandle );
#endif

  }

  return ECODE_EMDRV_EZRADIODRV_OK;
}

/**************************************************************************//**
 * @brief  Resets both the TX and RX FIFOs.
 *****************************************************************************/
void ezradioResetTRxFifo(void)
{
#if ( defined EZRADIO_PLUGIN_RECEIVE )
  ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_RX_BIT, NULL);
#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )

#if ( defined EZRADIO_PLUGIN_TRANSMIT )
  ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_TX_BIT, NULL);
#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )
}

