/***************************************************************************//**
 * @file ezradio_receive_plugin.c
 * @brief EzRadio receive plug-in managed by the plug-in manager if enabled.
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

#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"

#include "ezradio_cmd.h"
#include "ezradio_prop.h"
#include "ezradio_hal.h"
#include "ezradio_api_lib.h"
#include "ezradio_api_lib_add.h"
#include "ezradio_plugin_manager.h"
#include "ezradio_receive_plugin.h"
#include "serial.h"
#include "radio-config-wds-gen.h"


#if ( defined EZRADIO_PLUGIN_RECEIVE )

#if ( ( defined EZRADIO_PLUGIN_AUTO_ACK ) && ( defined EZRADIO_PLUGIN_TRANSMIT ) )
Ecode_t ezradioTransmitAutoAck(EZRADIODRV_Handle_t radioHandle);
#endif

/**************************************************************************//**
 * @brief EzRadio driver receive plug-in handler routine.
 *
 *  @param radioHandle EzRadio driver instance handler.
 *  @param radioReplyHandle EZRadio communication reply handler.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioHandleReceivePlugin( EZRADIODRV_Handle_t radioHandle, EZRADIODRV_ReplyHandle_t radioReplyHandle, uint16_t IRQ_tick)
{
  static uint8_t pktBufCount=0;
  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

  if(( radioReplyHandle->GET_INT_STATUS.MODEM_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_MODEM_PEND_PREAMBLE_DETECT_PEND_BIT) )
  {
  	pktBufCount = 0;
  	if ( radioHandle->packetRx.userCallback != NULL )
    {
      radioHandle->packetRx.userCallback( radioHandle, ECODE_EMDRV_EZRADIODRV_PREAMBLE_DETECT ,IRQ_tick);
    }
  }

  /* Check if Pkt Rxd IT is received */
  if (( radioReplyHandle->GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)||
      ( radioReplyHandle->GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_RX_FIFO_ALMOST_FULL_PEND_BIT))
  {
  	uint8_t len;
    ezradio_cmd_reply_t radioReplyLocal;
    if( radioReplyHandle->GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
    {
    	/* Check how many bytes we received. */
    	ezradio_fifo_info(0u, &radioReplyLocal);
    	len = radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT;
    }
    else
    {
    	len = RADIO_CONFIGURATION_DATA_PKT_RX_THRESHOLD;
    }
    /* Read out the RX FIFO content. */
    if((pktBufCount+len) <= radioHandle->packetRx.pktBufLen)
    {
    	if(len!= 0)
    	{
    		ezradio_read_rx_fifo(len, &(radioHandle->packetRx.pktBuf[pktBufCount]));
    	}
    	pktBufCount += len;
			if( (pktBufCount == (radioHandle->packetRx.pktBuf[2]+3))&&										// if number of bytes matches packet length
					( radioReplyHandle->GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)&&
				 ( radioHandle->packetRx.userCallback != NULL ))
			{
				radioHandle->packetRx.userCallback( radioHandle, ECODE_EMDRV_EZRADIODRV_PACKET_RX,IRQ_tick);
				pktBufCount = 0;
			}
    }

    if( radioReplyHandle->GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
    {
    	/* Note: Workaround for some FIFO issue. */
    	pktBufCount = 0;
    	ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_RX_BIT, NULL);
    	ezradioStartRx( radioHandle );
    }

#if ( ( defined EZRADIO_PLUGIN_AUTO_ACK ) && ( defined EZRADIO_PLUGIN_TRANSMIT ) )
    /* Transmit auto acknowledge packet if enabled */
    if (radioHandle->autoAck.ackMode  == ezradiodrvAutoAckSkipONe)
    {
      radioHandle->autoAck.ackMode  = ezradiodrvAutoAckImmediate;
    }
    else if (radioHandle->autoAck.ackMode  == ezradiodrvAutoAckImmediate)
    {
      ezradioTransmitAutoAck(radioHandle);
    }
#endif

    return ECODE_EMDRV_EZRADIODRV_OK;
  }

  /* Reset FIFO */
  //ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_RX_BIT, NULL);

  return ECODE_EMDRV_EZRADIODRV_OK;
}

/**************************************************************************//**
 * @brief Set Radio to RX mode, packet length is always coming from
 *        the radio configuration.
 *
 *  @param radioHandle Handler of the EzRadio driver instance where packet
 *         buffer, callback and channel configurations are set.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioStartRx(EZRADIODRV_Handle_t radioHandle)
{
#if 0	// channel should already be set, try speeding up
	ezradio_start_rx_fast();
#else
	/* Start Receiving packet, channel 0, START immediately, Packet n bytes long */
    ezradio_start_rx(radioHandle->packetRx.channel, 0u, 0u,
                  EZRADIO_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                  EZRADIO_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
                  EZRADIO_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX );
#endif
    return ECODE_EMDRV_EZRADIODRV_OK;
}

#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )
