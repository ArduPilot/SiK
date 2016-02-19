/***************************************************************************//**
 * @file ezradio_transmit_plugin.c
 * @brief EzRadio transmit plug-in managed by the plug-in manager if enabled.
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
#include "ezradio_transmit_plugin.h"
#include "timer.h"

#if ( defined EZRADIO_PLUGIN_TRANSMIT )

Ecode_t ezradioStartTx(EZRADIODRV_Handle_t radioHandle, bool updateFields, EZRADIODRV_PacketLengthConfig_t pktLengthConf, uint8_t *pioRadioPacket,uint16_t *TxTick);

#if ( ( defined EZRADIO_PLUGIN_AUTO_ACK ) && ( defined EZRADIO_PLUGIN_RECEIVE ) )
Ecode_t ezradioHandleAutoAckPlugin( EZRADIODRV_Handle_t radioHandle, EZRADIODRV_ReplyHandle_t radioReplyHandle );
#endif

/**************************************************************************//**
 * @brief EZRadioDrv transmit plug-in handler routine.
 *
 *  @param radioHandle EzRadio driver instance handler.
 *  @param radioReplyHandle EZRadio communication reply handler.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioHandleTransmitPlugin( EZRADIODRV_Handle_t radioHandle, EZRADIODRV_ReplyHandle_t radioReplyHandle )
{
  uint32_t CBParam = 0;
	if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

#if 0
  if ( radioReplyHandle->GET_INT_STATUS.CHIP_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_CHIP_PEND_CMD_ERROR_PEND_BIT )
	{
  	CBParam |= ECODE_EMDRV_EZRADIODRV_CHIP_ERROR;
  }
  if ( radioReplyHandle->GET_INT_STATUS.CHIP_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_CHIP_PEND_STATE_CHANGE_PEND_BIT )
  {
  	CBParam |= ECODE_EMDRV_EZRADIODRV_STATE_CHANGE;
  }
#endif
  if ( radioReplyHandle->GET_INT_STATUS.CHIP_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_CHIP_PEND_FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND_BIT )
  {
  	ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_TX_BIT, NULL);
  }
// REQUEST_DEVICE_STATE

  if ( radioReplyHandle->GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT)
	{
  	CBParam |= ECODE_EMDRV_EZRADIODRV_TX_NEAR_EMPTY;
  }

  if ( radioReplyHandle->GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT )
  {
#if ( ( defined EZRADIO_PLUGIN_AUTO_ACK ) && ( defined EZRADIO_PLUGIN_RECEIVE ) )
    /* Handle auto acknowledge packet if enabled */
    if ( (radioHandle->packetTx.pktType == ezradiodrvPacketTypeAutoAck) &&
         (radioHandle->autoAck.ackMode  == ezradiodrvAutoAckImmediate) )
    {
      ezradioHandleAutoAckPlugin(radioHandle, radioReplyHandle);
    }
    else
#endif //#if ( ( defined EZRADIO_PLUGIN_AUTO_ACK ) && ( defined EZRADIO_PLUGIN_RECEIVE ) )
    	CBParam |= ECODE_EMDRV_EZRADIODRV_PACKET_SENT;
  }
	if((0 != CBParam)&&(radioHandle->packetTx.userCallback != NULL ))
  {
    radioHandle->packetTx.userCallback( radioHandle, CBParam,0);
  }
  return ECODE_EMDRV_EZRADIODRV_OK;
}
#if 0
/**************************************************************************//**
 * @brief Start basic transmission. Radio transmits with data only in the first
 *        field in this case.
 *
 *  @param radioHandle Handler of the EzRadio driver instance where packet length
 *         and channel configurations are set.
 *  @param packetLength Length of the basic packet.
 *  @param pioRadioPacket Pointer to the packet buffer.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioStartTransmitBasic(EZRADIODRV_Handle_t radioHandle, uint16_t packetLength, uint8_t *pioRadioPacket)
{
  /* Set default field configuration from the generated header file */
  EZRADIODRV_PacketLengthConfig_t pktLengthConf =
      { ezradiodrvTransmitLenghtCustomPacketLen, packetLength };

  bool updateFields = false;

  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }


  /* Radio field condfig has to be updated if the configuration is different from the previous */
  if ( (radioHandle->packetTx.lenConfig.lenMode     != ezradiodrvTransmitLenghtCustomPacketLen) ||
       (radioHandle->packetTx.lenConfig.pktLen      != packetLength ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f1 != (uint8_t)packetLength ) )
  {
    updateFields = true;
    pktLengthConf.fieldLen.f1 = (uint8_t)packetLength;
    pktLengthConf.fieldLen.f2 = 0u;
    pktLengthConf.fieldLen.f3 = 0u;
    pktLengthConf.fieldLen.f4 = 0u;
    pktLengthConf.fieldLen.f5 = 0u;
  }

  ezradioStartTx(radioHandle, updateFields, pktLengthConf, pioRadioPacket);

  return ECODE_EMDRV_EZRADIODRV_OK;
}

/**************************************************************************//**
 * @brief Start transmission using the packet parameters from the generated
 *        configuration header file.
 *
 *  @param radioHandle Handler of the EzRadio driver instance where packet
 *         length and channel configurations are set.
 *  @param pioRadioPacket Pointer to the packet buffer.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioStartTransmitConfigured(EZRADIODRV_Handle_t radioHandle, uint8_t *pioRadioPacket)
{
  /* Set default field configuration from the generated header file */
  EZRADIODRV_PacketLengthConfig_t pktLengthConf =
      { ezradiodrvTransmitLenghtDefault, RADIO_CONFIG_DATA_MAX_PACKET_LENGTH, RADIO_CONFIG_DATA_FIELD_LENGTH };

  bool updateFields = false;

  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

  /* Radio field config has to be updated if the configuration is different from the previous */
  if ( (radioHandle->packetTx.lenConfig.lenMode     != ezradiodrvTransmitLenghtDefault) ||
       (radioHandle->packetTx.lenConfig.pktLen      != pktLengthConf.pktLen ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f1 != pktLengthConf.fieldLen.f1 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f2 != pktLengthConf.fieldLen.f2 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f3 != pktLengthConf.fieldLen.f3 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f4 != pktLengthConf.fieldLen.f4 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f5 != pktLengthConf.fieldLen.f5 ) )
  {
    updateFields = true;
  }

  ezradioStartTx(radioHandle, updateFields, pktLengthConf, pioRadioPacket);

  return ECODE_EMDRV_EZRADIODRV_OK;
}
#endif
/**************************************************************************//**
 * @brief Start transmission using the packet parameters from @ref pktLengthConf.
 *
 *  @param radioHandle Handler of the EzRadio driver instance where packet length
 *         and channel configurations are set.
 *  @param pktLengthConf Packet length configuration structure configuration of
 *         the custom packet.
 *  @param pioRadioPacket Pointer to the packet buffer.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioStartTransmitCustom(EZRADIODRV_Handle_t radioHandle, EZRADIODRV_PacketLengthConfig_t pktLengthConf, uint8_t *pioRadioPacket,uint16_t *TxTick)
{
  bool updateFields = false;

  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

  /* Radio field config has to be updated if the configuration is different from the previous */
  if ( (radioHandle->packetTx.lenConfig.lenMode     != ezradiodrvTransmitLenghtCustomFieldLen) ||
       (radioHandle->packetTx.lenConfig.pktLen      != pktLengthConf.pktLen ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f1 != pktLengthConf.fieldLen.f1 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f2 != pktLengthConf.fieldLen.f2 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f3 != pktLengthConf.fieldLen.f3 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f4 != pktLengthConf.fieldLen.f4 ) ||
       (radioHandle->packetTx.lenConfig.fieldLen.f5 != pktLengthConf.fieldLen.f5 ) )
  {
    updateFields = true;
  }

  return(ezradioStartTx(radioHandle, updateFields, pktLengthConf, pioRadioPacket,TxTick));

}
#if 0
/**************************************************************************//**
 * @brief Start a smart transmission. Depending on the information in the
 *        @ref pktLengthConf parameter the function decides which transmission
 *        function has to be called.
 *
 *  @param radioHandle Handler of the EzRadio driver instance where packet
 *         length and channel configurations are set.
 *  @param pktLengthConf Packet length configuration structure configuration
 *         of the custom packet.
 *  @param pioRadioPacket Pointer to the packet buffer.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioStartTransmitSmart(EZRADIODRV_Handle_t radioHandle, EZRADIODRV_PacketLengthConfig_t pktLengthConf, uint8_t *pioRadioPacket)
{
  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

  if (pktLengthConf.lenMode == ezradiodrvTransmitLenghtCustomPacketLen)
  {
    ezradioStartTransmitBasic(radioHandle, pktLengthConf.pktLen, pioRadioPacket);
  }
  else if (pktLengthConf.lenMode == ezradiodrvTransmitLenghtDefault)
  {
    ezradioStartTransmitConfigured(radioHandle, pioRadioPacket);
  }
  else if (pktLengthConf.lenMode == ezradiodrvTransmitLenghtCustomFieldLen)
  {
    ezradioStartTransmitCustom(radioHandle, pktLengthConf, pioRadioPacket);
  }

  return ECODE_EMDRV_EZRADIODRV_OK;
}
#endif

/**************************************************************************//**
 * @brief Start a default transmission using the packet information previously
 *        set to @ref radioHandle. Depending on the information in the
 *        @ref radioHandle parameter the function decides which transmission
 *        function has to be called.
 *
 *  @param radioHandle Handler of the EzRadio driver instance where packet length and channel configurations are set.
 *  @param pioRadioPacket Pointer to the packet buffer.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioStartTransmitDefault(EZRADIODRV_Handle_t radioHandle, uint8_t *pioRadioPacket)
{
  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

  ezradioStartTransmitSmart(radioHandle, radioHandle->packetTx.lenConfig, pioRadioPacket);

  return ECODE_EMDRV_EZRADIODRV_OK;
}

/**************************************************************************//**
 * @brief Set Radio to TX mode with packet configuration in @ref pktLengthConf.
 *        Field configurations are updated if necessary.
 *
 *  @param radioHandle Handler of the ezradiodrv instance where packet length
 *         and channel configurations are set.
 *  @param updateFields If true, packet field configurations are updated in
 *         radio, left unchanged otherwise.
 *  @param pktLengthConf Packet length configuration structure configuration
 *         of the packet.
 *  @param pioRadioPacket Pointer to the packet buffer.
 *
 *  @return
 *    @ref ECODE_EMDRV_EZRADIODRV_OK on success. On failure an appropriate EZRADIODRV
 *    @ref Ecode_t is returned.
 *****************************************************************************/
Ecode_t ezradioStartTx(EZRADIODRV_Handle_t radioHandle, bool updateFields, EZRADIODRV_PacketLengthConfig_t pktLengthConf, uint8_t *pioRadioPacket,uint16_t *TxTick)
{
  ezradio_cmd_reply_t ezradioReply;

  if ( radioHandle == NULL )
  {
    return ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE;
  }

  /* Request and check radio device state */
  // TODO need to add in FRR register reading for curr_state to reduce delays here
  // not adding back in may cause packets to be lost
  uint8_t CurrState;
#if 1
  ezradio_frr_b_read(1,&ezradioReply);
  CurrState = ezradioReply.FRR_B_READ.FRR_B_VALUE;
#else
  ezradio_request_device_state(&ezradioReply);
  CurrState = ezradioReply.REQUEST_DEVICE_STATE.CURR_STATE;
#endif
  if (CurrState == EZRADIO_CMD_REQUEST_DEVICE_STATE_REP_CURR_STATE_MAIN_STATE_ENUM_TX) {
    return ECODE_EMDRV_EZRADIODRV_TRANSMIT_FAILED;
  }
  //if(CurrState != EZRADIO_CMD_REQUEST_DEVICE_STATE_REP_CURR_STATE_MAIN_STATE_ENUM_RX)
  {
 //   return ECODE_EMDRV_EZRADIODRV_TRANSMIT_FAILED;
  }
  /* Update radio packet filed configurations if requested */
  if (updateFields)
  {
    radioHandle->packetTx.lenConfig.lenMode = pktLengthConf.lenMode;
    radioHandle->packetTx.lenConfig.pktLen  = 0;
    radioHandle->packetTx.lenConfig.pktLen  += radioHandle->packetTx.lenConfig.fieldLen.f1 = pktLengthConf.fieldLen.f1;
    radioHandle->packetTx.lenConfig.pktLen  += radioHandle->packetTx.lenConfig.fieldLen.f2 = pktLengthConf.fieldLen.f2;
    radioHandle->packetTx.lenConfig.pktLen  += radioHandle->packetTx.lenConfig.fieldLen.f3 = pktLengthConf.fieldLen.f3;
    radioHandle->packetTx.lenConfig.pktLen  += radioHandle->packetTx.lenConfig.fieldLen.f4 = pktLengthConf.fieldLen.f4;
    radioHandle->packetTx.lenConfig.pktLen  += radioHandle->packetTx.lenConfig.fieldLen.f5 = pktLengthConf.fieldLen.f5;

#if 0 // pass length into start tx to avoid setting properties
#if (RADIO_CONFIGURATION_DATA_RADIO_CHIP_FAMILY == RADIO_CHIP_FAMILY_EZRADIOPRO)

    ezradio_set_property(
        EZRADIO_PROP_GRP_ID_PKT, 2u,
        EZRADIO_PROP_GRP_INDEX_PKT_FIELD_1_LENGTH,
        0, pktLengthConf.fieldLen.f1
        );

    ezradio_set_property(
        EZRADIO_PROP_GRP_ID_PKT, 2u,
        EZRADIO_PROP_GRP_INDEX_PKT_FIELD_2_LENGTH,
        0, pktLengthConf.fieldLen.f2
        );

    ezradio_set_property(
        EZRADIO_PROP_GRP_ID_PKT, 2u,
        EZRADIO_PROP_GRP_INDEX_PKT_FIELD_3_LENGTH,
        0, pktLengthConf.fieldLen.f3
        );

    ezradio_set_property(
        EZRADIO_PROP_GRP_ID_PKT, 2u,
        EZRADIO_PROP_GRP_INDEX_PKT_FIELD_4_LENGTH,
        0, pktLengthConf.fieldLen.f4
        );

    ezradio_set_property(
        EZRADIO_PROP_GRP_ID_PKT, 2u,
        EZRADIO_PROP_GRP_INDEX_PKT_FIELD_5_LENGTH,
        0, pktLengthConf.fieldLen.f5
        );
#endif //#if !(RADIO_CONFIG_DATA_RADIO_TYPE == 4455)
#endif
  }

  /* Fill the TX fifo with data */
  // assume FIFO is empty, fill only to max size of buffer, ints
  // will take care of putting the rest in
  uint16_t FIFOLen = radioHandle->packetTx.lenConfig.pktLen;
  if(FIFOLen > EZRADIO_FIFO_SIZE){FIFOLen = EZRADIO_FIFO_SIZE;}
  ezradio_write_tx_fifo(FIFOLen, pioRadioPacket);

  /* Start sending packet, channel 0, START immediately, Packet n bytes long, go READY when done */
  *TxTick = timer2_tick();
#if 1
  ezradio_start_tx(radioHandle->packetTx.channel, 0x30,  radioHandle->packetTx.lenConfig.pktLen);
#else
  ezradio_start_tx(radioHandle->packetTx.channel, 0x30,  0u);
#endif
  return ECODE_EMDRV_EZRADIODRV_OK;
}

#endif //#if ( defined EZRADIO_PLUGIN_TRANSMIT )
