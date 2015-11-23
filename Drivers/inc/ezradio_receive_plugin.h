/***************************************************************************//**
 * @file ezradio_receive_plugin.h
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


#ifndef EZRADIO_RECEIVE_PLUGIN_H_
#define EZRADIO_RECEIVE_PLUGIN_H_

#if ( defined EZRADIO_PLUGIN_RECEIVE )

/// EzRadio receive plug-in instance initialization and handler structure.
typedef struct EZRADIODRV_PacketRxHandle
{
  EZRADIODRV_Callback_t userCallback;   ///< User callback.
  uint8_t channel;                      ///< Reception channel.
  uint8_t * pktBuf;                     ///< Pointer to the receive buffer.
} EZRADIODRV_PacketRxHandle_t;


Ecode_t ezradioStartRx(EZRADIODRV_Handle_t radioHandle);

/// Configuration data for EzRadio receive plug-in.
#define EZRADIODRV_RECEIVE_PLUGIN_INIT_DEFAULT                          \
{                               /* Packet RX                        */  \
  NULL,                         /* CallBack                         */  \
  0,                            /* Channel                          */  \
  NULL,                         /* Packet buffer                    */  \
},

#else //#if ( defined EZRADIO_PLUGIN_RECEIVE )

#define EZRADIODRV_RECEIVE_PLUGIN_INIT_DEFAULT

#endif //#if ( defined EZRADIO_PLUGIN_RECEIVE )

#endif /* EZRADIO_RECEIVE_PLUGIN_H_ */
