/***************************************************************************//**
 * @file ezradio_pn9_plugin.h
 * @brief EzRadio PN9 plug-in managed by the plug-in manager if enabled.
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


#ifndef EZRADIO_PN9_PLUGIN_H_
#define EZRADIO_PN9_PLUGIN_H_

#if ( defined EZRADIO_PLUGIN_PN9 )

/// EzRadio PN9 plug-in instance initialization and handler structure.
typedef struct EZRADIODRV_Pn9Handle
{
  uint8_t channel;      ///< PN9 transmit channel.
} EZRADIODRV_Pn9Handle_t;

Ecode_t ezradioStartPn9(EZRADIODRV_Handle_t radioHandle );
Ecode_t ezradioStopPn9( void );

/// Configuration data for EzRadio PN9 plug-in.
#define EZRADIODRV_PN9_PLUGIN_INIT_DEFAULT                         \
{                               /* PN9                              */  \
  0,                            /* Channel                          */  \
},

#else //#if ( defined EZRADIO_PLUGIN_PN9 )
#define EZRADIODRV_PN9_PLUGIN_INIT_DEFAULT
#endif  //#if ( defined EZRADIO_PLUGIN_PN9 )


#endif /* EZRADIO_PN9_PLUGIN_H_ */
