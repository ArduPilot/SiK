/***************************************************************************//**
 * @file uartdrv_config.h
 * @brief UARTDRV configuration file.
 * @version 4.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
#ifndef __SILICON_LABS_UARTDRV_CONFIG_H__
#define __SILICON_LABS_UARTDRV_CONFIG_H__

/***************************************************************************//**
 * @addtogroup EM_Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup UARTDRV
 * @{
 ******************************************************************************/

/// Maximum concurrent receive operations
#ifndef EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS
#define EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS    6
#endif

/// Maximum concurrent transmit operations
#ifndef EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS
#define EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS    6
#endif

/// Set to 1 to enable hardware flow control
#ifndef EMDRV_UARTDRV_HW_FLOW_CONTROL_ENABLE
#define EMDRV_UARTDRV_HW_FLOW_CONTROL_ENABLE    1
#endif

/// Maximum number of driver instances. This maximum applies only when EMDRV_UARTDRV_HW_FLOW_CONTROL_ENABLE = 1.
#ifndef EMDRV_UARTDRV_MAX_DRIVER_INSTANCES
#define EMDRV_UARTDRV_MAX_DRIVER_INSTANCES      4
#endif

/// UART software flow control code: request peer to start TX
#ifndef UARTDRV_FC_SW_XON
#define UARTDRV_FC_SW_XON                       0x11
#endif

/// UART software flow control code: request peer to stop TX
#ifndef UARTDRV_FC_SW_XOFF
#define UARTDRV_FC_SW_XOFF                      0x13
#endif

/** @} (end addtogroup UARTDRV) */
/** @} (end addtogroup EM_Drivers) */

#endif /* __SILICON_LABS_UARTDRV_CONFIG_H__ */
