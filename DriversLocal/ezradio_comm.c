/**************************************************************************//**
 * @file ezradio_comm.c
 * @brief This file contains the EZRadio communication layer.
 * @version 4.0.0
 ******************************************************************************
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

#include <stdint.h>
#include <stdarg.h>
#include "em_gpio.h"
#include "gpiointerrupt.h"

#include "ezradio_hal.h"
#include "ezradio_comm.h"

/** Can be used to prevent CTS check before any communication command. */
uint8_t ezradio_comm_CtsWentHigh = 0;

/*!
 * Gets a command response from the radio chip
 *
 * @param byteCount     Number of bytes to get from the radio chip
 * @param pData         Pointer to where to put the data
 *
 * @return CTS value
 */
uint8_t ezradio_comm_GetResp(uint8_t byteCount, uint8_t* pData)
{
  uint8_t ctsVal = 0;
  uint16_t errCnt = EZRADIO_CTS_TIMEOUT;

  while (errCnt != 0)      //wait until radio IC is ready with the data
  {
    ezradio_hal_ClearNsel();
    ezradio_hal_SpiWriteByte(0x44);    //read CMD buffer
    ezradio_hal_SpiReadByte(&ctsVal);
    if (ctsVal == 0xFF)
    {
      if (byteCount)
      {
        ezradio_hal_SpiReadData(byteCount, pData);
      }
      ezradio_hal_SetNsel();
      break;
    }
    ezradio_hal_SetNsel();
    errCnt--;
  }

  if (errCnt == 0)
  {
    while(1)
    {
      /* ERROR!!!!  CTS should never take this long. */
      #ifdef ezradio_comm_ERROR_CALLBACK
        ezradio_comm_ERROR_CALLBACK();
      #endif
    }
  }

  if (ctsVal == 0xFF)
  {
    ezradio_comm_CtsWentHigh = 1;
  }

  return ctsVal;
}

/*!
 * Sends a command to the radio chip
 *
 * @param byteCount     Number of bytes in the command to send to the radio device
 * @param pData         Pointer to the command to send.
 */
void ezradio_comm_SendCmd(uint8_t byteCount, uint8_t* pData)
{
    while (!ezradio_comm_CtsWentHigh)
    {
        ezradio_comm_PollCTS();
    }
    ezradio_hal_ClearNsel();
    ezradio_hal_SpiWriteData(byteCount, pData);
    ezradio_hal_SetNsel();
    ezradio_comm_CtsWentHigh = 0;
}

/*!
 * Gets a command response from the radio chip
 *
 * @param cmd           Command ID
 * @param pollCts       Set to poll CTS
 * @param byteCount     Number of bytes to get from the radio chip.
 * @param pData         Pointer to where to put the data.
 */
void ezradio_comm_ReadData(uint8_t cmd, uint8_t pollCts, uint8_t byteCount, uint8_t* pData)
{
    if(pollCts)
    {
        while(!ezradio_comm_CtsWentHigh)
        {
            ezradio_comm_PollCTS();
        }
    }
    ezradio_hal_ClearNsel();
    ezradio_hal_SpiWriteByte(cmd);
    ezradio_hal_SpiReadData(byteCount, pData);
    ezradio_hal_SetNsel();
    ezradio_comm_CtsWentHigh = 0;
}


/*!
 * Gets a command response from the radio chip
 *
 * @param cmd           Command ID
 * @param pollCts       Set to poll CTS
 * @param byteCount     Number of bytes to get from the radio chip
 * @param pData         Pointer to where to put the data
 */
void ezradio_comm_WriteData(uint8_t cmd, uint8_t pollCts, uint8_t byteCount, uint8_t* pData)
{
    if(pollCts)
    {
        while(!ezradio_comm_CtsWentHigh)
        {
            ezradio_comm_PollCTS();
        }
    }
    ezradio_hal_ClearNsel();
    ezradio_hal_SpiWriteByte(cmd);
    ezradio_hal_SpiWriteData(byteCount, pData);
    ezradio_hal_SetNsel();
    ezradio_comm_CtsWentHigh = 0;
}

/*!
 * Waits for CTS to be high
 *
 * @return CTS value
 */
uint8_t ezradio_comm_PollCTS(void)
{
#ifdef RADIO_USER_CFG_USE_GPIO1_FOR_CTS
    while(!ezradio_hal_Gpio1Level())
    {
        /* Wait...*/
    }
    ezradio_comm_CtsWentHigh = 1;
    return 0xFF;
#else
    return ezradio_comm_GetResp(0, 0);
#endif
}

/**
 * Clears the CTS state variable.
 */
void ezradio_comm_ClearCTS()
{
  ezradio_comm_CtsWentHigh = 0;
}

/*!
 * Sends a command to the radio chip and gets a response
 *
 * @param cmdByteCount  Number of bytes in the command to send to the radio device
 * @param pCmdData      Pointer to the command data
 * @param respByteCount Number of bytes in the response to fetch
 * @param pRespData     Pointer to where to put the response data
 *
 * @return CTS value
 */
uint8_t ezradio_comm_SendCmdGetResp(uint8_t cmdByteCount, uint8_t* pCmdData, uint8_t respByteCount, uint8_t* pRespData)
{
    ezradio_comm_SendCmd(cmdByteCount, pCmdData);
    return ezradio_comm_GetResp(respByteCount, pRespData);
}

