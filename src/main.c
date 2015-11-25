/***************************************************************************//**
 * @file main.c
 * @brief EZRadio simple trx example
 *
 * This example shows how to easily implement a simple trx code for your
 * controller using EZRadio or EZRadioPRO devices.
 *
 * @version 4.0.0
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

#include <stdint.h>
#include <stdbool.h>
//#include <stdio.h>
#include <stdlib.h>

#include "spidrv.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "rtcdriver.h"

#include "ezradio_cmd.h"
#include "ezradio_api_lib.h"
#include "ezradio_plugin_manager.h"
#include "ezradio_prop.h"
//#include "display.h"
//#include "textdisplay.h"
//#include "retargettextdisplay.h"
//#include "bspconfig.h"
//#include "image.h"
#include "AtCmds.h"
#include "xprintf.h"
#include "radio_old.h"
#include "tdm.h"
#include "timer.h"
#include "serial.h"

// ******************** defines and typedefs *************************

#ifdef printf
#undef printf
#endif
#define printf(...) xprintf(__VA_ARGS__)

#define LED_PORT gpioPortF
#define GREENLED 10
#define REDLED   11

#if (defined EZRADIO_VARIABLE_DATA_START)
#define APP_PKT_DATA_START EZRADIO_VARIABLE_DATA_START
#else
#define APP_PKT_DATA_START 1u
#endif


//#define APP_RTC_FREQ_HZ 9u																											/* RTC frequency */
//#define APP_RTC_TIMEOUT_MS ( 1000u / APP_RTC_FREQ_HZ )													/* RTC timeout */
#define TX_FIFO_TIMEOUT_MS	100L																									// max time to transmit a fifo buffer
// ******************** local variables ******************************
static uint8_t radioTxPkt[MAX_PACKET_LENGTH+1];
static uint8_t radioTxCount=0;
static bool     appTxActive = false;																						/* Sign tx active state */
static bool     RxDataReady = false;
static bool     RxDataIncoming = false;
static uint8_t radioRxPkt[MAX_PACKET_LENGTH+1];																					/* Rx packet data array */

static RTCDRV_TimerID_t TxFifoTimer;																						// Timer used to issue time elapsed for TX Fifo

static EZRADIODRV_HandleData_t appRadioInitData = EZRADIODRV_INIT_DEFAULT;							/* EZRadio driver init data and handler */
static EZRADIODRV_Handle_t appRadioHandle = &appRadioInitData;

// ******************** local function prototypes ********************
static void appPacketTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
/* sniprintf does not process floats, but occupy less flash memory ! */
#define snprintf    sniprintf
#endif
static void GpioSetup(void);

static void TxFifoTimeout(RTCDRV_TimerID_t id, void *user );

//static int RepeatCallbackRegister (void(*pFunction)(void*),void* pParameter,unsigned int frequency);
// ********************* Implementation ******************************

/**************************************************************************
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
static void GpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_DriveModeSet(LED_PORT, gpioDriveModeHigh);
  GPIO_PinModeSet(LED_PORT, GREENLED, gpioModePushPullDrive, 0);
  GPIO_PinModeSet(LED_PORT, REDLED  , gpioModePushPullDrive, 0);

  /* Initialize GPIO interrupt */
  GPIOINT_Init();
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 0);
}
#if 0
/**************************************************************************//**
 * @brief   Register a callback function to be called repeatedly at the
 *          specified frequency.
 *
 * @param[in] pFunction  Pointer to function that should be called at the
 *                       given frequency.
 * @param[in] pParameter Pointer argument to be passed to the callback function.
 * @param[in] frequency  Frequency at which to call function at.
 *
 * @return  0 for successful or
 *         -1 if the requested frequency is not supported.
 *****************************************************************************/
static int RepeatCallbackRegister (void(*pFunction)(void*),
                            void* pParameter,
                            unsigned int frequency)
{

  if (ECODE_EMDRV_RTCDRV_OK ==
      RTCDRV_AllocateTimer( &rtcRepeateTimer))
  {
    if (ECODE_EMDRV_RTCDRV_OK ==
      RTCDRV_StartTimer(rtcRepeateTimer, rtcdrvTimerTypePeriodic, frequency,
          (RTCDRV_Callback_t)pFunction, pParameter ))
    {
      return 0;
    }
  }

  return -1;
}
#endif

static void TxFifoTimeout(RTCDRV_TimerID_t id, void *RadioHandle )										  //TxFifoTimer handle tx fifo not complete interrupt failed
{
	(void) id;
  if(true == appTxActive)
  {
  	appTxActive = false;
  	ezradioStartRx(RadioHandle);
  }
}

static void TraceSwoSetup(void)
{
	/* Enable GPIO Clock. */
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
	/* Enable Serial wire output pin */
	GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_WONDER_FAMILY) || defined(_EFM32_LEOPARD_FAMILY)
	/* Set location 0 */
	GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

	/* Enable output on pin - GPIO Port F, Pin 2 */
	GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
	GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
	/* Set location 1 */
	GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
	/* Enable output on pin */
	GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
	GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif
	/* Enable debug clock AUXHFRCO */
	CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

	while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

	/* Enable trace in core debug */

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  ITM->LAR  = 0xC5ACCE55;
	  ITM->TER  = 0x0;
	  ITM->TCR  = 0x0;
	  TPI->SPPR = 2;
	  TPI->ACPR = 0xf;
	  ITM->TPR  = 0x0;
	  DWT->CTRL = 0x400003FE;
	  ITM->TCR  = 0x0001000D;
	  TPI->FFCR = 0x00000100;
	  ITM->TER  = 0x1;
}
static void func_out(unsigned char uc)
{
	ITM_SendChar(uc);
}

void Init_debug(void) // setup swo output, point xprintf to ITM_SendChar()
{
	TraceSwoSetup();
	xfunc_out = func_out;
}

static void hardware_init(void)
{
	timer_init();																																	// initialise timers
	serial_init(57/*param_s_get(PARAM_SERIAL_SPEED)*/);	//TODO													// UART - set the configured speed
}


/**************************************************************************//**
 * @brief  Main function of the example.
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* HFXO 48MHz, divided by 1, rco is only 28Mhz   TODO , calibrate RCO from radio XO??*/
  CMU_OscillatorEnable(cmuOsc_HFRCO,true,true);
  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
  CMU_ClockEnable(cmuClock_HF, true);
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);

  GpioSetup();																																	/* Setup GPIO for pushbuttons. */
  RTCDRV_Init();																																/* Set RTC to generate interrupt 250ms. */
  if (ECODE_EMDRV_RTCDRV_OK !=
      RTCDRV_AllocateTimer( &TxFifoTimer) )
  {
    while (1);
  }

  Init_debug();																																	// start debugging output
	hardware_init();																															// Do hardware initialisation
  appRadioInitData.packetTx.userCallback = &appPacketTransmittedCallback;				// Configure packet transmitted callback.
  appRadioInitData.packetRx.userCallback = &appPacketReceivedCallback;					// Configure packet received buffer and callback.
  appRadioInitData.packetRx.pktBuf = radioRxPkt;																// set the dest buffer
  appRadioInitData.packetRx.pktBufLen = sizeof(radioRxPkt);											// set the dest buffer size
  appRadioInitData.packetCrcError.userCallback = &appPacketCrcErrorCallback;		// Configure packet received with CRC error callback.
  ezradioInit( appRadioHandle );																								// Initialize EZRadio device.
  ezradioResetTRxFifo();																												// Reset radio fifos and start reception.
  ezradioStartRx( appRadioHandle );
  tdm_init();
  /* Enter infinite loop that will take care of ezradio plugin manager and packet transmission. */
  while (1)
  {
    ezradioPluginManager( appRadioHandle );
    tdm_serial_loop();
  }
}

/// begin transmission of a packet
/// @param length		Packet length to be transmitted; assumes
///				the data is already present in the FIFO.
/// @param timeout_ticks	The number of ticks to wait before assiming
///				that transmission has failed.
/// @return			true if packet sent successfully
///
bool radio_transmit(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks)
{
	static EZRADIODRV_PacketLengthConfig_t pktLength =
  {ezradiodrvTransmitLenghtCustomFieldLen,10,{1,9,0,0,0}} ;
	bool Res = false;

	if((!appTxActive)&&(length < sizeof(radioTxPkt)))
	{
		pktLength.fieldLen.f2  = length;
		pktLength.pktLen = length+1;
		radioTxPkt[0] = length;
  	memcpy(&radioTxPkt[1],buf,length);
  	appTxActive = (Res = (ECODE_EMDRV_EZRADIODRV_OK == ezradioStartTransmitCustom(appRadioHandle, pktLength, radioTxPkt)));
    if (appTxActive)
    {
    	radioTxCount = (pktLength.pktLen >= EZRADIO_FIFO_SIZE)?(EZRADIO_FIFO_SIZE):(pktLength.pktLen);
    	RTCDRV_StopTimer(TxFifoTimer);
    	RTCDRV_StartTimer(TxFifoTimer,rtcdrvTimerTypeOneshot, TX_FIFO_TIMEOUT_MS,TxFifoTimeout,appRadioHandle);
    }
	}
	return(Res);
}

/// receives a packet from the radio
/// @param len			Pointer to storage for the length of the packet
/// @param buf			Pointer to storage for the packet
/// @return			True if a packet was received
bool radio_receive_packet(uint8_t *length, uint8_t *  buf)
{
  if(RxDataReady)
  {
  	*length = radioRxPkt[0];
  	if(*length > MAX_PACKET_LENGTH)
  	{
  		*length = MAX_PACKET_LENGTH;
  	}
  	memcpy(buf,&radioRxPkt[1],*length);
  	RxDataReady = false;
  	return(true);
  }
	return(false);
}


/// test whether the radio has detected a packet preamble
/// @return			True if a preamble has been detected
bool radio_preamble_detected(void)
{
  return(RxDataIncoming);
}
/// check if a packet is coming in
/// @return			true if a packet is being received
bool radio_receive_in_progress(void)
{
	return(RxDataIncoming);
}

/**************************************************************************//**
 * @brief  Packet transmitted callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
void appPacketTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( ECODE_EMDRV_EZRADIODRV_PACKET_SENT == (status&ECODE_EMDRV_EZRADIODRV_PACKET_SENT))
  {
    /* Sign tx passive state */
  	RTCDRV_StopTimer(TxFifoTimer);
    appTxActive = false;
    ezradioStartRx( handle );
  }
  if((true == appTxActive)&&( ECODE_EMDRV_EZRADIODRV_TX_NEAR_EMPTY == (status&ECODE_EMDRV_EZRADIODRV_TX_NEAR_EMPTY)))
  {
  	if(radioTxCount < radioTxPkt[0])
  	{
  		uint8_t len = radioTxPkt[0] - radioTxCount;
  	  ezradio_cmd_reply_t radioReplyData;
      ezradio_fifo_info_fast_read(&radioReplyData);
  	  if (len > radioReplyData.FIFO_INFO.TX_FIFO_SPACE)
  	  {
  	  	len = radioReplyData.FIFO_INFO.TX_FIFO_SPACE;
  	  }
  		ezradio_write_tx_fifo(len, &radioTxPkt[radioTxCount]);
  		radioTxCount += len;
  	}
  }

 #if 0
  if(ECODE_EMDRV_EZRADIODRV_CHIP_ERROR == (status&ECODE_EMDRV_EZRADIODRV_CHIP_ERROR))
  {
    if(appTxActive)
    {
    	appTxActive = false;
    	ezradioStartRx( handle );
    }
  }
  if(ECODE_EMDRV_EZRADIODRV_UF_OF_ERROR == (status&ECODE_EMDRV_EZRADIODRV_UF_OF_ERROR))
  {
    if(appTxActive)
    {
			appTxActive = false;
			ezradioStartRx( handle );
    }
  }
  if(ECODE_EMDRV_EZRADIODRV_STATE_CHANGE == (status&ECODE_EMDRV_EZRADIODRV_STATE_CHANGE))
  {
    if(appTxActive)
    {
			appTxActive = false;
//			ezradioStartRx( handle );
    }
  }
#endif
}

/**************************************************************************//**
 * @brief  Packet received callback of the application.
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( ECODE_EMDRV_EZRADIODRV_PACKET_RX == status)
  {
    RxDataReady = true;
  	RxDataIncoming = false;																											// clear incoming, packet arrived
  }
  else if (ECODE_EMDRV_EZRADIODRV_PREAMBLE_DETECT == status)
  {
  	RxDataIncoming = true;
  }
}

/**************************************************************************//**
 * @brief  Packet received with CRC error callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
  	RxDataIncoming = false;
  	//printf("-->Pkt  RX: CRC Error\n");
    /* Change to RX state */
    ezradioStartRx( handle );
  }
}
