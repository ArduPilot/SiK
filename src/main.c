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
#include "xprintf.h"
#include "tdm.h"
#include "timer.h"
#include "serial.h"
#include "parameters.h"
#include "em_wdog.h"
#include "RCOComp.h"
#include "pins_user.h"
#include "aes.h"
// ******************** defines and typedefs *************************

#ifdef printf
#undef printf
#endif
#define printf(...) xprintf(__VA_ARGS__)

#define LED_PORT gpioPortF
#define GREENLED 10
#define REDLED   11

#define RCOCOMPUPDATEMS 5UL*60UL*1000UL								// update RCO for temperature drift this often i.e. 5Mins
// ******************** local variables ******************************
// ******************** local function prototypes ********************
#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
/* sniprintf does not process floats, but occupy less flash memory ! */
#define snprintf    sniprintf
#endif
static void GpioSetup(void);

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
#if PIN_MAX > 0
	pins_user_init();
#endif
	serial_init(param_s_get(PARAM_SERIAL_SPEED));																	// UART - set the configured speed
}


/**************************************************************************//**
 * @brief  Main function of the example.
 *****************************************************************************/
int main(void)
{
  //SCB->VTOR = 0x0000;	// vectors moved as with origin of code
	/* Chip errata */
  CHIP_Init();

  /* HFXO 48MHz, divided by 1, rco is only 28Mhz   TODO , calibrate RCO from radio XO??*/
  // calibration tables exist in ROM , data sheet has temp comp curves
  // use this info to calibrate on the fly
  CMU_OscillatorEnable(cmuOsc_HFRCO,true,true);
  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
  CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_1);
  CMU_ClockEnable(cmuClock_HF, true);
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);

  GpioSetup();																																	/* Setup GPIO for pushbuttons. */
  Init_debug();																																	// start debugging output
	// Load parameters from flash or defaults
	// this is done before hardware_init() to get the serial speed
	if (!param_load())
		param_default();
	hardware_init();																															// Do hardware initialisation
  tdm_init();
	if (! aes_init(param_s_get(PARAM_ENCRYPTION))) {
		//panic("failed to initialise aes");
	}

  WDOG_Init_TypeDef init=WDOG_INIT_DEFAULT;
  init.perSel =   wdogPeriod_2k;																								// 1 second
  WDOG_Init(&init);
  InitRCOCalibration();

  /* Enter infinite loop that will take care of ezradio plugin manager and packet transmission. */
  while (1)
  {
    WDOG_Feed();
    tdm_serial_loop();
    if(delay_expired())
    {
    	delay_set(RCOCOMPUPDATEMS);
      UpdateRCOCalibration();
    }
  }
}
