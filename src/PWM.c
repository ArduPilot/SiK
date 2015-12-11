/*
 * PWM.c
 *
 *  Created on: 26/10/2015
 *      Author: kentm
 *  set the PWM output as 0-255 for duty 0 -100%
 *  the clock will be determined so count 255 = 100%
 */

#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "pwm.h"
#include "timer_config.h"
// ******************** defines and typedefs *************************

#define PWM_PORT gpioPortE
#define PWM_PIN 2
// ******************** local variables ******************************
// ******************** local function prototypes ********************
// ********************* Implementation ******************************
void InitPWM(void)
{
	CMU_ClockEnable(cmuClock_PWMTIMER, true);
	GPIO_PinModeSet(PWM_PORT, PWM_PIN, gpioModePushPull, 0);
	uint32_t Top;
	TIMER_InitCC_TypeDef timerCCInit =
	{
		.eventCtrl  = timerEventEveryEdge,
		.edge       = timerEdgeBoth,
		.prsSel     = 0,
		.cufoa      = timerOutputActionNone,
		.cofoa      = timerOutputActionNone,
		.cmoa       = timerOutputActionToggle,
		.mode       = timerCCModePWM,
		.filter     = false,
		.prsInput   = false,
		.coist      = false,
		.outInvert  = false,
	};
	/* Configure CC channel 0 */
	TIMER_InitCC(PWMTIMER, PWMCCX, &timerCCInit);

	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale1,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};
	/* Route CC0 to location 3 (PD1) and enable pin */
	PWMTIMER->ROUTE |= (PWMTIMER_ROUTE_CCXPEN | PWMTIMER_ROUTE_LOCATION);
	Top = 0xff;
	// Freq = (28000000UL/256) = 109375 := 100Khz
	TIMER_TopSet(PWMTIMER,Top);
	TIMER_CompareSet(PWMTIMER,PWMCCX,0x00);			// set on time
	TIMER_Init(PWMTIMER, &timerInit);
}



void SetPwmDuty(uint8_t Duty8Bit)
{
	TIMER_CompareBufSet(PWMTIMER,PWMCCX,(~Duty8Bit)&0xFF);			// set on time, use buffered so no glitches
}

// ********************* PWM.c ***************************************
