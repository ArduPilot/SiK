/*
 * PWM.c
 *
 *  Created on: 26/10/2015
 *      Author: kentm
 */

#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "pwm.h"
// ******************** defines and typedefs *************************
#define FREQ 100000L

#define PWMTIMER TIMER3
#define PWMCCX	2
#define PWMTIMER_ROUTE_CCXPEN TIMER_ROUTE_CC2PEN
#define PWMTIMER_ROUTE_LOCATION TIMER_ROUTE_LOCATION_LOC1
#define cmuClock_PWMTIMER  cmuClock_TIMER3
#define PWM_PORT gpioPortE
#define PWM_PIN 2
// ******************** local variables ******************************
// ******************** local function prototypes ********************
// ********************* Implementation ******************************
void InitPWM(void)
{
	CMU_ClockEnable(cmuClock_PWMTIMER, true);
	GPIO_PinModeSet(PWM_PORT, PWM_PIN, gpioModePushPull, 0);
	SetPwmDuty(500);
}

void SetPwmDuty(uint16_t Dutyby10)
{
	uint32_t Top, On;
	// divide by 10 to get duty % in 0.1% increments
	if (Dutyby10 > 1000){		Dutyby10 = 1000;}


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
	Top = (((CMU_ClockFreqGet(cmuClock_HFPER))/(1*FREQ))-1);
	// (28000000UL)/(1*20000) = 1400
	TIMER_TopSet(PWMTIMER,Top);
	On = (Top * Dutyby10) / 1000;
	TIMER_CompareSet(PWMTIMER,PWMCCX, On);			// set on time
	TIMER_Init(PWMTIMER, &timerInit);
}

// ********************* PWM.c ***************************************
