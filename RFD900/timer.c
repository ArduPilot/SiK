/*
 * timer.c
 *
 *  Created on: 06/11/2015
 *      Author: kentm
 */

#include <stdint.h>
#include <stdbool.h>
#include "em_timer.h"
#include "em_cmu.h"
#include "radio_old.h"
#include "at.h"
#include "serial.h"
#include "timer_config.h"
#include "em_letimer.h"
#include "timer.h"
#include "ustimer.h"

// ******************** defines and typedefs *************************
#define TDMSHIFT 0
#define TDMFREQ (62500U*(1U<<TDMSHIFT))																					// 16uS to match tdm calculations

#define usec2Ticks(usec) ((usec+(1000000UL/TDMFREQ)-1)/(1000000UL/TDMFREQ))
// ******************** local variables ******************************
static volatile uint32_t delay_counter=0;																					/// Counter used by delay_msec
// ******************** global variables *****************************
// ******************** local function prototypes ********************
static void mSTimer_Init(void);
// ********************* Implementation ******************************
void delay_msec(register uint16_t msec)
{
	delay_set(msec);
	while (!delay_expired())
		;
}

// initialise timers
void timer_init(void)
{
	static bool Init = false;
	if(Init)return;
	Init = true;
	// set up 100Hz ms timer interrupt at 100Hz
	// set up cascaded timers to generate 64250 Hz 16 bit free running timer
	// similar to the old system
	// original timer was based on 24500000 Clock /12 = 2.041666 Mhz / (1<<5=32) = 63.802 Khz rollover @ ~ 1.03S
	// just use 1 bit of 16 bit timer for entropy, should be good enough

  CMU_ClockDivSet(cmuClock_HFPER,cmuClkDiv_1); 																	// set divide for hfper clk
  CMU_ClockEnable(TDMTIMER1_cmuClock, true);		 																/* Enable clock for TDMTIMER1 module */
  CMU_ClockEnable(TDMTIMER2_cmuClock, true);		 																/* Enable clock for TDMTIMER2 module */
  mSTimer_Init();

  TIMER_Init_TypeDef tdmtimerInit =						   /* Select TDMTIMER parameters */
  {
    .enable     = true,
    .debugRun   = false,
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

  uint32_t Count = ((CMU_ClockFreqGet(cmuClock_HFPER)+TDMFREQ)/TDMFREQ)-1;
  TIMER_TopSet(TDMTIMER1, Count); 						/* Set TIMER Top value */
  TIMER_Init(TDMTIMER1, &tdmtimerInit);						/* Configure TIMER */

  TIMER_Init_TypeDef tdmtimerInit2 =						   /* Select TDMTIMER parameters */
  {
    .enable     = true,
    .debugRun   = false,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelCascade,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  TIMER_TopSet(TDMTIMER2, 0xffff); 						/* Set TIMER Top value */
  TIMER_Init(TDMTIMER2, &tdmtimerInit2);						/* Configure TIMER */
}

// return some entropy
uint8_t timer_entropy(void)
{
	// use the SYSCLK/12 timer
	return(TIMER_CounterGet(TDMTIMER2));
}

void delay_set(register uint32_t msec)
{
	delay_counter = (msec + 90) / 100;
}

bool delay_expired(void)
{
	return delay_counter == 0;
}

// ********************* Implementation local functions **************


static void mSTimer_Init(void)
{
  CMU_OscillatorEnable(cmuOsc_LFRCO,true,true);
  CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_LFRCO );													// note rtc timer already sets this
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockDivSet(MSTIMER_cmuClock, cmuClkDiv_1);
	CMU_ClockEnable(MSTIMER_cmuClock, true);

	uint32_t Count = CMU_ClockFreqGet(MSTIMER_cmuClock)/100;
	LETIMER_CompareSet(LETIMER0, 0, Count);

	const LETIMER_Init_TypeDef letimerInit =
	{
	.enable         = true,                   /* Start counting when init completed. */
	.debugRun       = true,                  /* Counter shall not keep running during debug halt. */
	.rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
	.rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
	.comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	.bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	.out0Pol        = 0,                      /* Idle value for output 0. */
	.out1Pol        = 0,                      /* Idle value for output 1. */
	.ufoa0          = letimerUFOANone,         /* PWM output on output 0 */
	.ufoa1          = letimerUFOANone,       /* Pulse output on output 1*/
	.repMode        = letimerRepeatFree       /* Count until stopped */
	};

	LETIMER_IntEnable(MSTIMER,LETIMER_IF_UF);
  NVIC_EnableIRQ(MSTIMER_IRQn);
	LETIMER_Init(MSTIMER, &letimerInit);
}

void LETIMER0_IRQHandler(void)
{
  static uint16_t tick10mS;
  LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
	// call the AT parser tick
  if(++tick10mS >= 10)
  {
    at_timer();
    // update the delay counter
    if (delay_counter > 0)
      delay_counter--;
    tick10mS = 0;
  }
	Serial_Check();
}
Ecode_t USTIMER_Init( void )
{
	timer_init();
	return(ECODE_EMDRV_USTIMER_OK);
}

Ecode_t USTIMER_DeInit( void ){return(ECODE_EMDRV_USTIMER_OK);};

Ecode_t USTIMER_Delay( uint32_t usec )
{
	uint16_t Ticks = usec2Ticks(usec);
	uint16_t tickStart = timer2_tick();
	while(((uint16_t)(timer2_tick() - tickStart)) <Ticks);
	return(ECODE_EMDRV_USTIMER_OK);
}


// ********************* end of timer.c ******************************
