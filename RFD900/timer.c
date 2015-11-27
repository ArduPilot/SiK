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


// ******************** defines and typedefs *************************
#define TDMSHIFT 0
#define TDMFREQ (64250U*(1U<<TDMSHIFT))

// ******************** local variables ******************************
static volatile uint8_t delay_counter;																					/// Counter used by delay_msec
static uint32_t TdmTicks=0;
// ******************** global variables *****************************
// ******************** local function prototypes ********************
static void delay_set(register uint16_t msec);
static bool delay_expired(void);

// ********************* Implementation ******************************
void delay_msec(register uint16_t msec)
{
	delay_set(msec);
	while (!delay_expired())
		;
}

// return a 16 bit value that rolls over in approximately
// one second intervals
uint16_t timer2_tick(void)
{
	return(TdmTicks>>TDMSHIFT);
	//return(TIMER_CounterGet(TDMTIMER));
#if 0
	register uint16_t low, high;
	do {
		high = timer2_high;
		low = TIMER_CounterGet(TDMTIMER);
	} while (high != timer2_high);
	// take 11 high bits from the 2MHz counter, and 5 low bits
	// from the rollover of that counter
	return (high<<11) | (low>>5);
#endif
}

// initialise timers
void timer_init(void)
{
	// set up 100Hz timer interrupt at 100Hz
	// and 2 041 667 Hz 16 bit timer, interrupt on overflow only (31Hz)
	// system clock should be 32Mhz divides by 1-32768 in powers of  for hfper
	// and  1-1024 for timer module
  // divide by 16 to get 2Mhz

	// timer 3 should be a 10 mS or 100hz freq
	// timer 2 should be a free running counter rolling over 0xffff at 1S interval,T=15.2587uS, f =65536
	// clock is 28Mhz /512 = 54687Hz , 0xffff roll over at ~ 1.2S
	// original timer was based on 24500000 Clock /12 = 2.041666 Mhz / (1<<5=32) = 63.802 Khz rollover @ ~ 1.03S
	// take 11 high bits from the counter, and 5 low bits from the rollover of that counter
  // timer entropy was read from lower 16 bit of counter

  CMU_ClockDivSet(cmuClock_HFPER,cmuClkDiv_1); // set divide for phfper clk
  CMU_ClockEnable(TDMTIMER_cmuClock, true);		 /* Enable clock for TDMTIMER module */
  CMU_ClockEnable(MSTIMER_cmuClock, true);		 /* Enable clock for MSTIMER module */

  TIMER_Init_TypeDef mStimerInit =							 /* Select MSTIMER parameters */
  {
    .enable     = true,
    .debugRun   = false,
    .prescale   = timerPrescale16,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

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

	uint32_t Count = CMU_ClockFreqGet(cmuClock_HFPER);
	Count = (Count/(100*16));
  TIMER_TopSet(MSTIMER, Count);							  /* Set TIMER Top value */
  TIMER_IntEnable(MSTIMER, TIMER_IF_OF); 		  /* Enable overflow interrupt */
  NVIC_EnableIRQ(MSTIMER_IRQn);   						  /* Enable TIMER1 interrupt vector in NVIC */
  TIMER_Init(MSTIMER, &mStimerInit);						/* Configure TIMER */



	Count = ((CMU_ClockFreqGet(cmuClock_HFPER)+TDMFREQ)/TDMFREQ)-1;
  TIMER_TopSet(TDMTIMER, Count); 						/* Set TIMER Top value */
  TIMER_IntEnable(TDMTIMER, TIMER_IF_OF); 		/* Enable overflow interrupt */
  NVIC_EnableIRQ(TDMTIMER_IRQn);   						/* Enable TDMTIMER interrupt vector in NVIC */
  TIMER_Init(TDMTIMER, &tdmtimerInit);						/* Configure TIMER */
}

// return some entropy
uint8_t timer_entropy(void)
{
	// use the SYSCLK/12 timer
	return((uint8_t)TdmTicks);
}

// ********************* Implementation local functions **************

static void delay_set(register uint16_t msec)
{
	if (msec >= 2550) {
		delay_counter = 255;
	} else {
		delay_counter = (msec + 9) / 10;
	}
}

static bool delay_expired(void)
{
	return delay_counter == 0;
}


void TIMER2_IRQHandler(void)
{
  /* Clear flag for TIMER1 overflow interrupt */
  TIMER_IntClear(TIMER2, TIMER_IF_OF);
	// call the AT parser tick
	at_timer();

	// update the delay counter
	if (delay_counter > 0)
		delay_counter--;
}
void TIMER1_IRQHandler(void)
{
  /* Clear flag for TIMER2 overflow interrupt */
  TIMER_IntClear(TIMER1, TIMER_IF_OF);
  TdmTicks ++;
/*
  timer2_high++;
	if (feature_rtscts) {
		serial_check_rts();
	}
*/
}



// ********************* end of timer.c ******************************
