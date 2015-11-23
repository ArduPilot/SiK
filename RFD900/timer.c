


#include <stdint.h>
#include <stdbool.h>
#include "em_timer.h"
#include "em_cmu.h"
#include "radio_old.h"
#include "at.h"
#include "serial.h"

/// Counter used by delay_msec
///
static volatile uint8_t delay_counter;

/// high 16 bits of timer2 SYSCLK/12 interrupt
//static volatile uint16_t timer2_high;


void delay_set(register uint16_t msec)
{
	if (msec >= 2550) {
		delay_counter = 255;
	} else {
		delay_counter = (msec + 9) / 10;
	}
}

void delay_set_ticks(register uint8_t ticks)
{
	delay_counter = ticks;
}

bool delay_expired(void)
{
	return delay_counter == 0;
}

void delay_msec(register uint16_t msec)
{
	delay_set(msec);
	while (!delay_expired())
		;
}

void TIMER3_IRQHandler(void)
{
  /* Clear flag for TIMER1 overflow interrupt */
  TIMER_IntClear(TIMER3, TIMER_IF_OF);
	// call the AT parser tick
	at_timer();

	// update the delay counter
	if (delay_counter > 0)
		delay_counter--;
}
#if 0
void TIMER2_IRQHandler(void)
{
  /* Clear flag for TIMER2 overflow interrupt */
  TIMER_IntClear(TIMER2, TIMER_IF_OF);
	timer2_high++;
	if (feature_rtscts) {
		serial_check_rts();
	}
}
#endif
// return a 16 bit value that rolls over in approximately
// one second intervals
uint16_t timer2_tick(void)
{
	return(TIMER_CounterGet(TIMER2));
#if 0
	register uint16_t low, high;
	do {
		high = timer2_high;
		low = TIMER_CounterGet(TIMER2);
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

  CMU_ClockDivSet(cmuClock_HFPER,cmuClkDiv_1); // set divide for phfper clk
  CMU_ClockEnable(cmuClock_TIMER2, true);		/* Enable clock for TIMER2 module */
  CMU_ClockEnable(cmuClock_TIMER3, true);		/* Enable clock for TIMER1 module */

  TIMER_Init_TypeDef timerInit =						/* Select TIMER2 parameters */
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

  TIMER_Init_TypeDef timerInit2 =						/* Select TIMER1 parameters */
  {
    .enable     = true,
    .debugRun   = false,
    .prescale   = timerPrescale512,
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

  TIMER_TopSet(TIMER3, Count);							/* Set TIMER Top value */
  TIMER_IntEnable(TIMER3, TIMER_IF_OF); 		/* Enable overflow interrupt */
  NVIC_EnableIRQ(TIMER3_IRQn);   						/* Enable TIMER1 interrupt vector in NVIC */
  TIMER_Init(TIMER3, &timerInit);						/* Configure TIMER */

  TIMER_TopSet(TIMER2, 0xFFFF); 						/* Set TIMER Top value */
  //TIMER_IntEnable(TIMER2, TIMER_IF_OF); 		/* Enable overflow interrupt */
  //NVIC_EnableIRQ(TIMER2_IRQn);   						/* Enable TIMER2 interrupt vector in NVIC */
  TIMER_Init(TIMER2, &timerInit2);						/* Configure TIMER */
  #if 0
	TMR3RLL	 = (65536/65536=UL - ((SYSCLK / 12) / 100)) & 0xff;
	TMR3RLH	 = ((65536UL - ((SYSCLK / 12) / 100)) >> 8) & 0xff;
	TMR3CN	 = 0x04;	// count at SYSCLK / 12 and start
	EIE1	|= 0x80;

	// setup TMR2 as a timer source at SYSCLK/12
	TMR2RLL = 0;
	TMR2RLH = 0;
	TMR2CN  = 0x04; // start running, count at SYSCLK/12
	ET2 = 1;
#endif
}

// return some entropy
uint8_t timer_entropy(void)
{
	// use the SYSCLK/12 timer
	return((uint8_t)TIMER_CounterGet(TIMER2));
}
