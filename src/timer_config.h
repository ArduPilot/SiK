/*
 * timer_config.h
 *
 *  Created on: 27/11/2015
 *      Author: kentm
 */

#ifndef TIMER_CONFIG_H_
#define TIMER_CONFIG_H_
#include "em_device.h"
#include "em_cmu.h"
#include "em_timer.h"

//#define USTIMER_TIMER USTIMER_TIMER0

#define TDMTIMER1									 TIMER1
#define TDMTIMER1_cmuClock cmuClock_TIMER1
#define TDMTIMER1_IRQn								TIMER1_IRQn

#define TDMTIMER2									 TIMER2
#define TDMTIMER2_cmuClock cmuClock_TIMER2
#define TDMTIMER2_IRQn								TIMER2_IRQn

#define MSTIMER 									LETIMER0
#define MSTIMER_cmuClock 					cmuClock_LETIMER0
#define MSTIMER_IRQn							LETIMER0_IRQn

#define PWMTIMER TIMER3
#define cmuClock_PWMTIMER  cmuClock_TIMER3
#define PWMCCX	2
#define PWMTIMER_ROUTE_CCXPEN TIMER_ROUTE_CC2PEN
#define PWMTIMER_ROUTE_LOCATION TIMER_ROUTE_LOCATION_LOC1
/*
#define RXfn(_a) 	_a##_IRQHandler(void)
#define RXTIMER_IRQHandler(a) 		RXfn(TIMER0)
#define RXTIMER_CLOCK	 cmuClock_TIMER0
#define RXTIMER 								TIMER0
#define RXTIMER_IRQn            TIMER0_IRQn
 //temp disable until get another timer back
*/

#define PPMfn(_a) 	_a##_IRQHandler(void)
#define PPMTIMER_IRQHandler(a) 		PPMfn(TIMER0)
#define PPMTIMER_CLOCK	 cmuClock_TIMER0
#define PPMTIMER 								TIMER0
#define PPMTIMER_IRQn            TIMER0_IRQn
#define PPMCCRX	0												// needs to be cc0 always
#define PPMCCTX	2												// needs to be on actual pin
#define PPMTIMER_ROUTE_CCTXPEN TIMER_ROUTE_CC2PEN
#define PPMTIMER_ROUTE_LOCATION TIMER_ROUTE_LOCATION_LOC3
#define PPMCCRX2 2											// doesn't route anywhere, time detection only
#define PPMTIMER_IF_CCX2 TIMER_IF_CC2
#define DMAREQ_PPMRX 		DMAREQ_TIMER0_CC0
#define DMAREQ_PPMTX 		DMAREQ_TIMER0_CC2
#define DMAREQ_PPMTX2 	DMAREQ_TIMER0_UFOF

#endif /* TIMER_CONFIG_H_ */
