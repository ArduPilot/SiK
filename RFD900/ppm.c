/*
 * ppm.c
 * generates and reads a ppm stream
 * A PPM stream consists of a 4ms (or longer) period pulse time is always the same
 * followed by number of channels period width 1-2.3 mS (0-130%)
 * 2.016ms =100%
 * Note that polarity made change depending on vendor! ( should cope anyway since pulse is always the same)
 * note width is measured from rising edge to rising edge (or falling to falling)
 * pulse high(or low) width is usually fixed to 100uS.
 * measure the width in microseconds (or closest to) and regenerate
 * with a 100uS high(or low) pulse width
 * Use DMA to capture time ccx on each positive (or negative) edge
 * The number of channels will vary so use
 * a pulse period > 4ms to detect start of pulse, set max channels to 16
 * use a period > 4ms and record this time in the output stream to regenerate the
 * same timing on the output
 *
 * setup a timer to count from positive edge to positive edge (or negative - negative)
 * on edge the counter value will be captured and timer restarted
 * set up a compare on same timer to detect
 * when period exceeds a certain value (3-4ms). This compare event then
 * changes the DMA count remaining to one and records how many channels
 * will be recorded
 *
 * to regenerate we will use the recorded channel widths
 * including the sync pulse width so it matches exactly
 * what was seen at the input
 *
 * regeneration of PWM can be done using a shift register
 * with pulses as clock and frame detection as data
 * need to check on this? requires an extra pin for frame sync
 * let the user get one, this is not often used
 *
 *  Created on: 20/02/2016
 *      Author: kentm
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_int.h"
#include "gpiointerrupt.h"
#include "dmadrv.h"
#include "timer_config.h"
#include "prs_config.h"
#include "board_rfd900e.h"
#include "ppm.h"
#include "parameters.h"

// ******************** defines and typedefs *************************
#define PPMCLOCKPRESC			timerPrescale32																				// attempt to get around 1uS
#define PPMCLOCKDIV 			(1UL<<_TIMER_CTRL_PRESC_DIV32)												// PPM clock divide
#define PPMCLOCK 					(EFM32_HFXO_FREQ/PPMCLOCKDIV)													// PPM clock Freq
#define uS2Ticks(uS)  		((uS*(PPMCLOCK/1000UL)/1000UL))												// convert uS to ticks
#define SYNC_WIDTH_TICKS 	uS2Ticks(3000UL)																			// should be 4000 , but lets give us an extra mS to respond
#define TXSYNC_WIDTH_TICKS 	uS2Ticks(4000UL)																	  // should be 4000 , but lets give us an extra mS to respond
#define PULSE_WITDH_TICKS uS2Ticks(300UL)																				// rx detect period threshold for sync
#define MAX_DATA_LOSS			50																										// frames to lose before going back to default
// ******************** local variables ******************************
static PPMMode_t PPMMode=0;																											// what mode are we in tx or rx?
static uint16_t PPMTxLen=0;																											// what is the txlength (channnels+1)
static volatile uint16_t PPMRxLen[2]={0};																				// how much data did we rx (channels+1)
static const GPIO_Port_Pin_TypeDef PPMGPIO = P1_1;															// where is our tx/rx pin located
static unsigned int DMACh = 0;																									// rx dma , or tx dma on time
static DMA_CB_TypeDef cb;																												// callbacks for dma ch1
static uint16_t DMABuffer[2][DMA_BUFF_LEN];																			// tx/rx ping pong buffers
static uint16_t DataLossData[] = {[0 ... DMA_BUFF_LEN-2] = uS2Ticks(1000UL),[DMA_BUFF_LEN-1]=uS2Ticks(8000UL)};		// what to send when there is complete data loss
static uint16_t DataLossDataLen = DMA_BUFF_LEN;
static bool DataWaiting[2];																											// set by write calls, dma will chek and update if nothing there
static volatile int16_t LastPrimary=0;																					// ?alt/prim last tx/rx DMA complere event
static DMA_DESCRIPTOR_TypeDef *primDescr;																				// local storage for primary rx descriptor
static DMA_DESCRIPTOR_TypeDef *altDescr;																				// local storage for alternate rx descriptor
// ******************** local function prototypes ********************
static void SetupPPMReader(void);																								// PPM read setup
static void SetupPPMWriter(void);																								// PPM write setup
static void SetupRxTimer(void);																									// PPM read timer setup
static void SetupTxTimer(void);																									// PPM write timer setup
static void SetupRxDMA(void);																										// PPM read DMA setup
static void SetupTxDMA(void);																										// PPM write dma setup
static void RxDMAComplete(unsigned int channel, bool primary, void *user);			// rx dma complete callback
static void TxDMAComplete(unsigned int channel, bool primary, void *user);			// tx dma complete callback
// ********************* Implementation ******************************
__STATIC_INLINE uint16_t GetDMANMinus1(unsigned int Ch)													// find count of current DMA
{
	uint16_t NMinus1;
	DMA_DESCRIPTOR_TypeDef *currDescr;
	currDescr = (((DMA_CB_TypeDef *)(primDescr->USER))->primary)?(primDescr):(altDescr);
	NMinus1 = ((currDescr->CTRL&_DMA_CTRL_N_MINUS_1_MASK)>>_DMA_CTRL_N_MINUS_1_SHIFT);
	return(NMinus1);
}

bool PPMWrite(uint8_t *Data, uint16_t Len)																			// write one complete PPM stream to port
{
	static bool TimerRunning = false;																							// tx timer running flag
	int16_t i=0;
	if(PPMModeOut != PPMMode) return(false);
	Len = Len>>1;
	if(Len > DMA_BUFF_LEN) return(false);
	for(i=0;i<Len;i++)
	{
		if(((uint16_t*)Data)[i] < uS2Ticks(800UL))																	// if width invalid
		{
			return(false);
		}
	}
	// send data to the one that is not running right now
	// to determine this check which one finished last
	// however the next one may not have started yet
	// check the timer compare and count and next ones dma count for count -1
	bool prim = LastPrimary;
	if(!DataWaiting[!prim]) 																											// if no new data in next dma)
	{
		uint16_t DMA_NMinus1 = GetDMANMinus1(DMACh);																// read current dma count
		uint16_t count = TIMER_CounterGet(PPMTIMER);																// read current count
		uint16_t compare = TIMER_CaptureGet(PPMTIMER,PPMCCTX);											// read current compare value
		if(DMA_NMinus1 == (PPMTxLen-1))																							//if we haven't executed the first dma yet
		{
			if(count < compare)																												// if we haven't reached the first compare yet
			{
				if((compare-count)>= uS2Ticks(64UL))																		// if we have enough time to fill before executed
				{
					prim = !prim; 																												// copy to current dma buffer before it starts
				}
			}
		}
	}
	DataWaiting[prim] = true;
	memcpy(DMABuffer[prim],Data,Len<<1);																			    // copy to buffer just sent out as other is now active
	PPMTxLen = Len;																																// save len
	if(!TimerRunning)																															// need to load timer with first values and start if not running
	{
		// start width two long pulses
		TIMER_TopSet(PPMTIMER,uS2Ticks(8000UL));
	  TimerRunning = true;
		TIMER_Enable(PPMTIMER,true);
	}
	return(true);
}

bool ReadPPM(uint8_t *Data, uint16_t* Len)																			// read any complete incoming stream from PPM port
{
	bool prim;
	if(PPMModeIn != PPMMode) return(false);
	if(LastPrimary<0)return(false);																								// no data is ready
	prim = LastPrimary;																														// point to most fresh data
	if(0 == PPMRxLen[prim])return(false);																					// if it didn't find sync pulse
	LastPrimary = -1;																															// invalidate both buffers now
	*Len = PPMRxLen[prim]<<1;
	memcpy(Data,DMABuffer[prim],*Len);																						// copy data to user
	PPMRxLen[prim] = 0;																														// clear length field
	return(true);
}

bool PPMRecordDefault(void)																											// record the default signal to send now
{
	memcpy(DataLossData,DMABuffer[1],PPMTxLen<<1);
	DataLossDataLen = PPMTxLen;
	param_set_PPMDefaults(DataLossData,DataLossDataLen);
	return(true);
}

void PPM_Read_Defaults(uint16_t *Data)
{
	memcpy(Data,DataLossData,sizeof(DataLossData));
	Data[DMA_BUFF_LEN] = DataLossDataLen;
}

bool InitPPM(PPMMode_t Mode)																										// Initialise PPM to read or write
{
	if(Mode >= PPMModeLast)return(false);
	PPMMode = Mode;
	DMADRV_Init();																																// init dma driver if not already
	DMADRV_AllocateChannel(&DMACh, NULL );																				// allocate a channel for rx/tx dma
  primDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->CTRLBASE)) + DMACh;							// Get primary descriptor
  altDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->ALTCTRLBASE)) + DMACh;						// Get alternate descriptor
	param_get_PPMDefaults(DataLossData,&DataLossDataLen);
	int16_t i;
	for(i=0;i<DMA_BUFF_LEN;i++)																											// reset on time buffer values
	{
		DMABuffer[0][i] = uS2Ticks(8000UL);
		DMABuffer[1][i] = uS2Ticks(8000UL);
	}

	if(PPMModeIn == PPMMode)
	{
		SetupPPMReader();
	}
	else
	{
		SetupPPMWriter();
	}
	return(true);
}

static void SetupPPMReader(void)																								// setup PPM Read HW
{
	SetupRxDMA();																																	// set up DMA on rising edge of ccx
	SetupRxTimer();																																// set up timer with 1uS resolution
}

static void SetupPPMWriter(void)																								// setup PPM Write HW
{
	SetupTxDMA();																																	// set up dma to copy length into timer ccx buff on every compare
	SetupTxTimer();																																// set up timer to produce PWM from ccx output
}

static void SetupRxTimer(void)																									// setup PPM RX Timer, input capture with compare for sync pulse detection
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(PPMTIMER_CLOCK, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  GPIO_PinModeSet(PPMGPIO.Port, PPMGPIO.Pin, gpioModeInputPullFilter, 1);
  GPIO_InputSenseSet(GPIO_INSENSE_PRS, GPIO_INSENSE_PRS);
  GPIO_IntConfig(PPMGPIO.Port, PPMGPIO.Pin, false, false, false);
  PRS_SourceSignalSet(PPMPRS,(PPMGPIO.Pin<8)?(PRS_CH_CTRL_SOURCESEL_GPIOL):(PRS_CH_CTRL_SOURCESEL_GPIOH),
  		(PPMGPIO.Pin<8)?(PPMGPIO.Pin):(PPMGPIO.Pin-8), prsEdgeOff);

  TIMER_InitCC_TypeDef timerCCInit =																						// capture PRS on falling edge
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeFalling,
    .prsSel     = PPMPRS,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionNone,
    .mode       = timerCCModeCapture,
    .filter     = true,
    .prsInput   = true,
    .coist      = false,
    .outInvert  = false,
  };
  TIMER_InitCC(PPMTIMER,PPMCCRX, &timerCCInit);

  TIMER_InitCC_TypeDef timerCCInit2 =																						// set up a compare when timer reaches > 4ms to detect start/end frame marker
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeNone,
    .prsSel     = 0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionNone,
    .mode       = timerCCModeCompare,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };
  TIMER_InitCC(PPMTIMER,PPMCCRX2, &timerCCInit2);
  TIMER_CompareSet(PPMTIMER,PPMCCRX2,SYNC_WIDTH_TICKS);

  TIMER_Init_TypeDef timerInit =																								// setup timer ~ 1uS to reload from cc0 edge
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = PPMCLOCKPRESC,																								// want about 1uS clock 28M/1 ~ 32
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionReloadStart,																	// reload on falling edge
    .riseAction = timerInputActionNone,																					// do nothing on rising edge
    .mode       = timerModeUp,
    .dmaClrAct  = true,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };
	TIMER_TopSet(PPMTIMER, 0xffff);																								// set to max to reduce cc2 match interrupts
	TIMER_IntEnable(PPMTIMER, PPMTIMER_IF_CCX2);																	// enable int from compare match
  NVIC_EnableIRQ(PPMTIMER_IRQn);																								// enable in NVIC
  TIMER_Init(PPMTIMER, &timerInit);																							// Init Timer
}


void PPMTIMER_IRQHandler (void)																									// a pulse longer than 4ms has been found
{
	static bool primary;																													// which ping pong are we on?
	static DMA_DESCRIPTOR_TypeDef *descr;																					// pointer to descriptor for current ping pong
  static uint32_t remaining, iflag;
	TIMER_IntClear(PPMTIMER, PPMTIMER_IF_CCX2);																		// clear int flag

	primary = ((DMA_CB_TypeDef *)(primDescr->USER))->primary;											// get prim/alt ping pong
	descr = (primary)?(primDescr):(altDescr);																			// get prim/alt descriptor
	remaining = ( descr->CTRL &_DMA_CTRL_N_MINUS_1_MASK )>> _DMA_CTRL_N_MINUS_1_SHIFT;
	iflag = DMA->IF;																															// read flag
	INT_Enable();																																	// enable ints again after read flag??
	if (!(( remaining == 0 ) && ( iflag & ( 1 << DMACh))))												// if we have not completed DMA for this channel
	{
		remaining++;																																// inc remaining count to get right number left
	}
	if(remaining == DMA_BUFF_LEN)																									// got nothing timer expiry
	{
	}
	else if(remaining)																														// if any data not received yet
	{
	  descr->CTRL &= ~(_DMA_CTRL_N_MINUS_1_MASK);																	// set the remaining DMA to 1 (or 0 for one item) and cross your fingers this works
	  PPMRxLen[primary] = DMA_BUFF_LEN - remaining+1;															// set how many values will be in dma buffer after next one completes
	}
	else
	{
	  PPMRxLen[primary] = DMA_BUFF_LEN;																						// could be we didn't see next positive edge or there were 12 channels
	}
}

static void SetupRxDMA(void)																										// setup rx dma to read timer periods into buffers with ping pong
{
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;
  cb.cbFunc  = RxDMAComplete;
  cb.userPtr = NULL;

  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_PPMRX;																							// trigger from timer capture compare
  chnlCfg.cb        = &cb;																											// set call back data
  DMA_CfgChannel(DMACh, &chnlCfg);																							// configure DMA channel
  descrCfg.dstInc  = dmaDataInc2;																								// 2 byte timer in dest buffer array
  descrCfg.srcInc  = dmaDataIncNone;																						// peripheral register source
  descrCfg.size    = dmaDataSize2;																							// data size is two bytes
  descrCfg.arbRate = dmaArbitrate1;																							// arbitrate every cycle
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMACh, true, &descrCfg);																					// config primary descriptor
  DMA_CfgDescr(DMACh, false, &descrCfg);																				// config alternate descriptor
  DMA_ActivatePingPong(DMACh,																										// activate ping pong, note will cause a glitch at start because we don't know the length yet
                       false,
                       (void *)DMABuffer[1],																		// prim dst
                       (void *)&(PPMTIMER->CC[PPMCCRX].CCV),										// prim src
                       DMA_BUFF_LEN - 1,																				// prim len
                       (void *)DMABuffer[0],																		// alt dst
                       (void *)&(PPMTIMER->CC[PPMCCRX].CCV), 	 	 	 	 	 	 	 	 	 	// alt src
                       DMA_BUFF_LEN - 1);																				// alt len
}

static void RxDMAComplete(unsigned int channel, bool primary, void *user)
{
  (void) user;
  DMA_RefreshPingPong(channel,primary,false,NULL,NULL,DMA_BUFF_LEN - 1,false);	// refresh to max length again
  LastPrimary = primary;																												// let user know which is the most fresh
}

static void SetupTxTimer(void)																									// setup tx timer to generate PWM signal directly on pin
{
	// set up timer either compare or PWM
	// top buff set update the registers on the next overflow
	// pingpong dma will be needed to load period
	// pingpong will load n channels of data before switching to alternate set
	// two buffers will be provided for the user to update
	// a lack of input should send default data
  CMU_ClockEnable(cmuClock_GPIO, true);																					// enable gpio clock
  CMU_ClockEnable(PPMTIMER_CLOCK, true);																				// enable timer clock
  GPIO_PinModeSet(PPMGPIO.Port, PPMGPIO.Pin, gpioModePushPull, 1);							// set pin to output, high
  TIMER_InitCC_TypeDef timerCCInit =																						// timer CC PWM
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
    .outInvert  = true,
  };

  TIMER_InitCC(PPMTIMER,PPMCCTX, &timerCCInit);
	PPMTIMER->ROUTE |= (PPMTIMER_ROUTE_CCTXPEN | PPMTIMER_ROUTE_LOCATION);				// route pin to timer cc
	TIMER_CompareSet(PPMTIMER,PPMCCTX,PULSE_WITDH_TICKS);													// set on time
  TIMER_Init_TypeDef timerInit =																								// timer approx 1mS clock, up count, don't enable
  {
    .enable     = false,																												// enable when started only
    .debugRun   = true,
    .prescale   = PPMCLOCKPRESC,																								// use same as detector
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,																					// no edge detection for pwm
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = true,																													// auto ack dma request flag so it will retrigger
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };
  TIMER_Init(PPMTIMER, &timerInit);
}

static void SetupTxDMA(void)																										// setup TX DMA , two channels high time and period setting
{
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;
  cb.cbFunc  = TxDMAComplete;
  cb.userPtr = NULL;
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_PPMTX;																							// trigger from CC for high period
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMACh, &chnlCfg);																							// configure first channel
  descrCfg.dstInc  = dmaDataIncNone;																						// dest is peripheral register
  descrCfg.srcInc  = dmaDataInc2;																								// source is data array
  descrCfg.size    = dmaDataSize2;																							// data is two bytes long
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMACh, true, &descrCfg);																					// config prim descriptor ch1
  DMA_CfgDescr(DMACh, false, &descrCfg);																				// config alt  descriptor ch1
  DMA_ActivatePingPong(DMACh,																										// activate ping pong ch1
                       false,
                       (void *)&(PPMTIMER->TOPB),																// prim dst
                       (void *)DMABuffer[1],																		// prim src
                       0,																												// prim len, don't know, 1 glitch
                       (void *)&(PPMTIMER->TOPB),																// alt dst
                       (void *)DMABuffer[0],																		// alt src
                       0);																											// alt len, don't know, 1 glitch
}

static void TxDMAComplete(unsigned int channel, bool primary, void *user)				// refresh sending of PWM period
{
  (void) user;
  static uint16_t DataMissing = 0;
  LastPrimary = primary;																												// let user know which is the most fresh
  if(!DataWaiting[!primary])																										// we have run out of data
	{
		if(DataMissing >= MAX_DATA_LOSS)																						// if too many consecutive packets lost
		{
			memcpy(DMABuffer[!primary],DataLossData,DataLossDataLen);									// copy in default data
			PPMTxLen = DataLossDataLen;
		}
		else																																				// else
		{
			memcpy(DMABuffer[!primary],DMABuffer[primary],PPMTxLen<<1);								// copy in current data to next
			DataMissing++;																														// inc lost packet counter
		}
	}
  else
  {
		DataMissing=0;																															// clear packet loss counter
  }
  DataWaiting[primary] = false;																									// clear flag data is sent
  DMA_RefreshPingPong(channel,primary,false,NULL,DMABuffer[primary],PPMTxLen-1,false);				// refresh dma
}

// ********************** ppm.c **************************************
