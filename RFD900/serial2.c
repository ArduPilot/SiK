/*
 * serial2.c
 *
 *  Created on: 04/02/2016
 *      Author: kentm
 *
 *  Serial module that uses DMA transmit and receive
 *  and also support Flow control
 *  It will use a ping pong dma for rx with a ring
 *  buffer split in two. flow control will be checked every half buffer
 *  i.e. when dma completes for each ping pong.
 *  the data will also be copied to a larger buffer which
 *  will determine the flow control levels.
 *
 *  For tx it will send small blocks of data and check
 *  flow control at the end of each block before continuing.
 *  A simple DMA mode can be used for this and get the call back function
 *  to set up the next transmit
 *  An interrupt on de assertion of flow control will be configured
 *  to restart the DMA when flow control released
 *
 *  Values to be configured will be the tx dma block length and
 *  Rx DMA block length, probably both the same
 */
#include <stdint.h>
#include <stdbool.h>
#include "em_cmu.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_timer.h"
#include "em_prs.h"
#include "gpiointerrupt.h"
#include "board_rfd900e.h"
#include "uart_config.h"
#include "timer_config.h"
#include "dmadrv.h"
#include "pins_user.h"
#include "radio_old.h"
#include "at.h"
#include "packet.h"
#include "serial.h"
#include "prs_config.h"
// ******************** defines and typedefs *************************
#define RX_TIMEOUT_MS     100

typedef struct {
	uint8_t rate;
	uint32_t BAUD;
} serial_rates_t;

#define RX_DMA_BLOCK_SIZE	32
#define TX_DMA_BLOCK_SIZE	32
#define RX_BUFF_MAX 2048			// must be a multiple of 2*RX_DMA_BUFF_SIZE
#define TX_BUFF_MAX 1024			// must be a multiple of TX_DMA_BUFF_SIZE
// threshold for considering the rx buffer full
#define SERIAL_CTS_THRESHOLD_LOW  64	// must be a multiple of RX_DMA_BLOCK_SIZE
#define SERIAL_CTS_THRESHOLD_HIGH 96  // must be a multiple of RX_DMA_BLOCK_SIZE

// FIFO insert/remove operations
#define BUF_NEXT_INSERT(_b)	((_b##_insert + 1) == sizeof(_b##_buf)?0:(_b##_insert + 1))
#define BUF_NEXT_REMOVE(_b)	((_b##_remove + 1) == sizeof(_b##_buf)?0:(_b##_remove + 1))
// BUG BUG BUG ! for circular buffers, if you allow the
// insert to meet remove then you don't know if it is completely full or empty
// if it gets to that point before flow control stops it
// return buff free -1 to stop this eventuation
#define BUF_FULL(_b)	(BUF_NEXT_INSERT(_b) == (_b##_remove))
#define BUF_NOT_FULL(_b)	(BUF_NEXT_INSERT(_b) != (_b##_remove))
#define BUF_EMPTY(_b)	(_b##_insert == _b##_remove)
#define BUF_NOT_EMPTY(_b)	(_b##_insert != _b##_remove)
#define BUF_USED_ISR(_b)	((_b##_insert >= _b##_remove)?(_b##_insert - _b##_remove):(sizeof(_b##_buf) - _b##_remove) + _b##_insert)
#define BUF_USED_SAFE(_b)	(((insert=_b##_insert) >= (myremove=_b##_remove))?(insert - myremove):(sizeof(_b##_buf) - myremove) + insert)
#define BUF_FREE_ISR(_b)	((_b##_insert >= _b##_remove)?(sizeof(_b##_buf) + _b##_remove - _b##_insert-1):_b##_remove - _b##_insert-1)
#define BUF_FREE_SAFE(_b)	(((insert=_b##_insert) >= (myremove=_b##_remove))?(sizeof(_b##_buf) + myremove - insert-1):myremove - insert-1)

#define BUF_INSERT(_b, _c)	do { _b##_buf[_b##_insert] = (_c); \
		_b##_insert = BUF_NEXT_INSERT(_b); } while(0)
#define BUF_REMOVE(_b, _c)	do { (_c) = _b##_buf[_b##_remove]; \
		_b##_remove = BUF_NEXT_REMOVE(_b); } while(0)
#define BUF_PEEK(_b)	_b##_buf[_b##_remove]
#define BUF_PEEK2(_b)	_b##_buf[BUF_NEXT_REMOVE(_b)]
#define BUF_PEEKX(_b, offset)	_b##_buf[(_b##_remove+offset) % sizeof(_b##_buf)]
// ******************** local variables ******************************
static UART_Config_t UART_Config = UART_SETUP;
static const serial_rates_t serial_rates[] =
{
	{	1, 1200}, // 1200
		{	2, 2400}, // 2400
		{	4, 4800}, // 4800
		{	9, 9600}, // 9600
		{	19, 19200}, // 19200
		{	38, 38400}, // 38400
		{	57, 57600}, // 57600 - default
		{	115, 115200}, // 115200
		{	230, 230400}, // 230400
	};
//static const GPIO_Port_Pin_TypeDef CTS_PORT= SERIAL_CTS;
static const GPIO_Port_Pin_TypeDef RTS_PORT= SERIAL_RTS;
static uint8_t rx_buf[RX_BUFF_MAX] ={	0};
static uint8_t tx_buf[TX_BUFF_MAX] ={	0};
static volatile uint16_t rx_insert=0,rx_remove =0;
static volatile uint16_t tx_insert= 0,tx_remove = 0;
static uint16_t insert, myremove;							// for safe macros
static bool tx_idle = true;
static unsigned int DMATxCh = 0;
static unsigned int DMARxCh = 0;
static uint8_t * rxDmaDst[2] = { rx_buf + RX_DMA_BLOCK_SIZE, rx_buf };
static DMA_DESCRIPTOR_TypeDef *RxPrimDescr;
static DMA_DESCRIPTOR_TypeDef *RxAltDescr;
// ******************** local function prototypes ********************
static void Init_Serial(uint8_t speed);
static uint8_t serial_device_set_speed(register uint8_t speed);
static void serial_restart_fromISR(bool FromISR);
#define serial_restart() serial_restart_fromISR(false);
static bool dmaTransferDone(unsigned int channel, unsigned int seqNo,
		void *user);
static void RTSIrqCB(uint8_t pin);
static void setupRxDma(void);
//static void setupRxTimer(void);
static void rxDmaComplete(unsigned int channel, bool primary, void *user);
// ********************* Implementation ******************************
void serial_init(uint8_t speed)
{
	// setup tx dma channel on txdouble empty, call back to check flow control
	// after each DMA_BLOCK_SIZE bytes
	// setup rx dma on ping pong rxdatav for DMA_BLOCK_SIZE bytes, incrementing
	// alternate address each time, be careful of end of buffer as may not
	// align with dma transfer block size if dma is not complete when times out
	// set up a timer to reset on any rx pin edge. when expired (say 200mS)update the
	// rx buffer index, so small byte count can be read, be warned this delay may affect
	// data packets whose last bytes and held in the buffer until timeout or until
	// the next packet comes along, effectively delaying the end and hence the completed packet.
	Init_Serial(speed);
	DMADRV_Init();
	DMADRV_AllocateChannel(&DMATxCh, NULL );
	DMADRV_AllocateChannel(&DMARxCh, NULL );
	RxPrimDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->CTRLBASE)) + DMARxCh;// Get primary descriptor
	RxAltDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->ALTCTRLBASE)) + DMARxCh;// Get alternate descriptor

	GPIOINT_CallbackRegister(RTS_PORT.Pin, RTSIrqCB);
	GPIO_IntConfig(RTS_PORT.Port, RTS_PORT.Pin, false, true, true);
	setupRxDma();
	//setupRxTimer();
	//static const char hello[] = "hello how are won't you jump in my game?";
	//serial_write_buf((uint8_t*)hello, strlen(hello));
}

static void Init_Serial(uint8_t speed)
{
	uint8_t idx;

	GPIOSet(UART_Config.GPIOCTS, false);
	idx = serial_device_set_speed(speed);
	GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortD, 6, gpioModeInput, 0);

	UART_Config_t *Config = &UART_Config;
	if (NULL != Config->USART)
	{
		USART_InitAsync_TypeDef Init = USART_INITASYNC_DEFAULT;
		CMU_ClockEnable(Config->clock, true);
		Init.baudrate = serial_rates[idx].BAUD;
		Init.databits =
				(Databits8 == Config->Databits) ? (usartDatabits8) : (usartDatabits9);
		Init.parity =
				(NoParity == Config->Parity) ?
						(usartNoParity) :
						((EvenParity == Config->Parity) ?
								(usartEvenParity) : (usartOddParity));
		Init.stopbits =
				(Stopbits1 == Config->Stopbits) ? (usartStopbits1) : (usartStopbits2);
		Init.enable = false;  // enable later after everything set up
		Init.mvdis = 0;
		Init.prsRxEnable = 0;
		Init.prsRxCh = 0;

		USART_Reset(Config->USART);
		USART_InitAsync(Config->USART, &Init);
		// configure location
		Config->USART->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN
				| Config->location;
		USART_Enable(Config->USART, usartEnable); // enable rx and tx
	}
}

// RTS negative edge just occured, re enable transmit when this happens
static void RTSIrqCB(uint8_t pin)
{
	if (tx_idle)
	{
		serial_restart_fromISR(true);
	}
}

static bool dmaTransferDone(unsigned int channel, unsigned int seqNo,
		void *user)
{
	(void) channel;
	(void) seqNo;
	tx_remove += (uint32_t) user;
	if (tx_remove >= sizeof(tx_buf))
		tx_remove -= sizeof(tx_buf);
	tx_idle = true;
	serial_restart_fromISR(true);
	return (true);
}

static void serial_restart_fromISR(bool FromISR)
{
	if ((!tx_idle)
			|| (feature_rtscts && GPIOGet(UART_Config.GPIORTS) && !at_mode_active))
	{
		// the line is blocked by hardware flow control
		return;
	}
	void *src;
	uint16_t len = (FromISR) ? (BUF_USED_ISR(tx)) : (BUF_USED_SAFE(tx));
	if(len)
	{
		if (len > TX_DMA_BLOCK_SIZE)
			len = TX_DMA_BLOCK_SIZE;
		if ((len + tx_remove) > sizeof(tx_buf))
			len = sizeof(tx_buf) - tx_remove;
		// start the dma transfer again
		src = &tx_buf[tx_remove];
		tx_idle = false;
		DMADRV_MemoryPeripheral(DMATxCh, dmadrvPeripheralSignal_USART1_TXBL,
				(void*) &UART_Config.USART->TXDATA, src, 1, len, dmadrvDataSize1,
				dmaTransferDone, (void*) ((uint32_t) len));
	}
}

// **************************** serial tx functions ******************
// return how much space in tx buffer
uint16_t serial_write_space(void)
{
	return (BUF_FREE_SAFE(tx));
}

// write as many bytes as will fit into the serial transmit buffer
void serial_write_buf(uint8_t * buf, uint8_t count)
{
	uint16_t space;
	uint8_t n1;

	if (count == 0)
	{
		return;
	}

	// discard any bytes that don't fit. We can't afford to
	// wait for the buffer to drain as we could miss a frequency
	// hopping transition
	space = serial_write_space();
	if (count > space)
	{
		count = space;
		if (errors.serial_tx_overflow != 0xFFFF)
		{
			errors.serial_tx_overflow++;
		}
	}

	// write to the end of the ring buffer
	n1 = count;
	if (n1 > sizeof(tx_buf) - tx_insert)
	{
		n1 = sizeof(tx_buf) - tx_insert;
	}
	memcpy(&tx_buf[tx_insert], buf, n1);
	buf += n1;
	count -= n1;
	insert = tx_insert;
	insert += n1;
	if (insert >= sizeof(tx_buf))
	{
		insert -= sizeof(tx_buf);
	}
	tx_insert = insert;
	// add any leftover bytes to the start of the ring buffer
	if (count != 0)
	{
		memcpy(&tx_buf[0], buf, count);
		{
			tx_insert = count;
		}
	}
	{
		if (tx_idle)
		{
			serial_restart();
		}
	}
}

void putChar(char c)
{
	if (c == '\n')
	{
		uint8_t cr = '\r';
		serial_write_buf(&cr, 1);
	}
	serial_write_buf((uint8_t*) &c, 1);
}

//
// check if a serial speed is valid
//
bool serial_device_valid_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			return true;
		}
	}
	return false;
}

// **************************** serial rx functions ******************

uint8_t serial_read(void)
{
	register uint8_t c;
	if (BUF_NOT_EMPTY(rx))
	{
		BUF_REMOVE(rx, c);
	}
	else
	{
		c = '\0';
	}

	if (BUF_FREE_SAFE(rx) > SERIAL_CTS_THRESHOLD_HIGH)
	{
		GPIOSet(UART_Config.GPIOCTS, false);
	}
	return c;
}

uint8_t serial_peek(void)
{
	register uint8_t c;
	c = BUF_PEEK(rx);
	return c;
}

uint8_t serial_peek2(void)
{
	register uint8_t c;
	c = BUF_PEEK2(rx);
	return c;
}

uint8_t serial_peekx(uint16_t offset)
{
	register uint8_t c;
	c = BUF_PEEKX(rx, offset);
	return c;
}

// read count bytes from the serial buffer. This implementation
// tries to be as efficient as possible, without disabling interrupts
bool serial_read_buf(uint8_t * buf, uint8_t count)
{
	uint16_t n1;
	// the caller should have already checked this,
	// but lets be sure
	if (count > serial_read_available())
	{
		return false;
	}
	// see how much we can copy from the tail of the buffer
	n1 = count;
	if (n1 > sizeof(rx_buf) - rx_remove)
	{
		n1 = sizeof(rx_buf) - rx_remove;
	}
	memcpy(buf, &rx_buf[rx_remove], n1);
	count -= n1;
	buf += n1;
	// update the remove marker being careful to only write it with valid values
	myremove = rx_remove;
	myremove += n1;
	if (myremove >= sizeof(rx_buf))
	{
		myremove -= sizeof(rx_buf);
	}
	rx_remove = myremove;
	// any more bytes to do?
	if (count > 0)
	{
		memcpy(buf, &rx_buf[0], count);
		{
			rx_remove = count;
		}
	}

	if (BUF_FREE_SAFE(rx) > SERIAL_CTS_THRESHOLD_HIGH)
	{
		GPIOSet(UART_Config.GPIOCTS, false);
	}
	return true;
}

uint16_t serial_read_available(void)
{
	return (BUF_USED_SAFE(rx));
}

// return available space in rx buffer as a percentage
uint8_t serial_read_space(void)
{
	uint16_t space = sizeof(rx_buf) - serial_read_available();
	space = (100 * (space / 8)) / (sizeof(rx_buf) / 8);
	return space;
}

static uint8_t serial_device_set_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++)
	{
		if (speed == serial_rates[i].rate)
		{
			break;
		}
	}
	if (i == num_rates)
	{
		i = 6; // 57600 default
	}

	// tell the packet layer how fast the serial link is. This is
	// needed for packet framing timeouts
	packet_set_serial_speed(speed * 125UL);
	return (i);
}
// ********************* serial2.c ***********************************

/**************************************************************************//**
 * @brief  Call-back called when RX is complete
 *****************************************************************************/
static void rxDmaComplete(unsigned int channel, bool primary, void *user)
{
	rx_insert = rxDmaDst[primary] + RX_DMA_BLOCK_SIZE - rx_buf;// increment the amount we just added
	if (rx_insert >= sizeof(rx_buf))
		rx_insert = 0;  														// note must always realign to 0
	rxDmaDst[primary] += (2 * RX_DMA_BLOCK_SIZE);	// set dst to end of other half
	if (rxDmaDst[primary] >= (rx_buf + sizeof(rx_buf)+RX_DMA_BLOCK_SIZE))// if end goes past end of buffer
	{
		rxDmaDst[primary] -= sizeof(rx_buf);				// subtract buffer length to get address nearest start
	}
	// Re-arm DMA channel for RX
	DMA_RefreshPingPong(channel, primary, false, rxDmaDst[primary], NULL,
			RX_DMA_BLOCK_SIZE - 1, false);
	if (BUF_FREE_ISR(rx) < SERIAL_CTS_THRESHOLD_LOW)
	{
		GPIOSet(UART_Config.GPIOCTS,true);
	}
}

void Serial_Check(void)																													// save a timer and check serial port for inactivity every 100mS
{
	static bool primary,lastPrimary = true;
	primary = ((DMA_CB_TypeDef *)(RxPrimDescr->USER))->primary;
	if(primary == lastPrimary)
	{
		static uint16_t NMinus1,lastNMinus1 = 0;
		static DMA_DESCRIPTOR_TypeDef *currDescr;
		currDescr = (((DMA_CB_TypeDef *)(RxPrimDescr->USER))->primary)?(RxPrimDescr):(RxAltDescr);
		NMinus1 = ((currDescr->CTRL&_DMA_CTRL_N_MINUS_1_MASK)>>_DMA_CTRL_N_MINUS_1_SHIFT);
		if(lastNMinus1 == NMinus1)
		{
			uint8_t c;
			static uint16_t lastRxInsert;
			rx_insert = rxDmaDst[primary]+RX_DMA_BLOCK_SIZE-rx_buf-NMinus1-1;
			if(rx_insert >= sizeof(rx_buf))
			{
				rx_insert = 0;
			}
			if (at_mode_active)																												// if AT mode is active, the AT processor owns the byte
			{
				while(BUF_NOT_EMPTY(rx))																								// if any data available
				{
					BUF_REMOVE(rx,c);																											// remove last data from queue
					if (!at_cmd_ready)																										// If an AT command is ready/being processed, we would ignore this byte
					{
						at_input(c);																												// parse at byte
					}
				}
			}
			else																																			// else not at mode
			{
				if(lastRxInsert != rx_insert)																						// if another byte has been inserted
				{
					if(0 == rx_insert){c = rx_buf[sizeof(rx_buf)-1];}											// get last byte, don't dequeue
					else							{c = rx_buf[rx_insert-1];}
					at_plus_detector(c);																									// run the last byte past the +++ detector
				}
			}
			lastRxInsert = rx_insert;
		}
		lastNMinus1 = NMinus1;
	}
	lastPrimary = primary;
}
/**************************************************************************//**
 * @brief  TIMER? Interrupt handler
 * The TIMER OF interrupt is triggered when the RX line has been stationary for
 * longer than the timout period. This ISR then checks how many bytes have been
 * transferred and updates the rx_insert accordingly.
 *****************************************************************************/
#if 0
void RXTIMER_IRQHandler(void)
{
	uint8_t c;
	static uint16_t lastRxInsert;
	DMA_DESCRIPTOR_TypeDef *primDescr;
	DMA_DESCRIPTOR_TypeDef *altDescr;
	DMA_CB_TypeDef *dmaCallback;
	TIMER_IntClear(RXTIMER, TIMER_IFC_OF);									// Clear interrupt flag
	primDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->CTRLBASE)) + DMARxCh;// Get primary descriptor
	altDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->ALTCTRLBASE)) + DMARxCh;// Get alternate descriptor
	dmaCallback = (DMA_CB_TypeDef *)(primDescr->USER);// Get callback to check if primary or alternate structure is used
	// Check in alternate or primary descriptors how many bytes were transferred
	// and move rx_insert accordingly
	if (dmaCallback->primary)
	{
		rx_insert = rxDmaDst[1]+RX_DMA_BLOCK_SIZE-rx_buf-
		((primDescr->CTRL&_DMA_CTRL_N_MINUS_1_MASK)>>_DMA_CTRL_N_MINUS_1_SHIFT)-1;
	}
	else
	{
		rx_insert = rxDmaDst[0]+RX_DMA_BLOCK_SIZE-rx_buf-
		((altDescr->CTRL&_DMA_CTRL_N_MINUS_1_MASK)>>_DMA_CTRL_N_MINUS_1_SHIFT)-1;
	}
	if(rx_insert >= sizeof(rx_buf))
	{
		rx_insert = 0;
	}
	if (at_mode_active)												// if AT mode is active, the AT processor owns the byte
	{
		while(BUF_NOT_EMPTY(rx))										// if any data avilable
		{
			BUF_REMOVE(rx,c);										// remove last data from queue
			if (!at_cmd_ready)										// If an AT command is ready/being processed, we would ignore this byte
			{
				at_input(c);												// parse at byte
			}
		}
	}
	else																			// else not at mode
	{
		if(lastRxInsert != rx_insert)						// if another byte has been inserted
		{
			if(0 == rx_insert){c = rx_buf[sizeof(rx_buf)-1];}		// get last byte, don't dequeue
			else							{c = rx_buf[rx_insert-1];}
			at_plus_detector(c);									// run the last byte past the +++ detector
		}
	}
	lastRxInsert = rx_insert;
}

/**************************************************************************//**
 * @brief  Setup Timer
 * Configure TIMER to reset and start counter every time the RX pin on
 * the UART has a falling edge. If the RX line is idle for longer than the
 * timeout period the overflow interrupt is called indicating that a full
 * message has been received.
 *****************************************************************************/
static void setupRxTimer(void)
{
	TIMER_Init_TypeDef init = TIMER_INIT_DEFAULT;
	TIMER_InitCC_TypeDef initCc = TIMER_INITCC_DEFAULT;

  CMU_ClockEnable(RXTIMER_CLOCK, true);
  CMU_ClockEnable(cmuClock_PRS, true);
	// Configure TIMERx to set a DMA request on falling input on CCx
	init.fallAction = timerInputActionReloadStart;// Reload and Start TIMER on falling edge on CC0 input
	init.oneShot = true;                         	 	 // One shot, stop on overflow
	init.enable = false;                        	 	 	 	// Do not start timer
	init.prescale = timerPrescale1024;            	 	 	 	// Prescale by 1024
	TIMER_Init(RXTIMER, &init);

	initCc.prsInput = true;									// Configure CCx to listen to PRS CH3
	initCc.prsSel = timerPRSSELCh3;
	TIMER_InitCC(RXTIMER, 0, &initCc);
	// Set TOP value according to timout value
	TIMER_TopSet(RXTIMER,(CMU_ClockFreqGet(RXTIMER_CLOCK) / 1000) * RX_TIMEOUT_MS / 1024);
	GPIO_InputSenseSet(GPIO_INSENSE_PRS, GPIO_INSENSE_PRS);// Enable input sensing for PRS
	GPIO_IntConfig(gpioPortD, 6, false, false, false);// disable PRS/GPIO interrupts
																										// PRS channel 3 listen to pin 6
	PRS_SourceSignalSet(SERPRS, PRS_CH_CTRL_SOURCESEL_GPIOL,
			PRS_CH_CTRL_SIGSEL_GPIOPIN6, prsEdgeOff);
	TIMER_IntEnable(RXTIMER, TIMER_IEN_OF);// Generate interrupt on overflow (timeout)
	NVIC_EnableIRQ(RXTIMER_IRQn);			// enable Timer IRQ to take action on expiry
}
#endif

/**************************************************************************//**
 * @brief Configure DMA for UART RX
 * RX uses ping pong, where the primary and alternate descriptors operate on
 * on a rotating basis along the length of the rx buffer
 *****************************************************************************/
static void setupRxDma(void)
{
	static DMA_CB_TypeDef cb;
	DMA_CfgChannel_TypeDef rxChnlCfg;
	DMA_CfgDescr_TypeDef rxDescrCfg;

	cb.cbFunc = rxDmaComplete;										// Setting up call-back function
	cb.userPtr = NULL;
	rxChnlCfg.highPri = false;															// Setting up channel
	rxChnlCfg.enableInt = true;
	rxChnlCfg.select = DMAREQ_USART1_RXDATAV;
	rxChnlCfg.cb = &cb;
	DMA_CfgChannel(DMARxCh, &rxChnlCfg);

	rxDescrCfg.dstInc = dmaDataInc1;							// Setting up channel descriptor
	rxDescrCfg.srcInc = dmaDataIncNone;
	rxDescrCfg.size = dmaDataSize1;
	rxDescrCfg.arbRate = dmaArbitrate1;
	rxDescrCfg.hprot = 0;
	DMA_CfgDescr(DMARxCh, true, &rxDescrCfg);
	DMA_CfgDescr(DMARxCh, false, &rxDescrCfg);
	// Activate Alternate and Primary channels
	DMA_ActivatePingPong(DMARxCh, false, (void *) rx_buf,
			(void *) &(UART_Config.USART->RXDATA), RX_DMA_BLOCK_SIZE - 1,
			(void *) (rx_buf + RX_DMA_BLOCK_SIZE),
			(void *) &(UART_Config.USART->RXDATA), RX_DMA_BLOCK_SIZE - 1);
}
