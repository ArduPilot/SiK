
///
/// @file	serial.c
///
/// MCS51 Serial port driver with flow control and AT command
/// parser integration.
///

#include <stdint.h>
#include <stdbool.h>
#include "radio_old.h"
#include "serial.h"
#include "packet.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_leuart.h"
#include "board_rfd900e.h"
#include "at.h"
#include "pins_user.h"
#include "uart_config.h"

// ******************** defines and typedefs *************************
///
/// Table of supported serial speed settings.
/// the table is looked up based on the 'one byte'
/// serial rate scheme that APM uses. If an unsupported
/// rate is chosen then 57600 is used
///
typedef struct {
	uint8_t rate;
	uint32_t BAUD;
} serial_rates_t;

#define RX_BUFF_MAX 2048
#define TX_BUFF_MAX 1024

// threshold for considering the rx buffer full
#define SERIAL_CTS_THRESHOLD_LOW  17
#define SERIAL_CTS_THRESHOLD_HIGH 34
// ******************** local variables ******************************
static UART_Config_t UART_Config = UART_SETUP;
static const 	serial_rates_t serial_rates[] = {
	{1,   1200  }, // 1200
	{2,   2400  }, // 2400
	{4,   4800  }, // 4800
	{9,   9600  }, // 9600
	{19,  19200 }, // 19200
	{38,  38400 }, // 38400
	{57,  57600 }, // 57600 - default
	{115, 115200}, // 115200
	{230, 230400}, // 230400
};
// Serial rx/tx buffers.
//
// Note that the rx buffer is much larger than you might expect
// as we need the receive buffer to be many times larger than the
// largest possible air packet size for efficient TDM. Ideally it
// would be about 16x larger than the largest air packet if we have
// 8 TDM time slots
//

static uint8_t rx_buf[RX_BUFF_MAX] = {0};
static uint8_t tx_buf[TX_BUFF_MAX] = {0};

// FIFO insert/remove pointers
static volatile uint16_t				rx_insert, rx_remove;
static volatile uint16_t				tx_insert, tx_remove;

// flag indicating the transmitter is idle
static volatile bool			tx_idle;

// ******************** global variables *****************************
// ******************** local function prototypes ********************
// FIFO status
#define BUF_NEXT_INSERT(_b)	((_b##_insert + 1) == sizeof(_b##_buf)?0:(_b##_insert + 1))
#define BUF_NEXT_REMOVE(_b)	((_b##_remove + 1) == sizeof(_b##_buf)?0:(_b##_remove + 1))
#define BUF_FULL(_b)	(BUF_NEXT_INSERT(_b) == (_b##_remove))
#define BUF_NOT_FULL(_b)	(BUF_NEXT_INSERT(_b) != (_b##_remove))
#define BUF_EMPTY(_b)	(_b##_insert == _b##_remove)
#define BUF_NOT_EMPTY(_b)	(_b##_insert != _b##_remove)
#define BUF_USED(_b)	((_b##_insert >= _b##_remove)?(_b##_insert - _b##_remove):(sizeof(_b##_buf) - _b##_remove) + _b##_insert)
#define BUF_FREE(_b)	((_b##_insert >= _b##_remove)?(sizeof(_b##_buf) + _b##_remove - _b##_insert):_b##_remove - _b##_insert)

// FIFO insert/remove operations
//
// Note that these are nominally interrupt-safe as only one of each
// buffer's end pointer is adjusted by either of interrupt or regular
// mode code.  This is violated if printing from interrupt context,
// which should generally be avoided when possible.
//
#define BUF_INSERT(_b, _c)	do { _b##_buf[_b##_insert] = (_c); \
		_b##_insert = BUF_NEXT_INSERT(_b); } while(0)
#define BUF_REMOVE(_b, _c)	do { (_c) = _b##_buf[_b##_remove]; \
		_b##_remove = BUF_NEXT_REMOVE(_b); } while(0)
#define BUF_PEEK(_b)	_b##_buf[_b##_remove]
#define BUF_PEEK2(_b)	_b##_buf[BUF_NEXT_REMOVE(_b)]
#define BUF_PEEKX(_b, offset)	_b##_buf[(_b##_remove+offset) % sizeof(_b##_buf)]

static void			_serial_write(register uint8_t c);
static void			serial_restart(void);
static uint8_t serial_device_set_speed(register uint8_t speed);

__STATIC_INLINE void XUARTX_RX_IRQHandler(uint8_t c);
// ********************* Implementation ******************************

__STATIC_INLINE void XUARTX_RX_IRQHandler(uint8_t c)
{
	if (at_mode_active)											// if AT mode is active, the AT processor owns the byte
	{
		if (!at_cmd_ready)										// If an AT command is ready/being processed, we would ignore this byte
		{
			at_input(c);
		}
	}
	else
	{
		at_plus_detector(c);									// run the byte past the +++ detector
		if (BUF_NOT_FULL(rx))									// and queue it for general reception
		{
			BUF_INSERT(rx, c);
		}
		else
		{
			if (errors.serial_rx_overflow != 0xFFFF)
			{
				errors.serial_rx_overflow++;
			}
		}
#ifdef SERIAL_CTS
		if (BUF_FREE(rx) < SERIAL_CTS_THRESHOLD_LOW)
		{
			GPIOSet(UART_Config.GPIOCTS,true);
		}
#endif
	}
}

__STATIC_INLINE void XUARTX_TX_IRQHandler(LEUART_TypeDef* LEUARTX,USART_TypeDef* USARTX)
{
	uint8_t c;

	if (BUF_NOT_EMPTY(tx))								// look for another byte we can send
	{
#ifdef SERIAL_RTS
		static uint8_t rts_count=0;						// count of number of bytes we are allowed to send due to a RTS low reading
		if (feature_rtscts)
		{
			if (GPIOGet(UART_Config.GPIORTS) && !at_mode_active)
			{
				if (rts_count == 0)
				{
					tx_idle = true;									// the other end doesn't have room in its serial buffer
					return;
				}
				rts_count--;
			}
			else
			{
				rts_count = 8;
			}
		}
#endif
		// fetch and send a byte
		BUF_REMOVE(tx, c);
		{
			USART_Tx(USARTX, c);
		}
	}
	else
	{
		tx_idle = true;												// note that the transmitter requires a kick to restart it
	}
}


__STATIC_INLINE void USARTX_RX_IRQHandler(USART_TypeDef* USARTX)
{
	uint8_t c = USARTX->RXDATA;
	USARTX->IFC = USART_IF_RXDATAV;
	XUARTX_RX_IRQHandler(c);
}
__STATIC_INLINE void USARTX_TX_IRQHandler(USART_TypeDef* USARTX)
{
	USARTX->IFC = USART_IF_TXC;
	XUARTX_TX_IRQHandler(NULL,USARTX);
}

// remove any not needed, or leave them
void USART1_RX_IRQHandler(void){	USARTX_RX_IRQHandler(USART1);}
void USART1_TX_IRQHandler(void){	USARTX_TX_IRQHandler(USART1);}


/// check if RTS allows us to send more data
///
void Serial_Check(void)
//void serial_check_rts(void)
{
	if (BUF_NOT_EMPTY(tx) && tx_idle) {
		serial_restart();
	}
}

void serial_init(register uint8_t speed)
{
	uint8_t idx=0;
	// reset buffer state, discard all data
	rx_insert = 0;
	tx_remove = 0;
	tx_insert = 0;
	tx_remove = 0;
	tx_idle = true;

#ifdef SERIAL_CTS
	// setting SERIAL_CTS low tells the other end that we have
	// buffer space available
	GPIOSet(UART_Config.GPIOCTS,false);
#endif
	idx = serial_device_set_speed(speed);

	GPIO_PinModeSet(gpioPortD,7,gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortD,6,gpioModeInput   , 0);

	UART_Config_t *Config = &UART_Config;
  if(NULL != Config->USART)
  {
    USART_InitAsync_TypeDef Init = USART_INITASYNC_DEFAULT;
    CMU_ClockEnable(Config->clock, true);
    Init.baudrate = serial_rates[idx].BAUD;
    Init.databits = (Databits8==Config->Databits)?(usartDatabits8):(usartDatabits9);
    Init.parity = (NoParity==Config->Parity)?(usartNoParity):((EvenParity==Config->Parity)?(usartEvenParity):(usartOddParity));
    Init.stopbits = (Stopbits1 == Config->Stopbits)?(usartStopbits1):(usartStopbits2);
    Init.enable = false;  // enable later after everything set up
    USART_InitAsync(Config->USART,&Init);

    USART_IntClear(Config->USART, _USART_IFC_MASK  );
    USART_IntEnable(Config->USART,USART_IEN_RXDATAV);
    USART_IntEnable(Config->USART,USART_IEN_TXC);
    NVIC_ClearPendingIRQ(((USART1 == Config->USART)?(USART1_TX_IRQn):
                                                   (USART2_TX_IRQn)));
    NVIC_EnableIRQ(((USART1 == Config->USART)?(USART1_TX_IRQn):
                                              (USART2_TX_IRQn)));
    NVIC_ClearPendingIRQ(((USART1 == Config->USART)?(USART1_RX_IRQn):
                                                   (USART2_RX_IRQn)));
    /* Enable the correspondent vector to the processor */
    NVIC_EnableIRQ(((USART1 == Config->USART)?(USART1_RX_IRQn):
                                            (USART2_RX_IRQn)));
    // configure location
    Config->USART->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | Config->location;
    USART_Enable(Config->USART,usartEnable); // enable rx and tx
  }
}

bool serial_write(register uint8_t c)
{
	if (serial_write_space() < 1)
		return false;

	_serial_write(c);
	return true;
}

__STATIC_INLINE void EnableRXIRQ(bool On)
{
	if(NULL != UART_Config.USART)
	{
		(On)?(USART_IntEnable(UART_Config.USART,USART_IEN_RXDATAV)):
				(USART_IntDisable(UART_Config.USART,USART_IEN_RXDATAV));
	}
	else
	{
		(On)?(LEUART_IntEnable(UART_Config.LEUART,LEUART_IEN_RXDATAV)):
				(LEUART_IntDisable(UART_Config.LEUART,LEUART_IEN_RXDATAV));
	}
}

__STATIC_INLINE void EnableTXIRQ(bool On)
{
	if(NULL != UART_Config.USART)
	{
		(On)?(USART_IntEnable(UART_Config.USART,USART_IEN_TXC)):
				(USART_IntDisable(UART_Config.USART,USART_IEN_TXC));
	}
	else
	{
		(On)?(LEUART_IntEnable(UART_Config.LEUART,LEUART_IEN_TXC)):
				(LEUART_IntDisable(UART_Config.LEUART,LEUART_IEN_TXC));
	}
}

static void _serial_write(register uint8_t c)
{
	EnableTXIRQ(false);
	// if we have space to store the character
	if (BUF_NOT_FULL(tx)) {

		// queue the character
		BUF_INSERT(tx, c);

		// if the transmitter is idle, restart it
		if (tx_idle)
			serial_restart();
	} else if (errors.serial_tx_overflow != 0xFFFF) {
		errors.serial_tx_overflow++;
	}
	EnableTXIRQ(true);
}

// write as many bytes as will fit into the serial transmit buffer
void serial_write_buf(uint8_t * buf, uint8_t count)
{
	uint16_t space;
	uint8_t n1;

	if (count == 0) {
		return;
	}

	// discard any bytes that don't fit. We can't afford to
	// wait for the buffer to drain as we could miss a frequency
	// hopping transition
	space = serial_write_space();	
	if (count > space) {
		count = space;
		if (errors.serial_tx_overflow != 0xFFFF) {
			errors.serial_tx_overflow++;
		}
	}

	// write to the end of the ring buffer
	n1 = count;
	if (n1 > sizeof(tx_buf) - tx_insert) {
		n1 = sizeof(tx_buf) - tx_insert;
	}
	memcpy(&tx_buf[tx_insert], buf, n1);
	buf += n1;
	count -= n1;
	{
		tx_insert += n1;
		if (tx_insert >= sizeof(tx_buf)) {
			tx_insert -= sizeof(tx_buf);
		}
	}

	// add any leftover bytes to the start of the ring buffer
	if (count != 0) {
		memcpy(&tx_buf[0], buf, count);
		{
			tx_insert = count;
		}		
	}
	{
		if (tx_idle) {
			serial_restart();
		}
	}
}

uint16_t serial_write_space(void)
{
	uint16_t ret;
	EnableTXIRQ(false);
	ret = BUF_FREE(tx);
	EnableTXIRQ(true);
	return ret;
}

static void serial_restart(void)
{
#ifdef SERIAL_RTS
	if (feature_rtscts && GPIOGet(UART_Config.GPIORTS) && !at_mode_active) {
		// the line is blocked by hardware flow control
		return;
	}
#endif
	// generate a transmit-done interrupt to force the handler to send another byte
	tx_idle = false;
  (NULL != UART_Config.USART)?(UART_Config.USART->IFS = USART_IFS_TXC):(UART_Config.LEUART->IFS = LEUART_IFS_TXC);
}

uint8_t serial_read(void)
{
	register uint8_t	c;

	EnableRXIRQ(false);
	if (BUF_NOT_EMPTY(rx)) {
		BUF_REMOVE(rx, c);
	} else {
		c = '\0';
	}

#ifdef SERIAL_CTS
	if (BUF_FREE(rx) > SERIAL_CTS_THRESHOLD_HIGH)
	{
		GPIOSet(UART_Config.GPIOCTS,false);
	}
#endif
	EnableRXIRQ(true);
	return c;
}

uint8_t serial_peek(void)
{
	register uint8_t c;
	EnableRXIRQ(false);
	c = BUF_PEEK(rx);
	EnableRXIRQ(true);
	return c;
}

uint8_t serial_peek2(void)
{
	register uint8_t c;
	EnableRXIRQ(false);
	c = BUF_PEEK2(rx);
	EnableRXIRQ(true);
	return c;
}

uint8_t serial_peekx(uint16_t offset)
{
	register uint8_t c;
	EnableRXIRQ(false);
	c = BUF_PEEKX(rx, offset);
	EnableRXIRQ(true);
	return c;
}

// read count bytes from the serial buffer. This implementation
// tries to be as efficient as possible, while disabling interrupts
// for as short a time as possible
bool serial_read_buf(uint8_t * buf, uint8_t count)
{
	uint16_t n1;
	// the caller should have already checked this, 
	// but lets be sure
	if (count > serial_read_available()) {
		return false;
	}
	// see how much we can copy from the tail of the buffer
	n1 = count;
	if (n1 > sizeof(rx_buf) - rx_remove) {
		n1 = sizeof(rx_buf) - rx_remove;
	}
	memcpy(buf, &rx_buf[rx_remove], n1);
	count -= n1;
	buf += n1;
	// update the remove marker with interrupts disabled
	{
		rx_remove += n1;
		if (rx_remove >= sizeof(rx_buf)) {
			rx_remove -= sizeof(rx_buf);
		}
	}
	// any more bytes to do?
	if (count > 0) {
		memcpy(buf, &rx_buf[0], count);
		{
			rx_remove = count;
		}		
	}

#ifdef SERIAL_CTS
	{
		if (BUF_FREE(rx) > SERIAL_CTS_THRESHOLD_HIGH) \
		{
			GPIOSet(UART_Config.GPIOCTS,false);
		}
	}
#endif
	return true;
}

uint16_t serial_read_available(void)
{
	register uint16_t ret;
	EnableRXIRQ(false);
	ret = BUF_USED(rx);
	EnableRXIRQ(true);
	return ret;
}

// return available space in rx buffer as a percentage
uint8_t serial_read_space(void)
{
	uint16_t space = sizeof(rx_buf) - serial_read_available();
	space = (100 * (space/8)) / (sizeof(rx_buf)/8);
	return space;
}

void putChar (char c)
{
	if (c == '\n')
	{
		_serial_write('\r');
	}
	_serial_write(c);
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

static uint8_t serial_device_set_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			break;
		}
	}
	if (i == num_rates) {
		i = 6; // 57600 default
	}

	// tell the packet layer how fast the serial link is. This is
	// needed for packet framing timeouts
	packet_set_serial_speed(speed*125UL);
	return(i);
}


// **************************** end of seial.c *********************************
