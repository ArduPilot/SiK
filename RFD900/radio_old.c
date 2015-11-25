/*
 * radio_old.c
 *
 *  Created on: 06/11/2015
 *      Author: kentm
 */

#include <stdint.h>
#include <stdbool.h>
#include "radio_old.h"
// ******************** defines and typedefs *************************
#define NUM_DATA_RATES 13
// ******************** local variables ******************************
// ******************** global variables *****************************
bool feature_golay = false;
bool feature_opportunistic_resend = false;
uint8_t feature_mavlink_framing = false;
bool feature_rtscts = false;
statistics_t statistics, remote_statistics;
error_counts_t errors={0};

// Board infop
const char 		g_version_string[]= "blah";	///< printable version string
const char 		g_banner_string[] ="yoyo";	///< printable startup banner string
enum BoardFrequency	g_board_frequency;	///< board RF frequency from the bootloader
uint8_t			g_board_bl_version;	///< bootloader version

static const uint8_t air_data_rates[NUM_DATA_RATES] = {
	2,	4,	8,	16,	19,	24,	32,	48,	64,	96,	128,	192,	250
};

radio_settings_t settings= {
	918000000UL,//uint32_t frequency;
	50000UL,//uint32_t hancnel_spacing;
	10,//uint8_t air_data_rate;
	0,//uint8_t current_channel;
	0,//uint8_t transmit_power;
	16//uint8_t preamble_length; // in nibbles
} ;



// ******************** local function prototypes ********************
// ********************* Implementation ******************************
/// Print a message and halt, largely for debug purposes
/// @param	fmt		printf-style format string and argments to be printed.
void panic(char *fmt, ...)
{
	va_list ap;

	puts("\n**PANIC**");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	puts("");

	delay_msec(1000);

	// generate a software reset TODO
	for (;;)
		;
}

/// begin transmission of a packet
/// @param length		Packet length to be transmitted; assumes
///				the data is already present in the FIFO.
/// @param timeout_ticks	The number of ticks to wait before assiming
///				that transmission has failed.
/// @return			true if packet sent successfully
///
/*
bool radio_transmit(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks)
{
	return(false);
}
*/
/// switch the radio to receive mode
/// @return			Always true.
bool radio_receiver_on(void)
{
	return(false);
}

/// reset and intiialise the radio
/// @return			True if the initialisation completed successfully.
bool radio_initialise(void)
{
	return(false);
}

/// set the nominal radio transmit/receive frequencies
/// This is the frequency of channel zero.
/// @param value		The frequency in Hz
bool radio_set_frequency( uint32_t value)
{
	return(false);
}

/// set the channel spacing used by the channel offset control
/// @param value		The channel spacing in Hz
bool radio_set_channel_spacing( uint32_t value)
{
	if (value > 2550000L)
		return false;
	///value = scale_uint32(value, 10000);
	//settings.channel_spacing = value;
	return(true);
}

/// set the channel for transmit/receive
/// @param value		The channel number to select
void radio_set_channel(uint8_t channel)
{
}

/// get the tx/rx frequency channel
/// @return			the current channel
uint8_t radio_get_channel(void)
{
	return(0);
}

/// configure the radio for a given air data rate
/// @param air_rate		The air data rate, in bits per second
///				Note that this value is rounded up to the next supported value
/// @return			True if the radio was successfully configured.
bool radio_configure( uint8_t air_rate)
{
	return(false);
}

/// configure the radio network ID
/// The network ID is programmed as header bytes, so that packets for
/// other networks can be rejected at the hardware level.
/// @param id			The network ID to be sent, and to filter on reception
void radio_set_network_id(uint16_t id)
{
}

/// fetch the signal strength recorded for the most recent preamble
/// @return			The RSSI register as reported by the radio
///				the last time a valid preamble was detected.
uint8_t radio_last_rssi(void)
{
	return(0);
}

/// fetch the current signal strength for LBT
/// @return			The RSSI register as reported by the radio
uint8_t radio_current_rssi(void)
{
	return(0);
}

/// return the air data rate
/// @return			The value passed to the last successful call
///				to radio_configure
uint8_t radio_air_rate(void)
{
	return settings.air_data_rate;
}

/// set the radio transmit power (in dBm)
/// @param power		The desired transmit power in dBm
void radio_set_transmit_power(uint8_t power)
{
	/*
	if(PwrLevel > 127) PwrLevel=127;
  ezradio_set_property(EZRADIO_PROP_GRP_ID_PA, 1u, 1,PwrLevel);
*/
}

/// set the radio transmit power (in dBm)
/// @param increment	change the radio power up (true) or down (false)
/// @param maxPower		The maximum transmit power in dBm
/// @return						The actual transmit power in dBm
uint8_t radio_change_transmit_power(bool increment, uint8_t maxPower)
{
	/*
	if(PwrLevel > 127) PwrLevel=127;
  ezradio_set_property(EZRADIO_PROP_GRP_ID_PA, 1u, 1,PwrLevel);
*/
	return(0);
}

/// get the currend transmit power (in dBm)
/// @return			The actual transmit power in dBm
///
///
uint8_t radio_get_transmit_power(void)
{
	return(0);
}
void SetTxPowerLevel(uint8_t PLevel)
{
  // READ PA_MODE and PA_PWR_LVL
  //ezradio_get_property(EZRADIO_PROP_GRP_ID_PA, 3u, 0, &ezradioReply);
  //uint8_t Pa_Mode = ezradioReply.GET_PROPERTY.DATA[0];
  //uint8_t Pa_Pwr_Lvl = ezradioReply.GET_PROPERTY.DATA[1];
  //uint8_t Pa_Bias_Clk = ezradioReply.GET_PROPERTY.DATA[2];
/*
  ezradio_set_property(
      EZRADIO_PROP_GRP_ID_PKT, 2u,
      EZRADIO_PROP_GRP_INDEX_PKT_FIELD_1_LENGTH,
      0, pktLengthConf.fieldLen.f1
      );
*/

}

/// send a MAVLink status report packet
void MAVLink_report(void)
{}

/// return temperature in degrees C
///
/// @return		temperature in degrees C, from 0 to 127
///
int16_t radio_temperature(void)
{
	return(0);
}

// maximum temperature we allow the radio to get to before
// we start limiting the duty cycle
void radio_set_diversity(bool enable)
{}
// ********************* radio_old.c **********************************
