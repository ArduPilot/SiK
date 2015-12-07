/*
 * radio_old.c
 *
 *  Created on: 06/11/2015
 *      Author: kentm
 */

#include <stdint.h>
#include <stdbool.h>
#include "radio_old.h"
#include "ezradio_cmd.h"
#include "ezradio_api_lib.h"
#include "ezradio_prop.h"
#include "radio-config-wds-gen.h"
#include "PWM.h"
#include "timer.h"
#include "app-config.h"
#include "ezradio_cmd.h"
#include "ezradio_api_lib.h"
#include "ezradio_plugin_manager.h"
#include "ezradio_prop.h"
#include "ezradio_api_lib_add.h"
#include "parameters.h"

// ******************** defines and typedefs *************************
#define NUM_DATA_RATES 13
#define RFD900_INT_TX_POW 64           // TX power level into amp (0-127)not linear, must set by testing)
#define NUM_POWER_LEVELS 16
#define POWER_LEVEL_STEP 2

typedef union{
	uint8_t b[4];
	uint16_t w[2];
	uint32_t L;
} longin_t;
// ******************** local variables ******************************
// the power_levels array define 8 bit PWM values for each respective power level starting at ??dBm TODO this default table must be filled with close values
static const uint8_t power_levels[NUM_POWER_LEVELS] = {80,136,150,164,171,178,184,190,196,201,206,211,218,224,230,235};
static 	ezradio_cmd_reply_t ezradioReply;
static const uint32_t FCODIV[8] = {4,6,8,12,16,24,24,24};
static const uint32_t NPRESC[2] = {4,2};

// ******************** global variables *****************************
bool feature_golay = false;
bool feature_opportunistic_resend = false;
uint8_t feature_mavlink_framing = false;
bool feature_rtscts = true;
statistics_t statistics, remote_statistics;
error_counts_t errors={0};

// Board infop
const char 		g_version_string[]= "blah";	///< printable version string
const char 		g_banner_string[] ="yoyo";	///< printable startup banner string
enum BoardFrequency	g_board_frequency;	///< board RF frequency from the bootloader
uint8_t			g_board_bl_version;	///< bootloader version

//static const uint8_t air_data_rates[NUM_DATA_RATES] = {
//	2,	4,	8,	16,	19,	24,	32,	48,	64,	96,	128,	192,	250
//};

radio_settings_t settings= {
	915000000UL,//uint32_t frequency;
	250000UL,//uint32_t hancnel_spacing;
	64,//uint8_t air_data_rate;
	0,//uint8_t current_channel;
	0,//uint8_t transmit_power;
	16,//uint8_t preamble_length; // in nibbles
	0// networkID
} ;

extern EZRADIODRV_Handle_t appRadioHandle;
uint8_t lastRSSI=0;
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
//TODO reset stuff when this called
#if 0
	packet_received = 0;
	receive_packet_length = 0;
	preamble_detected = 0;
	partial_packet_length = 0;

	// enable receive interrupts
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, RADIO_RX_INTERRUPTS);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENPREAVAL);

	clear_status_registers();
	radio_clear_transmit_fifo();
	radio_clear_receive_fifo();

	// put the radio in receive mode
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, EZRADIOPRO_RXON | EZRADIOPRO_XTON);

	// enable receive interrupt
	EX0 = 1;

	return true;
#endif
	return(true);																																	// the radio layer should take care of this anyway
}

/// reset and intiialise the radio
/// @return			True if the initialisation completed successfully.
bool radio_initialise(void)
{
	InitPWM();
	// TODO , add radio startup code here
	return(false);
}

/// set the nominal radio transmit/receive frequencies
/// This is the frequency of channel zero.
/// @param value		The frequency in Hz
bool radio_set_frequency( uint32_t value)
{
	if (value < 240000000UL || value > 935000000UL) {
		return false;
	}
	settings.frequency = value;
	// RF FreqHz = (FCInte + (FCFrac/2<<19))*(NPRESC*Frq_xo/outdiv)
	// bit 19 of FCRac always set, so  1 =< (FCFrac/2<<19) < 2 AND FCInte -=1;
	ezradio_get_property(EZRADIO_PROP_GRP_ID_MODEM, 1u,EZRADIO_PROP_GRP_INDEX_MODEM_CLKGEN_BAND, &ezradioReply);
	uint8_t SY_SEL = (EZRADIO_PROP_MODEM_CLKGEN_BAND_SY_SEL_MASK&ezradioReply.GET_PROPERTY.DATA[0])>>EZRADIO_PROP_MODEM_CLKGEN_BAND_SY_SEL_LSB;
	uint8_t BAND   = (EZRADIO_PROP_MODEM_CLKGEN_BAND_BAND_MASK&ezradioReply.GET_PROPERTY.DATA[0])>>EZRADIO_PROP_MODEM_CLKGEN_BAND_BAND_LSB;
  uint32_t NPresc = NPRESC[SY_SEL];
	uint32_t outdiv = FCODIV[BAND];
	uint32_t Scale = (NPresc*RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ)/outdiv;
	longin_t Integer,Frac;
	uint64_t temp;																																// 32 bit overflowing, use 64
	temp = (value%Scale);
	temp <<= 19u;
	temp /= Scale;
	Frac.L = temp;
	Frac.L |= (1UL<<19);
	Integer.L = value/Scale;
	Integer.L -- ;
	if(Integer.L > 0x7FUL)return(false);
	if(Frac.L > 0xFFFFFUL)return(false);
	ezradio_set_property(EZRADIO_PROP_GRP_ID_FREQ_CONTROL, 4u,
			EZRADIO_PROP_GRP_INDEX_FREQ_CONTROL_INTE,
			Integer.b[0],Frac.b[2],Frac.b[1],Frac.b[0]);
	return(true);
}


/// set the channel spacing used by the channel offset control
/// @param value		The channel spacing in Hz
bool radio_set_channel_spacing( uint32_t value)
{
	longin_t StepSize;
	// this is called from tdm init on startup to set channel spacing
	if (value > 2550000L)
		return false;
	//Freq_CTRL_CHAN_STEP_SIZE = (2^19 x outdiv x DesiredStepHz )/(NPresc x freq_xo)
	// NPresc read from MODEM_CLKGEN_BAND:SY_SEL.
	// outdiv read from MODEM_CLKGEN_BAND:BAND.
	// freq_xo read from crstal xo
	ezradio_get_property(EZRADIO_PROP_GRP_ID_MODEM, 1u,EZRADIO_PROP_GRP_INDEX_MODEM_CLKGEN_BAND, &ezradioReply);
	uint8_t SY_SEL = (EZRADIO_PROP_MODEM_CLKGEN_BAND_SY_SEL_MASK&ezradioReply.GET_PROPERTY.DATA[0])>>EZRADIO_PROP_MODEM_CLKGEN_BAND_SY_SEL_LSB;
	uint8_t BAND   = (EZRADIO_PROP_MODEM_CLKGEN_BAND_BAND_MASK&ezradioReply.GET_PROPERTY.DATA[0])>>EZRADIO_PROP_MODEM_CLKGEN_BAND_BAND_LSB;
  uint32_t NPresc = NPRESC[SY_SEL];
	uint32_t outdiv = FCODIV[BAND];

	StepSize.L = ((1UL<<19u)*outdiv*(value/1000));																	// was overflowing 32bits, so dive top and bottom by 1000 first
	StepSize.L /= (NPresc*(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ/1000));
	ezradio_set_property(EZRADIO_PROP_GRP_ID_FREQ_CONTROL, 2u,
			EZRADIO_PROP_GRP_INDEX_FREQ_CONTROL_CHANNEL_STEP_SIZE,
			StepSize.b[1],StepSize.b[0]);
	settings.channel_spacing = StepSize.L;
	return(true);
}

/// set the channel for transmit/receive
/// @param value		The channel number to select
void radio_set_channel(uint8_t channel)
{
	return;
	if((channel != appRadioHandle->packetRx.channel)||
		 (channel !=  appRadioHandle->packetTx.channel))
	{
		appRadioHandle->packetRx.channel = channel;
	  appRadioHandle->packetTx.channel = channel;
	  // restart mode if necessary
	  ezradio_request_device_state(&ezradioReply);
	  if(channel != ezradioReply.REQUEST_DEVICE_STATE.CURRENT_CHANNEL)
	  {
	  	if(EZRADIO_CMD_REQUEST_DEVICE_STATE_REP_CURR_STATE_MAIN_STATE_ENUM_RX ==
	  			ezradioReply.REQUEST_DEVICE_STATE.CURR_STATE)
	  	{
	  	  ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);
				ezradioStartRx(appRadioHandle);
				// TODO clear radio fifos and state variables
				 // TODO might be better to use RX hop for this in case we interrupt a receive process
	  	}
	  }
	}
}

/// get the tx/rx frequency channel
/// @return			the current channel
uint8_t radio_get_channel(void)
{
	return(appRadioHandle->packetRx.channel);
}

/// configure the radio for a given air data rate
/// @param air_rate		The air data rate, in bits per second (assume they mean Kbits/s, hmmm this needs to change for this modem TODO)
///				Note that this value is rounded up to the next supported value
/// @return			True if the radio was successfully configured.
bool radio_configure( uint16_t air_rate)
{
	static const uint32_t TXOSR[3] = {10,40,20};
	if(air_rate > 1024)return(false);
	settings.air_data_rate = air_rate;
	// TX_DATA_RATE = (NCO_CLK_FREQ/TXOSR)
	// NCO_CLK_FREQ = (MODEM_DATA_RATE*FxtalHz/NCO_MODE)
  // MODEM_DATA_RATE = (NCO_CLK_FREQ*NCO_MODE)/FxtalHz
  // NCO_CLK_FREQ = TX_DATA_RATE*TXOSR
	ezradio_get_property(EZRADIO_PROP_GRP_ID_MODEM, 7u,EZRADIO_PROP_GRP_INDEX_MODEM_DATA_RATE, &ezradioReply);
	longin_t ModemRate = {{0}};
	longin_t NOCMode = {{0}};
	uint8_t TxOsr;
	ModemRate.b[2] = ezradioReply.GET_PROPERTY.DATA[0];
	ModemRate.b[1] = ezradioReply.GET_PROPERTY.DATA[1];
	ModemRate.b[0] = ezradioReply.GET_PROPERTY.DATA[2];
	NOCMode.b[3] = (ezradioReply.GET_PROPERTY.DATA[3]&EZRADIO_PROP_MODEM_TX_NCO_MODE_NCOMOD_25_24_MASK)>>EZRADIO_PROP_MODEM_TX_NCO_MODE_NCOMOD_25_24_LSB;
	NOCMode.b[2] = ezradioReply.GET_PROPERTY.DATA[4];
	NOCMode.b[1] = ezradioReply.GET_PROPERTY.DATA[5];
	NOCMode.b[0] = ezradioReply.GET_PROPERTY.DATA[6];
	TxOsr = (ezradioReply.GET_PROPERTY.DATA[3]>>EZRADIO_PROP_MODEM_TX_NCO_MODE_TXOSR_LSB)&EZRADIO_PROP_MODEM_TX_NCO_MODE_TXOSR_MAX;
	uint32_t NCO_CLK_FREQ = air_rate*1000UL*TXOSR[TxOsr];
	uint64_t temp = ((uint64_t)NCO_CLK_FREQ*(uint64_t)NOCMode.L);
	temp /= RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ;
	ModemRate.L = temp;
	ezradio_set_property(EZRADIO_PROP_GRP_ID_MODEM, 3u,EZRADIO_PROP_GRP_INDEX_MODEM_DATA_RATE,
			ModemRate.b[2],ModemRate.b[1],ModemRate.b[0]);

	// TODO we will also need to set deviation and rx bandwidth and modulation type 2G or 4G
	// modulation type MODEM_MOD_TYPEGroup: 0x20 Index: 0x00 MOD_TYPE
	// deviation MODEM_FREQ_DEV Group: 0x20 Indexes: 0x0a ... 0x0c
	// MODEM_FREQ_DEV = (2^19*outdiv*desired_Dev_Hz)/(NPRESC*freq_xo)
	//For 2(G)FSK mode, the specified value is the peak deviation. For 4(G)FSK mode (if supported), the specified value is the inner deviation (i.e., between channel center frequency and the nearest symbol deviation level).
	// rx bandwidth
	// MODEM_DECIMATION_CFG1 MODEM_DECIMATION_CFG0 MODEM_CHFLT_RX1_CHFLT_COE MODEM_CHFLT_RX2_CHFLT_COE
	return(true);
}

/// configure the radio network ID
/// The network ID is programmed as header bytes, so that packets for
/// other networks can be rejected at the hardware level.
/// @param id			The network ID to be sent, and to filter on reception
void radio_set_network_id(uint16_t id)
{
	longin_t val;
	val.L = id;
	settings.networkID = id;
	// packet format is [preamble(16)][sync(2)][id(2)][length(1)][data(N)][CRC]
	//#define RF_SET_PROPERTY_MATCH_VALUE_3 0x11, 0x30, 0x01, 0x06, 0x55
	//#define RF_SET_PROPERTY_MATCH_VALUE_4 0x11, 0x30, 0x01, 0x09, 0xAA
	ezradio_set_property(EZRADIO_PROP_GRP_ID_MATCH, 1u,
			EZRADIO_PROP_GRP_INDEX_MATCH_VALUE_3,val.b[1]);
	ezradio_set_property(EZRADIO_PROP_GRP_ID_MATCH, 1u,
			EZRADIO_PROP_GRP_INDEX_MATCH_VALUE_4,val.b[0]);
}

/// fetch the signal strength recorded for the most recent preamble
/// @return			The RSSI register as reported by the radio
///				the last time a valid preamble was detected.
uint8_t radio_last_rssi(void)
{
	return(lastRSSI);
}

/// fetch the current signal strength for LBT
/// @return			The RSSI register as reported by the radio
uint8_t radio_current_rssi(void)
{
	ezradio_get_modem_status(0x00,&ezradioReply);
	lastRSSI = ezradioReply.GET_MODEM_STATUS.CURR_RSSI;
	return(lastRSSI);
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

	uint8_t i;
	ezradio_set_property(EZRADIO_PROP_GRP_ID_PA, 1u,
			EZRADIO_PROP_GRP_INDEX_PA_PWR_LVL,RFD900_INT_TX_POW);
	i = calibration_get(power);
	if (i != 0xFF)
	{
		SetPwmDuty(i);
		settings.transmit_power = power;
	}
	else
	{
		i = power / POWER_LEVEL_STEP;
		SetPwmDuty(power_levels[i]);
		settings.transmit_power = i * POWER_LEVEL_STEP;
	}
}

/// set the radio transmit power (in dBm)
/// @param increment	change the radio power up (true) or down (false)
/// @param maxPower		The maximum transmit power in dBm
/// @return						The actual transmit power in dBm
uint8_t radio_change_transmit_power(bool increment, uint8_t maxPower)
{
	uint8_t power = settings.transmit_power;
	// set the chip output power to +10dBm (? or whatever set level is, not linear, must set by testing)
	if (increment) {
		power += 2;
		if(power > maxPower) {
			return settings.transmit_power;
		}
	}
	else if(power != 0)
	{
		power -= 2;
	}
	radio_set_transmit_power(power);
	return settings.transmit_power;
}

/// get the current transmit power (in dBm)
/// @return			The actual transmit power in dBm
uint8_t radio_get_transmit_power(void)
{
	return settings.transmit_power;
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
	ezradio_get_adc_reading(EZRADIO_CMD_GET_ADC_READING_ARG_ADC_EN_TEMPERATURE_EN_BIT,
			0xA5,&ezradioReply );
	return(ezradioReply.GET_ADC_READING.TEMP_ADC);
}
// maximum temperature we allow the radio to get to before
// we start limiting the duty cycle

void radio_set_diversity(bool enable)
{
	ezradio_get_property(EZRADIO_PROP_GRP_ID_MODEM, 1u,EZRADIO_PROP_GRP_INDEX_MODEM_ANT_DIV_CONTROL, &ezradioReply);
	ezradioReply.GET_PROPERTY.DATA[0] &= ~EZRADIO_PROP_MODEM_ANT_DIV_CONTROL_ANTDIV_MASK;
	uint8_t Mask = (enable)?(EZRADIO_PROP_MODEM_ANT_DIV_CONTROL_ANTDIV_ENUM_AUTO):
			(EZRADIO_PROP_MODEM_ANT_DIV_CONTROL_ANTDIV_ENUM_FIXED);
	Mask <<= EZRADIO_PROP_MODEM_ANT_DIV_CONTROL_ANTDIV_LSB;
	ezradioReply.GET_PROPERTY.DATA[0] |= Mask;
	ezradio_set_property(EZRADIO_PROP_GRP_ID_MODEM, 1u,
			EZRADIO_PROP_GRP_INDEX_MODEM_ANT_DIV_CONTROL,ezradioReply.GET_PROPERTY.DATA[0]);
}
// ********************* radio_old.c **********************************
