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
#include "radio-config-2gfsk-4-25.h"
#include "radio-config-2gfsk-64-96.h"
#include "radio-config-4gfsk-250-42.h"
#include "radio-config-4gfsk-500-83.h"
#include "radio-config-2gfsk-250-159.h"
#include "rtcdriver.h"
#include "golay.h"
#include "crc.h"
#include "serial.h"

// ******************** defines and typedefs *************************
#define TX_FIFO_TIMEOUT_MS	5000L																									// max time to transmit a fifo buffer
#define RX_FIFO_TIMEOUT_MS	5000L																									// max time to transmit a fifo buffer

#define NUM_DATA_RATES 5
#define RFD900_INT_TX_POW 26           // TX power level into amp (0-127)not linear, 10dbm out, 6.8 after atten
// 2.250V @ 30dbm; 1.75V @ n = 127; 3.15V @n = 1 ; 90mV @n = 255
#define NUM_POWER_LEVELS 16
#define POWER_LEVEL_STEP 2

typedef union{
	uint8_t b[4];
	uint16_t w[2];
	uint32_t L;
} longin_t;

typedef enum {
	ModType_CW = EZRADIO_PROP_MODEM_MOD_TYPE_MOD_TYPE_ENUM_CW,
	ModType_OOK = EZRADIO_PROP_MODEM_MOD_TYPE_MOD_TYPE_ENUM_OOK,
	ModType_2FSK = EZRADIO_PROP_MODEM_MOD_TYPE_MOD_TYPE_ENUM_2FSK,
	ModType_2GFSK = EZRADIO_PROP_MODEM_MOD_TYPE_MOD_TYPE_ENUM_2GFSK,
	ModType_4FSK = EZRADIO_PROP_MODEM_MOD_TYPE_MOD_TYPE_ENUM_4FSK,
	ModType_4GFSK = EZRADIO_PROP_MODEM_MOD_TYPE_MOD_TYPE_ENUM_4GFSK,
} ModType_t;

typedef struct {
	uint32_t air_rate;
		ModType_t Modulation; // TODO remove these unused fields, relys on config header files now
	uint32_t Deviation;
	const uint8_t *CfgList;
//uint8_t  MdmDecCfg[3];
//uint8_t  MdmFltCoe[36];
} RFParams_t;
// ******************** local variables ******************************
static uint8_t radioTxPkt[MAX_PACKET_LENGTH+3];
static uint8_t radioTxCount=0;
static bool     appTxActive = false;																						/* Sign tx active state */
static bool     RxDataReady = false;
static uint16_t Rx_Tick;																												// what was the tick count when packet was received
static bool     RxDataIncoming = false;
static uint8_t radioRxPkt[MAX_PACKET_LENGTH+3];																					/* Rx packet data array */

static RTCDRV_TimerID_t RxFifoTimer;																						// Timer used to issue time elapsed for TX Fifo

static EZRADIODRV_HandleData_t appRadioInitData = EZRADIODRV_INIT_DEFAULT;							/* EZRadio driver init data and handler */
static EZRADIODRV_Handle_t appRadioHandle = &appRadioInitData;
static bool RxTimedOut = false;

// the power_levels array define 8 bit PWM values for each respective power level starting at ??dBm TODO this default table must be filled with close values
static const uint8_t power_levels[NUM_POWER_LEVELS] = { 235, 230, 224, 218, 211, 206, 201, 196, 190, 184, 178, 171, 164, 150, 136, 80 };
static ezradio_cmd_reply_t ezradioReply;
static const uint32_t FCODIV[8] = { 4, 6, 8, 12, 16, 24, 24, 24 };
static const uint32_t NPRESC[2] = { 4, 2 };
static const uint8_t Radio_Configuration_Data_Array_2G425[] = RADIO_2G425_CONFIGURATION_DATA_ARRAY;
static const uint8_t Radio_Configuration_Data_Array_2G6496[] = RADIO_2G6496_CONFIGURATION_DATA_ARRAY;
static const uint8_t Radio_Configuration_Data_Array_4G25042[] = RADIO_4G25042_CONFIGURATION_DATA_ARRAY;
static const uint8_t Radio_Configuration_Data_Array_4G50083[] = RADIO_4G50083_CONFIGURATION_DATA_ARRAY;
static const uint8_t Radio_Configuration_Data_Array_2G250159[] = RADIO_2G250159_CONFIGURATION_DATA_ARRAY;

static const RFParams_t RFParams[NUM_DATA_RATES] =
{
	{	4  ,ModType_2GFSK, 25000,Radio_Configuration_Data_Array_2G425  },
	{	64 ,ModType_2GFSK, 96000,Radio_Configuration_Data_Array_2G6496 },
	{	250,ModType_2GFSK,159000,Radio_Configuration_Data_Array_2G250159},
	{	500,ModType_4GFSK,159000,Radio_Configuration_Data_Array_4G25042},
	{1000,ModType_4GFSK,159000,Radio_Configuration_Data_Array_4G50083},
};

static uint8_t lastRSSI=0;
static uint8_t netid[2];

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
enum BoardFrequency	g_board_frequency;	///< board RF frequency from the cal table
uint8_t			g_board_bl_version;	///< bootloader version

radio_settings_t settings= {
	915000000UL,//uint32_t frequency;
	250000UL,//uint32_t hancnel_spacing;
	64,//uint8_t air_data_rate;
	0,//uint8_t current_channel;
	0,//uint8_t transmit_power;
	16,//uint8_t preamble_length; // in nibbles
	0// networkID
} ;

// ******************** local function prototypes ********************
static void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status ,uint16_t IRQ_ticks);
static void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status ,uint16_t IRQ_ticks);
static bool radio_transmit_simple(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks,uint16_t *TxTick);
static bool radio_transmit_golay(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks,uint16_t *TxTick);

// ********************* Implementation ******************************

void radio_daemon(void)
{
	if(true == RxTimedOut)
	{
		RxDataIncoming = false;																											// clear incoming, packet arrived
		RxTimedOut = false;
		ezradioResetTRxFifo();
		ezradio_get_int_status(0u, 0u, 0u, NULL);
		ezradioStartRx(appRadioHandle);
	}

	ezradioPluginManager( appRadioHandle );
}

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

	for (;;)																																			// watchdog will catch after 1 sec
		;
}

static void RxFifoTimeout(RTCDRV_TimerID_t id, void *RadioHandle )										  //RxFifoTimer handle rx fifo not complete interrupt failed
{
	(void) id;
  if((true == RxDataIncoming)&&(false == RxTimedOut))
  {
  	RxTimedOut = true;
  }
}


/// begin transmission of a packet
/// @param length		Packet length to be transmitted; assumes
///				the data is already present in the FIFO.
/// @param timeout_ticks	The number of ticks to wait before assiming
///				that transmission has failed.
/// @return			true if packet sent successfully
///
bool radio_transmit(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks,uint16_t *TxTick)
{
	bool ret;
	if (!feature_golay)
	{
		ret = radio_transmit_simple(length, buf, timeout_ticks,TxTick);
	}
	else
	{
		ret = radio_transmit_golay(length, buf, timeout_ticks,TxTick);
	}
	return(ret);
}

static bool radio_transmit_simple(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks,uint16_t *TxTick)
{
	static EZRADIODRV_PacketLengthConfig_t pktLength =
  {ezradiodrvTransmitLenghtCustomFieldLen,10,{2,1,9,0,0}} ;
	bool Res = false;

	if((!appTxActive)&&(length <= (sizeof(radioTxPkt)-3)))
	{
		pktLength.pktLen = length+3;
#if 1
		appRadioHandle->packetTx.lenConfig.pktLen = pktLength.pktLen;
		appRadioHandle->packetTx.lenConfig.fieldLen.f1 = pktLength.fieldLen.f1 = pktLength.pktLen;
		pktLength.fieldLen.f2  = 0;
		pktLength.fieldLen.f3  = 0;
#else
		pktLength.fieldLen.f3  = length;
#endif
		radioTxPkt[0] = ((settings.networkID>>8)&0xFF);
		radioTxPkt[1] = ((settings.networkID)&0xFF);
		radioTxPkt[2] = length;
		if(&radioTxPkt[3] != buf)
		{
			memcpy(&radioTxPkt[3],buf,length);
		}
  	uint16_t tickStart = timer2_tick();
  	appTxActive = (Res = (ECODE_EMDRV_EZRADIODRV_OK == ezradioStartTransmitCustom(appRadioHandle, pktLength, radioTxPkt,TxTick)));
  	if (appTxActive)
    {
  		// this nasty bit of code is here because the access time for fifo length and tx write and int status is so slow that we can't write the buffer in time
  		// using the tx thresholds
  		//uint8_t maxbuff=0;
  		static ezradio_cmd_reply_t radioReplyData;
  		radioTxCount = (pktLength.pktLen >= EZRADIO_FIFO_SIZE)?(EZRADIO_FIFO_SIZE):(pktLength.pktLen);
    	while((radioTxCount<pktLength.pktLen)&&((uint16_t)(timer2_tick()-tickStart) <  timeout_ticks))
    	{
    	  static uint8_t insertlen;
        ezradio_fifo_info_fast_read(&radioReplyData);
        insertlen = (pktLength.pktLen-radioTxCount);
        if (insertlen > radioReplyData.FIFO_INFO.TX_FIFO_SPACE)
        {
        	insertlen = radioReplyData.FIFO_INFO.TX_FIFO_SPACE;
        }
        if(insertlen != 0)
        {
        	ezradio_write_tx_fifo(insertlen, &radioTxPkt[radioTxCount]);
        	radioTxCount += insertlen;
        }
        //if(maxbuff  < radioReplyData.FIFO_INFO.TX_FIFO_SPACE) maxbuff = radioReplyData.FIFO_INFO.TX_FIFO_SPACE;
    	}
    	if(radioTxCount != pktLength.pktLen)
    	{
    		Res= false;
    	}
    	else
    	{
				// wait for tx complete
				bool done =false;
				while(!done&&((uint16_t)(timer2_tick()-tickStart) <  timeout_ticks))
				{
					ezradio_get_int_status(0,0,0,&radioReplyData);
					if ( radioReplyData.GET_INT_STATUS.CHIP_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_CHIP_PEND_FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND_BIT )
					{
						ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_TX_BIT, NULL); // TODO use frr registers for this
						done = true;
						Res = false;	// we failed :(
					}
					else if( radioReplyData.GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT )
					{
						done = true;
					}
				}
    	}
    }
	}
	return(Res);
}

static bool radio_transmit_golay(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks,uint16_t *TxTick)
{
	uint16_t crc;
	uint8_t gin[3];
	uint8_t elen, rlen;

	if (length > (sizeof(radioTxPkt)/2)-9)
	{
		debug("golay packet size %u\n", (unsigned)length);
		panic("oversized golay packet");
	}

	rlen = ((length+2)/3)*3;																											// rounded length
	elen = (rlen+6)*2;																														// encoded length
	gin[0] = netid[0];																														// start of packet is network ID and packet length
	gin[1] = netid[1];
	gin[2] = length;

	golay_encode(3, gin, &radioTxPkt[3]);																						// golay encode the header
	crc = crc16(length, buf);																											// next add a CRC, we round to 3 bytes for simplicity, adding
	gin[0] = crc&0xFF;																														// another copy of the length in the spare byte
	gin[1] = crc>>8;
	gin[2] = length;
	golay_encode(3, gin, &radioTxPkt[9]);																				// golay encode the CRC
	golay_encode(rlen, buf, &radioTxPkt[15]);																		// encode the rest of the payload
	return radio_transmit_simple(elen, &radioTxPkt[3], timeout_ticks,TxTick);
}

/// receives a packet from the radio
/// @param len			Pointer to storage for the length of the packet
/// @param buf			Pointer to storage for the packet
/// @return			True if a packet was received
bool radio_receive_packet(uint16_t *length, uint8_t *  buf,uint16_t * Tick)
{
  if(RxDataReady)
  {
  	uint16_t receive_packet_length = radioRxPkt[2];
		if(receive_packet_length > MAX_PACKET_LENGTH)
		{
			receive_packet_length = MAX_PACKET_LENGTH;
		}
		*Tick = Rx_Tick;
		RxDataReady = false;
  	if (!feature_golay)
  	{
			*length = receive_packet_length;
			memcpy(buf,&radioRxPkt[3],*length);
			//radio_receiver_on();
			return(true);
  	}
  	else			// use golay en/decoding
  	{
    	uint8_t gout[3];
    	uint8_t errcount = 0;
    	uint16_t crc1, crc2;
    	uint16_t elen;
  		// decode it in the callers buffer. This relies on the
  		// in-place decode properties of the golay code. Decoding in
  		// this way allows us to overlap decoding with the next receive
  		memcpy(buf, &radioRxPkt[3], receive_packet_length);
  		// enable the receiver for the next packet. This also
  		// enables the EX0 interrupt
  		elen = receive_packet_length;
  		radio_receiver_on();

			if (elen < 12 || (elen%6) != 0)
			{
				// not a valid length
				debug("rx len invalid %u\n", (unsigned)elen);
				goto failed;
			}

			// decode the header
			errcount = golay_decode(6, buf, gout);
			if (gout[0] != netid[0] || gout[1] != netid[1])
			{
				// its not for our network ID
				debug("netid %x %x\n",(unsigned)gout[0],(unsigned)gout[1]);
				goto failed;
			}
			if (6*((gout[2]+2)/3+2) != elen)
			{
				debug("rx len mismatch1 %u %u\n",(unsigned)gout[2],(unsigned)elen);
				goto failed;
			}
			// decode the CRC
			errcount += golay_decode(6, &buf[6], gout);
			crc1 = gout[0] | (((uint16_t)gout[1])<<8);
			if (elen != 12)
			{
				errcount += golay_decode(elen-12, &buf[12], buf);
			}
			*length = gout[2];
			crc2 = crc16(*length, buf);
			if (crc1 != crc2)
			{
				debug("crc1=%x crc2=%x len=%u [%x %x]\n",(unsigned)crc1,(unsigned)crc2,
  		       (unsigned)*length,(unsigned)buf[0],(unsigned)buf[1]);
				goto failed;
			}
			if (errcount != 0)
			{
				if ((uint16_t)(0xFFFF - errcount) > errors.corrected_errors)
				{
					errors.corrected_errors += errcount;
				} else
				{
					errors.corrected_errors = 0xFFFF;
				}
				if (errors.corrected_packets != 0xFFFF) {
					errors.corrected_packets++;
				}
			}
			return true;
  	}
  failed:
  	if (errors.rx_errors != 0xFFFF)
  	{
  		errors.rx_errors++;
  	}
  	return false;
  }
	return(false);
}


/// test whether the radio has detected a packet preamble
/// @return			True if a preamble has been detected
bool radio_preamble_detected(void)
{
  return(RxDataIncoming);
}
/// check if a packet is coming in
/// @return			true if a packet is being received
bool radio_receive_in_progress(void)
{
	return(RxDataIncoming);
}

/**************************************************************************//**
 * @brief  Packet received callback of the application.
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status ,uint16_t IRQ_ticks)
{
  if ( ECODE_EMDRV_EZRADIODRV_PACKET_RX == status)
  {
  	RTCDRV_StopTimer(RxFifoTimer);
    RxDataReady = true;
  	RxDataIncoming = false;																											// clear incoming, packet arrived
  	Rx_Tick = IRQ_ticks;
  	//putChar ('X');
  }
  else if (ECODE_EMDRV_EZRADIODRV_PREAMBLE_DETECT == status)
  {
  	static 	ezradio_cmd_reply_t ezradioReply;
  	extern uint8_t lastRSSI;
  	if(!RxDataIncoming)
  	{
    	//putChar ('P');
  		RxDataIncoming = true;
  		ezradio_get_modem_status(0x00,&ezradioReply);
  		lastRSSI  = ezradioReply.GET_MODEM_STATUS.CURR_RSSI;
    	RTCDRV_StopTimer(RxFifoTimer);
    	RTCDRV_StartTimer(RxFifoTimer,rtcdrvTimerTypeOneshot, RX_FIFO_TIMEOUT_MS,RxFifoTimeout,appRadioHandle);
  	}
  }
}

/**************************************************************************//**
 * @brief  Packet received with CRC error callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status ,uint16_t IRQ_ticks)
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
  	RxDataIncoming = false;
  	//printf("-->Pkt  RX: CRC Error\n");
    /* Change to RX state */
    ezradioStartRx( handle );
  }
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
	appTxActive = false;
	RxDataIncoming = false;																											// clear incoming, packet arrived
	RxTimedOut = false;
	//ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);
	ezradioResetTRxFifo();
#if 1
	ezradio_get_int_status_fast_clear();
#else
	ezradio_get_int_status(0u, 0u, 0u, NULL);
#endif
	ezradioStartRx(appRadioHandle);
	//putChar ('O');
	return(true);																																	// the radio layer should take care of this anyway
}

/// reset and intiialise the radio
/// @return			True if the initialisation completed successfully.
bool radio_initialise(uint16_t air_rate)
{
	InitPWM();
  RTCDRV_Init();																																/* Set RTC to generate interrupt 250ms. */
  if (ECODE_EMDRV_RTCDRV_OK !=
      RTCDRV_AllocateTimer( &RxFifoTimer) )
  {
    while (1);
  }

  appRadioInitData.packetRx.userCallback = &appPacketReceivedCallback;					// Configure packet received buffer and callback.
  appRadioInitData.packetRx.pktBuf = radioRxPkt;																// set the dest buffer
  appRadioInitData.packetRx.pktBufLen = sizeof(radioRxPkt);											// set the dest buffer size
  appRadioInitData.packetCrcError.userCallback = &appPacketCrcErrorCallback;		// Configure packet received with CRC error callback.
  uint8_t RateIdx;
	const RFParams_t *Params;
	while ((RateIdx < NUM_DATA_RATES) && (air_rate != RFParams[RateIdx].air_rate))
	{
		RateIdx++;
	}
	if (NUM_DATA_RATES <= RateIdx)
		return (false);
	Params = &RFParams[RateIdx];
	if(NULL == Params->CfgList)
		return (false);
	ezradioInit(appRadioHandle ,Params->CfgList);
	settings.air_data_rate = Params->air_rate;
  ezradioResetTRxFifo();																												// Reset radio fifos and start reception.
  ezradioStartRx( appRadioHandle );

	return(true);
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
void radio_set_channel(uint8_t channel, bool Update)
{
	//return;
	if((channel != appRadioHandle->packetRx.channel)||
		 (channel !=  appRadioHandle->packetTx.channel))
	{
		appRadioHandle->packetRx.channel = channel;
	  appRadioHandle->packetTx.channel = channel;
	  // when tx or rx called this will be updated
	  // unfortunately the old code required updating channel variable in radio
	  // however access to this variable is not available in the new radio
	  // when in rx mode will need to manually change channel

	  if(Update)
	  {
	  	ezradioStartRx(appRadioHandle);
		}
	}
}

/// get the tx/rx frequency channel
/// @return			the current channel
uint8_t radio_get_channel(void)
{
	return(appRadioHandle->packetRx.channel);
}

bool radio_RateValid(uint16_t air_rate)
{
	uint8_t RateIdx = 0;
	while ((RateIdx < NUM_DATA_RATES) && (air_rate != RFParams[RateIdx].air_rate))
	{
		RateIdx++;
	}
	if (NUM_DATA_RATES <= RateIdx)
		return (false);
	return(true);
}
/// configure the radio for a given air data rate
/// @param air_rate		The air data rate, in bits per second (assume they mean Kbits/s, hmmm this needs to change for this modem TODO)
///				Note that this value is rounded up to the next supported value
/// @return			True if the radio was successfully configured.
bool radio_configure( void)
{
	if (feature_golay) 																														// if golay crc and network id done after packet rx
	{
		ezradio_set_property(EZRADIO_PROP_GRP_ID_MATCH,															// turn all matching off value,mask,ctrl x 4
				12u,EZRADIO_PROP_GRP_INDEX_MATCH_VALUE_1,0,0,0,0,0,0,0,0,0,0,0,0);
		ezradio_set_property(EZRADIO_PROP_GRP_ID_PKT, 1u,														// only turn on crc calc, don't enable checking
				EZRADIO_PROP_GRP_INDEX_PKT_RX_FIELD_3_CRC_CONFIG,EZRADIO_PROP_PKT_RX_FIELD_3_CRC_CONFIG_CRC_ENABLE_BIT);
	}
	return(true);
}

/// configure the radio network ID
/// The network ID is programmed as header bytes, so that packets for
/// other networks can be rejected at the hardware level.
/// @param id			The network ID to be sent, and to filter on reception
void radio_set_network_id(uint16_t id)
{
	netid[0] = id&0xFF;
	netid[1] = id>>8;
	if (!feature_golay)
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
	return(ezradioReply.GET_MODEM_STATUS.CURR_RSSI);															// just return it, last is always read during packet rx
}

/// return the air data rate
/// @return			The value passed to the last successful call
///				to radio_configure
uint16_t radio_air_rate(void)
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
	int32_t TempDegC;
	ezradio_get_adc_reading(EZRADIO_CMD_GET_ADC_READING_ARG_ADC_EN_TEMPERATURE_EN_BIT,
			0xA5,&ezradioReply );
	//TEMP(degC) = (899/4096)*TEMP_ADC - 293
	TempDegC = ezradioReply.GET_ADC_READING.TEMP_ADC;
	TempDegC *= 899L;
	TempDegC /= 4096L;
	TempDegC -= 293;
	return(TempDegC);
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
