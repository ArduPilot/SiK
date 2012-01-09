//=============================================================================
// rtPhy.c
//=============================================================================
// Copyright 2010 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// C File Description:
//
// Target:
//    Any Silicon Labs C8051 MCU.
//
// IDE:
//    Silicon Laboratories IDE   version 3.3
//
// Tool Chains:
//    Keil
//       c51.exe     version 8.0.8
//       bl51.exe    version 6.0.5
//    SDCC
//       sdcc.exe    version 2.8.0
//       aslink.exe  version 1.75
//
// Project Name:
//    MCM Code Exmaples
//
// Beta Release 0.1
//    - TBD
//
// This software must be used in accordance with the End User License Agreement.
//
//=============================================================================
//-----------------------------------------------------------------------------
// Includes
//
// These includes must be in a specific order. Dependencies listed in comments.
//-----------------------------------------------------------------------------
#include "board.h"
#include "rtPhy.h"
#include "radio.h"
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
__bit RxPacketReceived;
SEGMENT_VARIABLE (RxPacketLength, U8, BUFFER_MSPACE);
SEGMENT_VARIABLE (RxErrors, U8, BUFFER_MSPACE);
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
SEGMENT_VARIABLE (RxIntBuffer[64], U8, BUFFER_MSPACE);
static uint8_t RxHeader;

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
U8    RxIntPhyRead (U8);
void  RxIntPhyWrite (U8, U8);
void  RxIntphyReadFIFO (U8, VARIABLE_SEGMENT_POINTER(buffer, U8, BUFFER_MSPACE));
//=============================================================================
//
// API Functions
//
//=============================================================================
static rtPhySettingsStruct SEG_XDATA rtPhySettings;
static __bit PhyInitialized = 0;

//=============================================================================
// local functions
//=============================================================================
U8    InitSoftwareReset(void);

void  SetTRxFrequency (U32);
void  SetTRxChannelSpacing  (U32);

//-----------------------------------------------------------------------------
// Function Name
//    rtPhyInit()
//
// Parameters   : none
// Return Value : status
//
// After a pin reset or other reset the state of the Radio is unknown.
// The MCU will wait for full POR duration, then figure out if the
// Radio needs a software reset.
//
//-----------------------------------------------------------------------------
PHY_STATUS rtPhyInit(void)
{
   U8 status;

   SDN = 0;

   delay_msec(25);

   status = phyRead(EZRADIOPRO_DEVICE_VERSION); // check version
   if(status == 0xFF)
      return PHY_STATUS_ERROR_SPI;
   else if (status == 0x00)
      return  PHY_STATUS_ERROR_SPI;
   else if (status < MIN_RADIO_VERSION)
      return PHY_STATUS_ERROR_UNSUPPORTED_RADIO;

   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   if((status & EZRADIOPRO_IPOR)==0)
   {
      // radio needs a software reset
      return InitSoftwareReset();
   }
   else if((status & EZRADIOPRO_ICHIPRDY)==0)
   {
      // enable Chip read only
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENCHIPRDY);

      // wait on IRQ with 2 MS timeout
      delay_set(2);
      while(IRQ)
      {
         if(delay_expired())
            return PHY_STATUS_ERROR_RADIO_XTAL;
      }
   }
   return PHY_STATUS_SUCCESS; // success
}


//-----------------------------------------------------------------------------
// Function Name
//    RadioInitSWReset()
//
// Parameters   : none
// Return Value : status
//
// This function uses a software reset to reset the radio. This can be used
// to reset the radio when a radio POR has not occured.
// A T0 interrupt timeout is used to minimize start-up time.
//-----------------------------------------------------------------------------
U8 InitSoftwareReset(void)
{
   U8 status;

   // Clear interrupt enable and interrupt flag bits
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   // SWReset
   phyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, (EZRADIOPRO_SWRES|EZRADIOPRO_XTON));

   // wait on IRQ with 2 MS timeout
   delay_set(2);
   while(IRQ)
   {
      if(delay_expired())
         return PHY_STATUS_ERROR_NO_IRQ;
   }

   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   if((status & EZRADIOPRO_ICHIPRDY)==0)
   {
      // enable Chip read only
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENCHIPRDY);

      // wait on IRQ with 2 MS timeout
      delay_set(2);
      while(IRQ)
      {
         if(delay_expired())
            return PHY_STATUS_ERROR_RADIO_XTAL;
      }
   }

   return PHY_STATUS_SUCCESS; // success
}


//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
PHY_STATUS rtPhyIdle(void)
{
   U8 status;

   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   if((status & EZRADIOPRO_ICHIPRDY)==EZRADIOPRO_ICHIPRDY)
      return PHY_STATUS_SUCCESS;
   else
   {
      // enable just the chip ready IRQ
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0x00);
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENCHIPRDY);

      // read Si4432 interrupts to clear
      status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
      status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

      // enable XTON
      phyWrite (EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, EZRADIOPRO_XTON);

      // wait on IRQ with 2 MS timeout
      delay_set(2);
      while(IRQ)
      {
         if(delay_expired())
            return PHY_STATUS_ERROR_RADIO_XTAL;
      }

      return PHY_STATUS_SUCCESS;
   }
}

//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
PHY_STATUS rtPhySet (U8 addr, U32 value)
{
   switch (addr)
   {
      // case 0x00:
      case (TRX_FREQUENCY):
         if (value < 240000000L)
            return PHY_STATUS_ERROR_INVALID_VALUE;
         else if (value > 930000000L )
            return PHY_STATUS_ERROR_INVALID_VALUE;
         else
         {
            rtPhySettings.TRxFrequency = value;
            if(PhyInitialized)
               SetTRxFrequency(value);

            return PHY_STATUS_SUCCESS;
         }

      // case 0x01:
      case (TRX_CHANNEL_SPACING):
         if (value > 2550000L)
            return PHY_STATUS_ERROR_INVALID_VALUE;
         else
         {
            rtPhySettings.TRxChannelSpacing = value;
            if(PhyInitialized)
               SetTRxChannelSpacing(value);

            return PHY_STATUS_SUCCESS;
         }

      default:
            return PHY_STATUS_ERROR_INVALID_ADDRESS;
   }
}

/*
  This table gives the register settings for the radio core indexed by
  the desired air data rate.

  the data for this table is based on the OpenPilot rfm22b driver
 */
#define NUM_DATA_RATES 13
#define NUM_RADIO_REGISTERS 16
__code static const uint32_t air_data_rates[NUM_DATA_RATES] = {   
	         500,  1000,  2000,  4000,  8000,  9600, 16000, 19200, 24000,  32000,  64000, 128000, 192000
};

__code static const uint8_t reg_table[NUM_RADIO_REGISTERS][1+NUM_DATA_RATES] = {
	// first column is the register number
	// remaining columns are the values for the corresponding air data rate
	{ 0x1C, 0x37,  0x37,  0x37,  0x37,  0x3A,  0x3B,  0x26,  0x28,  0x2E,   0x16,   0x07,   0x83,   0x8A},
	{ 0x1D, 0x44,  0x44,  0x44,  0x44,  0x44,  0x44,  0x44,  0x44,  0x44,   0x44,   0x44,   0x44,   0x44},
	{ 0x1E, 0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,   0x0A,   0x0A,   0x0A,   0x0A},
	{ 0x1F, 0x03,  0x03,  0x03,  0x03,  0x03,  0x03,  0x03,  0x03,  0x03,   0x03,   0x03,   0x03,   0x03},
	{ 0x20, 0xE8,  0xF4,  0xFA,  0x70,  0x3F,  0x34,  0x3F,  0x34,  0x2A,   0x3F,   0x3F,   0x5E,   0x3F},
	{ 0x21, 0x60,  0x20,  0x00,  0x01,  0x02,  0x02,  0x02,  0x02,  0x03,   0x02,   0x02,   0x01,   0x02},
	{ 0x22, 0x20,  0x41,  0x83,  0x06,  0x0C,  0x75,  0x0C,  0x75,  0x12,   0x0C,   0x0C,   0x5D,   0x0C},
	{ 0x23, 0xC5,  0x89,  0x12,  0x25,  0x4A,  0x25,  0x4A,  0x25,  0x6F,   0x4A,   0x4A,   0x86,   0x4A},
	{ 0x24, 0x00,  0x00,  0x00,  0x02,  0x07,  0x07,  0x07,  0x07,  0x07,   0x07,   0x07,   0x05,   0x07},
	{ 0x25, 0x0A,  0x23,  0x85,  0x0E,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,   0xFF,   0xFF,   0x74,   0xFF},
	{ 0x2A, 0x0E,  0x0E,  0x0E,  0x0E,  0x0E,  0x0D,  0x0D,  0x0E,  0x12,   0x17,   0x31,   0x50,   0x50},
	{ 0x6E, 0x04,  0x08,  0x10,  0x20,  0x41,  0x4E,  0x83,  0x9D,  0xC4,   0x08,   0x10,   0x20,   0x31},
	{ 0x6F, 0x19,  0x31,  0x62,  0xC5,  0x89,  0xA5,  0x12,  0x49,  0x9C,   0x31,   0x62,   0xC5,   0x27},
	{ 0x70, 0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,   0x0D,   0x0D,   0x0D,   0x0D},
	{ 0x71, 0x23,  0x23,  0x23,  0x23,  0x23,  0x23,  0x23,  0x23,  0x23,   0x23,   0x23,   0x23,   0x23},
	{ 0x72, 0x06,  0x06,  0x06,  0x06,  0x06,  0x08,  0x0D,  0x0F,  0x13,   0x1A,   0x33,   0x66,   0x9A}
};


//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
PHY_STATUS rtPhyInitRadio(uint32_t air_rate)
{
   U8 status, i, rate_selection;

   // disable interrupts
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0x00);
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

   // read Si4432 interrupts to clear
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

#ifdef ENABLE_RF_SWITCH
   //set GPIO0 to GND
   phyWrite(EZRADIOPRO_GPIO0_CONFIGURATION, 0x14);	// RX data (output)
   //set GPIO1 & GPIO2 to control the TRX switch
   phyWrite(EZRADIOPRO_GPIO1_CONFIGURATION, 0x12);	// TX state (output)
   phyWrite(EZRADIOPRO_GPIO2_CONFIGURATION, 0x15);	// RX state (output)
#elif ENABLE_RFM50_SWITCH
   //set GPIO0 & GPIO1 to control the TRX switch
   phyWrite(EZRADIOPRO_GPIO0_CONFIGURATION, 0x15);	// RX state (output)
   phyWrite(EZRADIOPRO_GPIO1_CONFIGURATION, 0x12);	// TX state (output)
   //set GPIO2 to GND
   phyWrite(EZRADIOPRO_GPIO2_CONFIGURATION, 0x14);	// RX data (output)
#else
   //set GPIOx to GND
   phyWrite(EZRADIOPRO_GPIO0_CONFIGURATION, 0x14);	// RX data (output)
   phyWrite(EZRADIOPRO_GPIO1_CONFIGURATION, 0x14);	// RX data (output)
   phyWrite(EZRADIOPRO_GPIO2_CONFIGURATION, 0x14);	// RX data (output)
#endif

   //set  cap. bank
   //status = phyRead(EZRADIOPRO_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE);
   phyWrite(EZRADIOPRO_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE, EZRADIOPRO_OSC_CAP_VALUE);

   // TRX Frequency & channel spacing
   SetTRxFrequency(rtPhySettings.TRxFrequency);
   SetTRxChannelSpacing(rtPhySettings.TRxChannelSpacing);

   // enable automatic packet handling and CRC
   phyWrite(EZRADIOPRO_DATA_ACCESS_CONTROL, 
	    EZRADIOPRO_ENPACTX | 
	    EZRADIOPRO_ENCRC | 
	    EZRADIOPRO_CRC_16 |
	    EZRADIOPRO_ENPACRX);

   // set FIFO limits to max (we are not using FIFO
   // overflow/underflow interrupts)
   phyWrite(EZRADIOPRO_TX_FIFO_CONTROL_1, 0x3F);
   phyWrite(EZRADIOPRO_TX_FIFO_CONTROL_2, 0x0);
   phyWrite(EZRADIOPRO_RX_FIFO_CONTROL, 0x3F);

   // preamble setup
   phyWrite(EZRADIOPRO_PREAMBLE_LENGTH, 0x0A); // 10 nibbles, 40 bits
   phyWrite(EZRADIOPRO_PREAMBLE_DETECTION_CONTROL, 0x28); //  5 nibbles, 20 chips, 10 bits

   // 2 sync bytes and 1 header bytes
   phyWrite(EZRADIOPRO_HEADER_CONTROL_2, EZRADIOPRO_HDLEN_1BYTE | EZRADIOPRO_SYNCLEN_2BYTE);
   phyWrite(EZRADIOPRO_SYNC_WORD_3, 0x2D);
   phyWrite(EZRADIOPRO_SYNC_WORD_2, 0xD4);

   // no header checking
   phyWrite(EZRADIOPRO_HEADER_CONTROL_1, EZRADIOPRO_DISABLE_HFILTERS);
   // max output power
   phyWrite(EZRADIOPRO_TX_POWER, 0x7);


   // work out which register table column we will use
   for (i=0; i<NUM_DATA_RATES-1; i++) {
	   if (air_data_rates[i] >= air_rate) break;
   }
   rate_selection = i;

   // set the registers from the table
   for (i=0; i<NUM_RADIO_REGISTERS; i++) {
	   phyWrite(reg_table[i][0], 
		    reg_table[i][rate_selection+1]);
   }
   
   PhyInitialized = 1;

   return PHY_STATUS_SUCCESS;
}

//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
U32   divideWithRounding (U32 value, U32 divisor)
{
   value += (divisor >> 1);
   value /= divisor;
   return value;
}


//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void SetTRxFrequency (U32 frequency)
{
   U8 frequencyBandSelect;
   UU16 nominalCarrierFrequency;

   if (frequency > 480000000L )
   {
      frequency -= 480000000L;
      frequencyBandSelect  = frequency / 20000000L;
      frequency -= (U32)frequencyBandSelect * 20000000L;
      frequency  = divideWithRounding(frequency, 625);
      frequency <<= 1;
      frequencyBandSelect |= 0x20;
   }
   else
   {
      frequency -= 240000000L;
      frequencyBandSelect  = frequency / 10000000L;
      frequency -= (U32)frequencyBandSelect * 10000000L;
      frequency  = divideWithRounding(frequency, 625);
      frequency <<= 2;
   }

   frequencyBandSelect |= 0x40;        // set sbsel

   nominalCarrierFrequency.U16 = (U16)frequency;

   phyWrite(EZRADIOPRO_FREQUENCY_BAND_SELECT, frequencyBandSelect);
   phyWrite(EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_1, nominalCarrierFrequency.U8[MSB]);
   phyWrite(EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_0, nominalCarrierFrequency.U8[LSB]);

}

//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void   SetTRxChannelSpacing  (U32 channelSpacing)
{
   channelSpacing = divideWithRounding(channelSpacing, 10000);

   phyWrite(EZRADIOPRO_FREQUENCY_HOPPING_STEP_SIZE, channelSpacing);
}

/*
  start transmitting bytes from the TX FIFO
 */
#ifndef RECEIVER_ONLY
void rtPhyTxStart (U8 length, U8 txheader)
{
   U8 status;

   phyWrite(EZRADIOPRO_TRANSMIT_HEADER_3, txheader);
   phyWrite(EZRADIOPRO_TRANSMIT_PACKET_LENGTH, length);

   // enable just the packet sent IRQ
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, EZRADIOPRO_ENPKSENT);
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

   // read Si4432 interrupts to clear
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);

   // start TX
   phyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1,EZRADIOPRO_TXON|EZRADIOPRO_XTON);

   // wait on IRQ with 20 MS timeout
   delay_set(20);
   while (true) {
	   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);
	   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
	   if (status & EZRADIOPRO_IPKSENT) {
		   return;
	   }
	   if (delay_expired()) {
		   rtPhyClearTxFIFO();
		   return;
	   }
   }
   
}
#endif

/*
  put us back in receive state
 */
#ifndef TRANSMITTER_ONLY
PHY_STATUS rtPhyRxOn (void)
{
   U8 status;

   RxPacketReceived = 0;

   // enable packet valid and CRC error IRQ
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, EZRADIOPRO_ENPKVALID|EZRADIOPRO_ENCRCERROR);
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

   // read Si4432 interrupts to clear
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   // enable RX
   phyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1,EZRADIOPRO_RXON);

   EX0 = 1;

   return PHY_STATUS_SUCCESS;
}
#endif

//=============================================================================
//
// spi Functions for rtPhy.c module
//
//=============================================================================
//
// Notes:
//
// The spi functions in this module are for use in the main thread. The EZMacPro API calls should
// only be used in the main thread. The SPI is used by the main thread, as well as the external
// interrupt INT0 thread, and the T0 interrupt. Since all SPI tranfers are multiple bytes. It is
// important that MAC interrupts are disabled when using the SPI from the main thread.
//
// These SPI functions may be interrupted by other interrupts, so the double buffered transfers
// are managed with this in mind.
//
// The double buffered transfer maximizes the data throughput and elimiates any software
// delay between btyes. The clock is continuous for the two byte transfer. Instead of using the
// SPIF flag for each byte, the TXBMT is used to keep the transmit buffer full, then the SPIBSY
// bit is used to determine when all bits have been transfered. The SPIF flag should not be
// polled in double buffered transfers.
//
//-----------------------------------------------------------------------------
// Function Name phyWrite()
//
// Return Value   : None
// Parameters :
//    U8 reg - register address from the si4432.h file.
//    U8 value - value to write to register
//
// Notes:
//
//    MAC interrupts are preserved and restored.
//    Write uses a Double buffered transfer.
//
//-----------------------------------------------------------------------------
void phyWrite (U8 reg, U8 value)
{
   __bit restoreEA;

   // disable interrupts during SPI transfer
   restoreEA = EA;
   EA = 0;

   // Send SPI data using double buffered write
   NSS1 = 0;                           // drive NSS low
   SPIF1 = 0;                          // clear SPIF
   SPI1DAT = (reg | 0x80);             // write reg address
   while(!TXBMT1);                     // wait on TXBMT
   SPI1DAT = value;                    // write value
   while(!TXBMT1);                     // wait on TXBMT
   while((SPI1CFG & 0x80) == 0x80);    // wait on SPIBSY

   SPIF1 = 0;                          // leave SPIF cleared
   NSS1 = 1;                           // drive NSS high

   // Restore interrupts after SPI transfer
   EA = restoreEA;
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : U8 value - value returned from the si4432 register
// Parameters   : U8 reg - register address from the si4432.h file.
//
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Function Name
//    phyRead()
//
// Parameters   : U8 reg - register address from the si4432.h file.
// Return Value : U8 value - value returned from the si4432 register
//
//-----------------------------------------------------------------------------
U8 phyRead (U8 reg)
{
   U8 value;

   __bit restoreEA;

   // disable interrupts during SPI transfer
   restoreEA = EA;
   EA = 0;

   // Send SPI data using double buffered write
   NSS1 = 0;                           // dsrive NSS low
   SPIF1 = 0;                          // cleat SPIF
   SPI1DAT = ( reg );                  // write reg address
   while(!TXBMT1);                     // wait on TXBMT
   SPI1DAT = 0x00;                     // write anything
   while(!TXBMT1);                     // wait on TXBMT
   while((SPI1CFG & 0x80) == 0x80);    // wait on SPIBSY
   value = SPI1DAT;                    // read value
   SPIF1 = 0;                          // leave SPIF cleared
   NSS1 = 1;                           // drive NSS low

   // Restore interrupts after SPI transfer
   EA = restoreEA;

   return value;
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
#ifndef RECEIVER_ONLY
void phyWriteFIFO (U8 n, VARIABLE_SEGMENT_POINTER(buffer, U8, BUFFER_MSPACE))
{
   __bit restoreEA;

   // disable interrupts during SPI transfer
   restoreEA = EA;
   EA = 0;

   NSS1 = 0;                            // drive NSS low
   SPIF1 = 0;                           // clear SPIF
   SPI1DAT = (0x80 | EZRADIOPRO_FIFO_ACCESS);

   while(n--)
   {
      while(!TXBMT1);                   // wait on TXBMT
      SPI1DAT = *buffer++;             // write buffer
   }

   while(!TXBMT1);                      // wait on TXBMT
   while((SPI1CFG & 0x80) == 0x80);    // wait on SPIBSY

   SPIF1 = 0;                           // leave SPI  cleared
   NSS1 = 1;                            // drive NSS high

   // Restore interrupts after SPI transfer
   EA = restoreEA;
}
#endif
//=============================================================================
//
// Receiver Functions
//
//=============================================================================
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
#ifndef TRANSMITTER_ONLY
PHY_STATUS  rtPhyGetRxPacket(U8 *pLength, VARIABLE_SEGMENT_POINTER(rxBuffer, U8, BUFFER_MSPACE), U8 *rxheader)
{
   __bit restoreEX0;
   U8 i;

   if(RxPacketReceived)
   {
      // disable interrupts during copy
      restoreEX0 = EX0;
      EX0 = 0;

      for(i=0;i<RxPacketLength;i++)
      {
         rxBuffer[i]=RxIntBuffer[i];
      }

      *rxheader = RxHeader;
      RxPacketReceived = 0;

      EX0 = restoreEX0;

      *pLength = i;

      return PHY_STATUS_SUCCESS;
   }
   else
   {
      return PHY_STATUS_ERROR_NO_PACKET;
   }
}

#endif

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
// Prevent allocation of arguments and local variables in the overlay segment
// for routines below, which can be called from interrupt context.
#pragma save
#pragma nooverlay

#ifndef TRANSMITTER_ONLY
INTERRUPT(Receiver_ISR, INTERRUPT_INT0)
{
   U8 status;

   IE0 = 0;

   status = RxIntPhyRead(EZRADIOPRO_INTERRUPT_STATUS_2);
   status = RxIntPhyRead(EZRADIOPRO_INTERRUPT_STATUS_1);

   if((status & EZRADIOPRO_IPKVALID)==EZRADIOPRO_IPKVALID)
   {
      if(RxPacketReceived==0)
      {
	      RxPacketReceived = 1;
	      RxPacketLength = RxIntPhyRead(EZRADIOPRO_RECEIVED_PACKET_LENGTH);
	      RxHeader       = RxIntPhyRead(EZRADIOPRO_RECEIVED_HEADER_3);
	      if (RxPacketLength != 0) {
		      RxIntphyReadFIFO(RxPacketLength, RxIntBuffer);
	      }
      }
      else
      {
         // Clear RX FIFO
         status = RxIntPhyRead(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2);
         status |= EZRADIOPRO_FFCLRRX;
         RxIntPhyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, status);
         status &= ~EZRADIOPRO_FFCLRRX;
         RxIntPhyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, status);
      }
   }
   else if((status & EZRADIOPRO_ICRCERROR)==EZRADIOPRO_ICRCERROR)
   {
      RxErrors++;
   }
   else
   {
   }

   // enable packet valid and CRC error IRQ
   RxIntPhyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, EZRADIOPRO_ENPKVALID|EZRADIOPRO_ENCRCERROR);
   RxIntPhyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

   // enable RX again
   RxIntPhyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1,(EZRADIOPRO_RXON|EZRADIOPRO_XTON));
}
#endif


/*
  clear the send FIFO
 */
void rtPhyClearTxFIFO(void)
{
	uint8_t status;
	status = RxIntPhyRead(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2);
	status |= EZRADIOPRO_FFCLRTX;
	RxIntPhyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, status);
	status &= ~EZRADIOPRO_FFCLRTX;
	RxIntPhyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, status);
}


//=============================================================================
//
// spi Functions for Rx Receiver interrupt
//
//=============================================================================
//-----------------------------------------------------------------------------
// Function Name RxIntPhyWrite()
//
// Return Value   : None
// Parameters :
//    U8 reg - register address from the si4432.h file.
//    U8 value - value to write to register
//
// Notes:
//
//    MAC interrupts are preserved and restored.
//    Write uses a Double buffered transfer.
//
//-----------------------------------------------------------------------------
#ifndef TRANSMITTER_ONLY
void RxIntPhyWrite (U8 reg, U8 value)
{
   __bit restoreEA;

   // disable interrupts during SPI transfer
   restoreEA = EA;
   EA = 0;

   // Send SPI data using double buffered write
   NSS1 = 0;                           // drive NSS low
   SPIF1 = 0;                          // clear SPIF
   SPI1DAT = (reg | 0x80);             // write reg address
   while(!TXBMT1);                     // wait on TXBMT
   SPI1DAT = value;                    // write value
   while(!TXBMT1);                     // wait on TXBMT
   while((SPI1CFG & 0x80) == 0x80);    // wait on SPIBSY

   SPIF1 = 0;                          // leave SPIF cleared
   NSS1 = 1;                           // drive NSS high

   // Restore interrupts after SPI transfer
   EA = restoreEA;
}
#endif
//-----------------------------------------------------------------------------
// Function Name
//    RxIntPhyRead()
//
// Parameters   : U8 reg - register address from the si4432.h file.
// Return Value : U8 value - value returned from the si4432 register
//
//-----------------------------------------------------------------------------
#ifndef TRANSMITTER_ONLY
U8 RxIntPhyRead (U8 reg)
{
   U8 value;

   __bit restoreEA;

   // disable interrupts during SPI transfer
   restoreEA = EA;
   EA = 0;

   // Send SPI data using double buffered write
   NSS1 = 0;                           // dsrive NSS low
   SPIF1 = 0;                          // cleat SPIF
   SPI1DAT = ( reg );                  // write reg address
   while(!TXBMT1);                     // wait on TXBMT
   SPI1DAT = 0x00;                     // write anything
   while(!TXBMT1);                     // wait on TXBMT
   while((SPI1CFG & 0x80) == 0x80);    // wait on SPIBSY
   value = SPI1DAT;                    // read value
   SPIF1 = 0;                          // leave SPIF cleared
   NSS1 = 1;                           // drive NSS low

   // Restore interrupts after SPI transfer
   EA = restoreEA;

   return value;
}
#endif
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
// Note that this function does no use double buffered data transfers to prevent data loss.
// This function may be interrupted by another process and should not loose dat or hang on the
// SPIF flag.
//
//-----------------------------------------------------------------------------
#ifndef TRANSMITTER_ONLY
void RxIntphyReadFIFO (U8 n, VARIABLE_SEGMENT_POINTER(buffer, U8, BUFFER_MSPACE))
{
   __bit restoreEA;

   // disable interrupts during SPI transfer
   restoreEA = EA;
   EA = 0;

   NSS1 = 0;                            // drive NSS low
   SPIF1 = 0;                           // clear SPIF
   SPI1DAT = (EZRADIOPRO_FIFO_ACCESS);
   while(!SPIF1);                       // wait on SPIF
   ACC = SPI1DAT;                      // discard first byte

   while(n--)
   {
      SPIF1 = 0;                        // clear SPIF
      SPI1DAT = 0x00;                  // write anything
      while(!SPIF1);                    // wait on SPIF
      *buffer++ = SPI1DAT;             // copy to buffer
   }

   SPIF1 = 0;                           // leave SPIF cleared
   NSS1 = 1;                            // drive NSS high

   // Restore interrupts after SPI transfer
   EA = restoreEA;
}
#endif

#pragma restore

//-----------------------------------------------------------------------------
//=============================================================================
// end rtPhy..c
//=============================================================================
