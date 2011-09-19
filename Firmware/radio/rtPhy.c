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
#include "rtPhy_const.h"
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
rtPhySettingsStruct SEG_XDATA rtPhySettings;
__bit PhyInitialized = 0;

//=============================================================================
// local functions
//=============================================================================
U8    InitSoftwareReset(void);
void  SetTimeOut (U16);
void  ClearTimeOut (void);
void  delay (U16);


void  SetTRxFrequency (U32);
void  SetTRxChannelSpacing  (U32);
void  SetTxFrequencyDeviation (U32);
void  SetTxDataRate (U32);

void  UpdateRxModemSettings(void);
U8    LookUpFilterIndex (U32);
U32   CalcAFC_PullInRange(U32);
void  SetAFC_Limit (U32);
U16   CalcRxOverSamplingRatio (U8, U32);
void  SetRxOverSamplingRatio (U16);
U32   CalcClockRecoveryOffset (U8, U32);
void  SetClockRecoveryOffset (U32);
U16   CalcClockRecoveryTimingLoopGain (U32, U16, U32);
void  SetClockRecoveryTimingLoopGain (U16);

void InitConfigSettings(void);

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

   delay(MILLISECONDS(25));

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
      SetTimeOut(MILLISECONDS(2));
      while(IRQ)
      {
         if(TIMEOUT_T0)
            return PHY_STATUS_ERROR_RADIO_XTAL;
      }
      ClearTimeOut();
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
   SetTimeOut(MILLISECONDS(2));
   while(IRQ)
   {
      if(TIMEOUT_T0)
         return PHY_STATUS_ERROR_NO_IRQ;
   }
   ClearTimeOut();

   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   if((status & EZRADIOPRO_ICHIPRDY)==0)
   {
      // enable Chip read only
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENCHIPRDY);

      // wait on IRQ with 2 MS timeout
      SetTimeOut(MILLISECONDS(2));
      while(IRQ)
      {
         if(TIMEOUT_T0)
            return PHY_STATUS_ERROR_RADIO_XTAL;
      }
      ClearTimeOut();
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
      SetTimeOut(MILLISECONDS(2));
      while(IRQ)
      {
         if(TIMEOUT_T0)
            return PHY_STATUS_ERROR_RADIO_XTAL;
      }
      ClearTimeOut();

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
PHY_STATUS rtPhyStandby (void)
{
   U8 status;

   // disable interrupts
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0x00);
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

   // read Si4432 interrupts to clear
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   // stop XTAL
   phyWrite (EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, 0);

   return PHY_STATUS_SUCCESS; // success
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
PHY_STATUS rtPhyShutDown (void)
{
   P2MDOUT |= 0x40;                    // Enable SDN push pull
   SDN = 1;
   PhyInitialized = 0;

   return PHY_STATUS_SUCCESS; // success
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
PHY_STATUS rtPhyReStart(void)
{
   U8 status;

   SDN = 0;

   delay(MILLISECONDS(2));

   // wait on IRQ with 25 MS timeout
   SetTimeOut(MILLISECONDS(25));
   while(IRQ)
   {
      if(TIMEOUT_T0)
         return PHY_STATUS_ERROR_RADIO_XTAL;
   }
   ClearTimeOut();

   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   if((status & EZRADIOPRO_IPOR)==0)
   {
      // radio needs a software reset
      return InitSoftwareReset();
   }
   else if((status & EZRADIOPRO_ICHIPRDY)==0)
   {
      // disable POR interrupt
      phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENCHIPRDY);

      // wait on IRQ with 2 MS timeout
      SetTimeOut(MILLISECONDS(2));
      while(IRQ)
      {
         if(TIMEOUT_T0)
            return PHY_STATUS_ERROR_RADIO_XTAL;
      }
      ClearTimeOut();
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
      // case 0x02:
      case (TRX_DEVIATION):
         if (value >= 320000L)
            return PHY_STATUS_ERROR_INVALID_VALUE;
         else
         {
            rtPhySettings.TRxDeviation = value;
            if(PhyInitialized)
            {
               SetTxFrequencyDeviation(value);
               UpdateRxModemSettings();
            }
            return PHY_STATUS_SUCCESS;
         }

      // case 0x03:
      case (TRX_DATA_RATE):
         if (value > 125000L)
            return PHY_STATUS_ERROR_INVALID_VALUE;
         else
         {
            rtPhySettings.TRxDataRate = value;
            if(PhyInitialized)
            {
               SetTxDataRate(value);
               UpdateRxModemSettings();
            }

            return PHY_STATUS_SUCCESS;
         }

      // case 0x04:
      case (RX_BAND_WIDTH):
         if (value > 620700000L)
            return PHY_STATUS_ERROR_INVALID_VALUE;
         else
         {
            rtPhySettings.RxBandWidth = value;
            if(PhyInitialized)
               UpdateRxModemSettings();

            return PHY_STATUS_SUCCESS;
         }

      default:
            return PHY_STATUS_ERROR_INVALID_ADDRESS;
   }
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
PHY_STATUS rtPhyInitRadio (void)
{
   U8 status;

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


   InitConfigSettings();

   //Init Radio registers using current settings
   // TRX Frequency & Modem Settings
   SetTRxFrequency(rtPhySettings.TRxFrequency);
   SetTRxChannelSpacing(rtPhySettings.TRxChannelSpacing);
   SetTxFrequencyDeviation(rtPhySettings.TRxDeviation);
   SetTxDataRate(rtPhySettings.TRxDataRate);
   // RX Modem Settings
   UpdateRxModemSettings();

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
void InitConfigSettings (void)
{
   U8 i;
   U8 addr;
   U8 value;

  //Set the Radio Parameters from rtPhy_const
   for(i = 0; i < NUMBER_OF_INIT_REGISTERS; i++)
   {
      addr = rtPhyInitRegisters[i];
      value = rtPhyInitSettings[i];
      phyWrite(addr, value);
   }
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
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void SetTxFrequencyDeviation (U32 deviation)
{
   U8 txFrequencyDeviation;
   U8 modulationControl2Mask;

   deviation = divideWithRounding (deviation, 625);

   txFrequencyDeviation = (U8)deviation;

   if (deviation > 255)
      modulationControl2Mask = 0x04;
   else
      modulationControl2Mask = 0x00;

   phyWrite(EZRADIOPRO_FREQUENCY_DEVIATION, txFrequencyDeviation);
   phyWrite(EZRADIOPRO_MODULATION_MODE_CONTROL_2,
      (phyRead(EZRADIOPRO_MODULATION_MODE_CONTROL_2)|modulationControl2Mask));
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void SetTxDataRate (U32 dataRate)
{
   UU16  txDataRate;
   U8    modulationControl1Mask;

  if (dataRate < 30000)
   {
      // dataRate = dataRate * 2^21 / 10^6  : exceeds 32-bits
      // dataRate = dataRate * 2^17 / 62500 : OK for 32 bit math
      dataRate <<= 17;
      dataRate = divideWithRounding (dataRate, 62500L);
      modulationControl1Mask = 0x20; //set txdtrtscale
   }
   else
   {
      // dataRate = dataRate * 2^16 / 10^6   : exceeds 32-bits
      // dataRate = dataRate * 2^15 / 500000 : OK for 32 bit math
     dataRate <<= 15;
     dataRate = divideWithRounding (dataRate, 500000L);
     modulationControl1Mask = 0x00; //don't set txdtrtscale
   }

   txDataRate.U16 = (U16)dataRate;

   // TX Modem Settings
   phyWrite(EZRADIOPRO_TX_DATA_RATE_1, txDataRate.U8[MSB]);
   phyWrite(EZRADIOPRO_TX_DATA_RATE_0, txDataRate.U8[LSB]);
   phyWrite(EZRADIOPRO_MODULATION_MODE_CONTROL_1,
      (phyRead(EZRADIOPRO_MODULATION_MODE_CONTROL_1)|modulationControl1Mask));

}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void UpdateRxModemSettings()
{
   // local rX modem variables

   U8    filterIndex;            // used with look-up table
   U8    filterSetting;            // used with look-up table
   U32   afcPullInRange;
   U16   rxOverSamplingRatio;
   U32   clockRecoveryOffset;
   U16   loopGain;

   // Use RX bandwidth to look-up filter index.
   filterIndex = LookUpFilterIndex (rtPhySettings.RxBandWidth);
   // use index to set filter setting register
   filterSetting = rtPhyTableIF_FilterSetting[filterIndex];
   phyWrite(EZRADIOPRO_IF_FILTER_BANDWIDTH, filterSetting);

   // also use index to update Rxbandwidth
   rtPhySettings.RxBandWidth = (rtPhyTableRxBandwidth[filterIndex]) * (100L);

   // Use updated Rxbandwidth to calculate afc pull in and set afc limit.
   afcPullInRange = CalcAFC_PullInRange(rtPhySettings.RxBandWidth);
   SetAFC_Limit(afcPullInRange);

   // Use Filter setting and RxDataRate to set RxoverSamplingRatio
   rxOverSamplingRatio = CalcRxOverSamplingRatio (filterSetting, rtPhySettings.TRxDataRate);
   SetRxOverSamplingRatio(rxOverSamplingRatio);

   // Use Filter setting and RxDataRate to set ClockRecoveryOffset
   clockRecoveryOffset = CalcClockRecoveryOffset (filterSetting, rtPhySettings.TRxDataRate);
   SetClockRecoveryOffset (clockRecoveryOffset);

   // Use RxDataRate, RxoverSamplingRatio, and RxDeviation to set Loop Gain
   loopGain = CalcClockRecoveryTimingLoopGain (rtPhySettings.TRxDataRate, rxOverSamplingRatio, rtPhySettings.TRxDeviation);
   SetClockRecoveryTimingLoopGain(loopGain);
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
U8 LookUpFilterIndex (U32 RxBandwidth)
{
   U8 i;

   RxBandwidth  = divideWithRounding (RxBandwidth, 100);

   i=0;

   // Find largest value in table smaller than target.
   while (rtPhyTableRxBandwidth[i] <  RxBandwidth)
   {
      i++;
   }

   return i;
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
U32 CalcAFC_PullInRange (U32 rxBandwidth)
{
   U32 afcPullIn;

   afcPullIn = rxBandwidth * 7;
   afcPullIn = divideWithRounding (afcPullIn, 10);
   return afcPullIn;
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void SetAFC_Limit (U32 afcLimit)
{
   U8 frequencyBandSelect;

   //TRX frequency must be set first
   frequencyBandSelect = phyRead(EZRADIOPRO_FREQUENCY_BAND_SELECT);

   if((frequencyBandSelect&0x20)==0x20)
   {
      afcLimit>>=1;
   }

   afcLimit = divideWithRounding (afcLimit, 625);

   if(afcLimit>0xFF)
      afcLimit = 0xFF;

   phyWrite(EZRADIOPRO_AFC_LIMITER, afcLimit);
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
U16 CalcRxOverSamplingRatio (U8 filter, U32 rxDataRate)
{
   U32 rxOverSamplingRatio;
   U8 ndec_exp, dwn3_bypass;

   filter >>= 4;
   ndec_exp = filter & 0x07;
   filter >>= 3;
   dwn3_bypass = filter & 0x01;

   //calculate rxOverSamplingRatio
   if(dwn3_bypass)
      rxOverSamplingRatio = (500000L * 3 * 8);
   else
      rxOverSamplingRatio = (500000L * 8);

   rxOverSamplingRatio >>= ndec_exp;
   rxOverSamplingRatio = divideWithRounding(rxOverSamplingRatio, rxDataRate);

   if (rxOverSamplingRatio > 0x07FF)    // limit to 11 bits
      rxOverSamplingRatio = 0x07FF;

   return (U16) rxOverSamplingRatio;
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void SetRxOverSamplingRatio (U16 rxOverSamplingRatio)
{
   U8 ClockRecoveryOffset2;

   if (rxOverSamplingRatio > 0x07FF)    // limit to 11 bits
      rxOverSamplingRatio = 0x07FF;

   ClockRecoveryOffset2 = phyRead(EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2);
   ClockRecoveryOffset2 &= 0x1F; // clear rxOverSamplingRatio bits
   ClockRecoveryOffset2 |= ((rxOverSamplingRatio>>3)&0xE0);

   phyWrite(EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2, ClockRecoveryOffset2);
   phyWrite(EZRADIOPRO_CLOCK_RECOVERY_OVERSAMPLING_RATIO, (U8)rxOverSamplingRatio);
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
U32 CalcClockRecoveryOffset (U8 filter, U32 rxDataRate)
{
   U32 clockRecoveryOffset;

   U8 filset, ndec_exp, dwn3_bypass;

   filset = filter & 0x0F;
   filter >>= 4;
   ndec_exp = filter & 0x07;
   filter >>= 3;
   dwn3_bypass = filter & 0x01;

   // calculate clockRecoveryOffset
   clockRecoveryOffset = rxDataRate;
   // limit operands to 32-bits
   // clockRecoveryOffset = clockRecoveryOffset * 2^20 /500000 : exceeds 32-bit
   // clockRecoveryOffset = clockRecoveryOffset * 2^15 /15625  : OK
  
   clockRecoveryOffset <<= 15;
 
#ifdef __RC51__
   // bug fix for Raisonance compiler
   // divide by 15625 does not work
   // so divide by 125 twice
   // 125 * 125 = 15625
   clockRecoveryOffset = divideWithRounding (clockRecoveryOffset, 125);
   clockRecoveryOffset = divideWithRounding (clockRecoveryOffset, 125);
#else
   clockRecoveryOffset = divideWithRounding (clockRecoveryOffset, 15625);
#endif

   clockRecoveryOffset <<= ndec_exp;
   if(dwn3_bypass)
   {
     clockRecoveryOffset = divideWithRounding (clockRecoveryOffset, 3);
   }

   if (clockRecoveryOffset > 0x000FFFFF)    // limit to 20 bits
      clockRecoveryOffset = 0x000FFFFF;

   return clockRecoveryOffset;
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void SetClockRecoveryOffset (U32 clockRecoveryOffset)
{
   UU16 ncoff;
   U8 ClockRecoveryOffset2;

   if (clockRecoveryOffset > 0x000FFFFF)    // limit to 20 bits
      clockRecoveryOffset = 0x000FFFFF;

   ClockRecoveryOffset2 = phyRead(EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2);
   ClockRecoveryOffset2 &= 0xF0; // clear ncoff bits
   ClockRecoveryOffset2 |= ((clockRecoveryOffset>>16)&0x0F);

   ncoff.U16 = (U16) clockRecoveryOffset;

   phyWrite(EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2, ClockRecoveryOffset2);
   phyWrite(EZRADIOPRO_CLOCK_RECOVERY_OFFSET_1, ncoff.U8[MSB]);
   phyWrite(EZRADIOPRO_CLOCK_RECOVERY_OFFSET_0, ncoff.U8[LSB]);
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
U16 CalcClockRecoveryTimingLoopGain (U32 rxDataRate, U16 RxOverSamplingRatio, U32 RxDeviation)
{
   U32 clockRecoveryTimingLoopGain;

    // calulate clockRecoveryTimingLoopGain
   clockRecoveryTimingLoopGain = rxDataRate;
   clockRecoveryTimingLoopGain <<= 15;
   clockRecoveryTimingLoopGain = divideWithRounding (clockRecoveryTimingLoopGain, RxOverSamplingRatio);
   clockRecoveryTimingLoopGain = divideWithRounding (clockRecoveryTimingLoopGain, RxDeviation);
   clockRecoveryTimingLoopGain += 2;

   if (clockRecoveryTimingLoopGain > 0x07FF)    // limit to 11 bits
      clockRecoveryTimingLoopGain = 0x07FF;

   return (U16) clockRecoveryTimingLoopGain;
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
void SetClockRecoveryTimingLoopGain (U16 clockRecoveryTimingLoopGain)
{
   UU16 crgain;

   if (clockRecoveryTimingLoopGain > 0x07FF)    // limit to 11 bits
      clockRecoveryTimingLoopGain = 0x07FF;

   crgain.U16 = clockRecoveryTimingLoopGain;

   phyWrite(EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_1, crgain.U8[MSB]);
   phyWrite(EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_0, crgain.U8[LSB]);
}
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
S8 PhySetTxPower (S8 power)
{
   if(power > 20)
   {
      power = 20;
   }
   else if (power < -1)
   {
      power = -1;
   }

   power = (power+1)/3;

   phyWrite(EZRADIOPRO_TX_POWER, power);

   power = power*3 - 1;

   return power;
}

//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
// PHY_STATUS rtPhyHopChannel (U8 channel)
// {
//    phyWrite(EZRADIOPRO_FREQUENCY_HOPPING_CHANNEL_SELECT, channel);
//    return PHY_STATUS_SUCCESS;
// }
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
#ifndef RECEIVER_ONLY
U8 rtPhyTx (U8 length, VARIABLE_SEGMENT_POINTER(txBuffer, U8, BUFFER_MSPACE))
{
   U8 status;

   phyWrite(EZRADIOPRO_TRANSMIT_PACKET_LENGTH, length);

   phyWriteFIFO(length, txBuffer);

   // enable just the packet sent IRQ
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, EZRADIOPRO_ENPKSENT);
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

   // read Si4432 interrupts to clear
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   // start TX
   phyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1,(EZRADIOPRO_TXON|EZRADIOPRO_XTON));

   // wait on IRQ with 70 MS timeout
   // time out based on longest message and lowest data rate
   SetTimeOut(MILLISECONDS(70));
   while(IRQ)
   {
      if(TIMEOUT_T0)
       return PHY_STATUS_ERROR_RADIO_XTAL;
   }
   ClearTimeOut();

   return 0;
}
#endif

//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
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
   phyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1,(EZRADIOPRO_RXON|EZRADIOPRO_XTON));

   EX0 = 1;

   return PHY_STATUS_SUCCESS;
}
#endif
//-----------------------------------------------------------------------------
// Function Name
//
// Return Value : None
// Parameters   :
//
//-----------------------------------------------------------------------------
#ifndef TRANSMITTER_ONLY
PHY_STATUS rtPhyRxOff (void)
{
   U8 status;

   EX0 = 0;

   // clear interrupt enables
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_1, 0x00);
   phyWrite(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

   // read Si4432 interrupts to clear
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_1);
   status = phyRead(EZRADIOPRO_INTERRUPT_STATUS_2);

   // disable RX
   phyWrite(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1,(EZRADIOPRO_XTON));

   return PHY_STATUS_SUCCESS;
}
#endif
//-----------------------------------------------------------------------------
// delay ()
//-----------------------------------------------------------------------------
void delay (U16 ticks)
{
   UU16 reload;

   reload.U16 = -ticks;

   TR0 = 0;
   TF0 = 0;

   TMOD  &= ~0x0F;                     // clear T0 bits in TMOD
   TMOD  |=  0x01;                     // T0 uses prescaller
   CKCON &= ~0x03;                     // clear T0 bits in CKCON
   CKCON |=  0x02;                     // divide by 48 prescaller

   TL0     = reload.U8[LSB];
   TH0     = reload.U8[MSB];

   TR0 = 1;
   while (!TF0);
   TR0 = 0;
   TF0 = 0;
}
//-----------------------------------------------------------------------------
// timeout ()
//-----------------------------------------------------------------------------
void SetTimeOut (U16 ticks)
{
   UU16 reload;

   reload.U16 = -ticks;

   TR0 = 0;
   TF0 = 0;

   TMOD  &= ~0x0F;                     // clear T0 bits in TMOD
   TMOD  |=  0x01;                     // T0 uses prescaller
   CKCON &= ~0x03;                     // clear T0 bits in CKCON
   CKCON |=  0x02;                     // divide by 48 prescaller

   TL0     = reload.U8[LSB];
   TH0     = reload.U8[MSB];

   TR0 = 1;
   ET0 = 1;
}
//-----------------------------------------------------------------------------
// T0_ISR() used with timeout
//-----------------------------------------------------------------------------
INTERRUPT(T0_ISR, INTERRUPT_TIMER0)
{
   ET0 = 0;
   TF0 = 0;
   TR0 = 0;
}
//-----------------------------------------------------------------------------
// ClearTimeOut() used to cancel TimeOut
//-----------------------------------------------------------------------------
void ClearTimeOut (void)
{
   ET0 = 0;
   TF0 = 0;
   TR0 = 0;
}
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
PHY_STATUS  rtPhyGetRxPacket(U8 *pLength, VARIABLE_SEGMENT_POINTER(rxBuffer, U8, BUFFER_MSPACE))
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

      RxPacketReceived = 0;

      EX0 = restoreEX0;

      *pLength = RxPacketLength;

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
         RxPacketLength = RxIntPhyRead(EZRADIOPRO_RECEIVED_PACKET_LENGTH);
         RxIntphyReadFIFO(RxPacketLength, RxIntBuffer);
         RxPacketReceived = 1;
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

//-----------------------------------------------------------------------------
//=============================================================================
// end rtPhy..c
//=============================================================================
