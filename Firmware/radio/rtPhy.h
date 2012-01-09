#ifndef  RT_PHY_H
#define  RT_PHY_H
//================================================================================================
// Phy.h
//------------------------------------------------------------------------------------------------
// Copyright 2009 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Header File Description:
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
//    MCM Code Examples
//
// Beta Release 0.9
//    - TBD
//
// This software must be used in accordance with the End User License Agreement.
//
//================================================================================================
#include "compiler_defs.h"
#include "rtPhy_defs.h"
//-----------------------------------------------------------------------------
// Expected RADIO_VERSION code for radio VERSION register
// Production version should be 0x06
//-----------------------------------------------------------------------------
#define MIN_RADIO_VERSION   0x05
//------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------
#define SYSCLK_HZ                (24500000L)
#define MILLISECONDS(t)          (((SYSCLK_HZ/1000)*(t))/48)
//-----------------------------------------------------------------------------
// ET0 inverted and used as timeout flag
//-----------------------------------------------------------------------------
#define TIMEOUT_T0   (!ET0)
//------------------------------------------------------------------------------------------------
// PHY Variables accessed using phySet()/phyget()
//------------------------------------------------------------------------------------------------
enum
{
 TRX_FREQUENCY = 0,
 TRX_CHANNEL_SPACING,
 TRX_DATA_RATE
};
//------------------------------------------------------------------------------------------------
// phy data structure typedef - must match enum above
//------------------------------------------------------------------------------------------------
typedef struct rtPhySettingsStruct
{
 U32 TRxFrequency;
 U32 TRxChannelSpacing;
 U32 TRxDataRate;
} rtPhySettingsStruct;
//------------------------------------------------------------------------------------------------
// phy status values
//------------------------------------------------------------------------------------------------
#define PHY_STATUS U8
enum
{
   PHY_STATUS_SUCCESS = 0x00,
   PHY_STATUS_ERROR_SPI,
   PHY_STATUS_ERROR_UNSUPPORTED_RADIO,
   PHY_STATUS_ERROR_NO_IRQ,
   PHY_STATUS_ERROR_RADIO_XTAL,
   PHY_STATUS_ERROR_RADIO_SHUTDOWN,
   PHY_STATUS_ERROR_READ_ONLY_ADDRESS,
   PHY_STATUS_ERROR_INVALID_ADDRESS,
   PHY_STATUS_ERROR_INVALID_VALUE,
   PHY_STATUS_TRANSMIT_ERROR,
   PHY_STATUS_ERROR_NO_PACKET
};
//------------------------------------------------------------------------------------------------
// Public variables (API)
//------------------------------------------------------------------------------------------------
extern __bit RxPacketReceived;
extern SEGMENT_VARIABLE (RxPacketLength, U8, BUFFER_MSPACE);
extern SEGMENT_VARIABLE (RxErrors, U8, BUFFER_MSPACE);
//------------------------------------------------------------------------------------------------
// Public Run Time PHY function prototypes (API)
//------------------------------------------------------------------------------------------------
PHY_STATUS    rtPhyInit (void);          // called once after MCU reset
PHY_STATUS    rtPhySet (U8, U32);
PHY_STATUS    rtPhyGet (U32 *);
PHY_STATUS    rtPhyInitRadio (void);
void          phyWriteFIFO (U8, VARIABLE_SEGMENT_POINTER(buffer, U8, BUFFER_MSPACE));
void 	      rtPhyTxStart (U8, U8);
void          rtPhyClearTxFIFO(void);
PHY_STATUS    rtPhyRxOn (void);
PHY_STATUS    rtPhyRxOff (void);
PHY_STATUS    rtPhyGetRxPacket (U8*, VARIABLE_SEGMENT_POINTER(buffer, U8, BUFFER_MSPACE), U8*);
//------------------------------------------------------------------------------------------------
// Public primitive phy function prototypes
//------------------------------------------------------------------------------------------------
void  phyWrite (U8, U8);
U8    phyRead (U8);
//=================================================================================================
//=================================================================================================
#endif //RT_PHY_H
