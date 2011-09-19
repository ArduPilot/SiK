// phy.c
//================================================================================================
// Copyright 2009 Silicon Laboratories, Inc.
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
//    Pong
//
// Beta Release 0.1
//    - TBD
//
// This software must be used in accordance with the End User License Agreement.
//
//================================================================================================
//------------------------------------------------------------------------------------------------
// Includes
//
// These includes must be in a specific order. Dependencies listed in comments.
//------------------------------------------------------------------------------------------------
#include "radio.h"
#include "rtPhy_defs.h"
#include "rtPhy_const.h"
//-------------------------------------------------------------------------------------------------
// Phy Radio Register to be set
//-------------------------------------------------------------------------------------------------
const SEGMENT_VARIABLE (rtPhyInitRegisters[NUMBER_OF_INIT_REGISTERS], U8, SEG_CODE) =
{
   EZRADIOPRO_DATA_ACCESS_CONTROL,                  // 0x30
   EZRADIOPRO_HEADER_CONTROL_1,                     // 0x32
   EZRADIOPRO_HEADER_CONTROL_2,                     // 0x33
   EZRADIOPRO_PREAMBLE_LENGTH,                      // 0x34
   EZRADIOPRO_PREAMBLE_DETECTION_CONTROL,           // 0x35
   EZRADIOPRO_MODULATION_MODE_CONTROL_1,            // 0x70
   EZRADIOPRO_MODULATION_MODE_CONTROL_2,            // 0x71
};
//---------------------------------------------------------------------------------------------------
// 868 MHz Band Radio Settings
//---------------------------------------------------------------------------------------------------
const SEGMENT_VARIABLE (rtPhyInitSettings[NUMBER_OF_INIT_REGISTERS], U8, SEG_CODE) =
{
   0x8C, // 0x30 - SI4432_DATA_ACCESS_CONTROL - enable TX & RX packet handler, enable CRC
   0x00, // 0x32 - SI4432_HEADER_CONTROL_1 - no header
   0x02, // 0x33 - SI4432_HEADER_CONTROL_2 - 2 byte sync word, variable packet length
   0x0A, // 0x34 - SI4432_PREAMBLE_LENGTH - 10 nibbles, 40 bits
   0x28, // 0x35 - SI4432_PREAMBLE_DETECTION_CONTROL -  5 nibbles, 20 chips, 10 bits
   0x0C, // 0x70 - EZRADIOPRO_MODULATION_MODE_CONTROL_1 default Manchester disabled
   0x23, // 0x71 - EZRADIOPRO_MODULATION_MODE_CONTROL_2 - FIFO mode, GFSK
};
//-------------------------------------------------------------------------------------------------
//
// Look up table for IF Filter Settings
//
//-------------------------------------------------------------------------------------------------
const SEGMENT_VARIABLE (rtPhyTableRxBandwidth[NUMBER_OF_RX_BANDWIDTH_VALUES], U16, SEG_CODE) =
{
   26,   //   2.6 kHz
   28,   //   2.8 kHz
   31,   //   3.1 kHz
   32,   //   3.2 kHz
   37,   //   3.7 kHz
   42,   //   4.2 kHz
   45,   //   4.5 kHz
   49,   //   4.9 kHz
   54,   //   5.4 kHz
   59,   //   5.9 kHz
   61,   //   6.1 kHz
   72,   //   7.2 kHz
   82,   //   8.2 kHz
   88,   //   8.8 kHz
   95,   //   9.5 kHz
   106,  //  10.6 kHz
   115,  //  11.5 kHz
   121,  //  12.1 kHz
   142,  //  14.2 kHz
   162,  //  16.2 kHz
   175,  //  17.5 kHz
   189,  //  18.9 kHz
   210,  //  21.0 kHz
   227,  //  22.7 kHz
   240,  //  24.0 kHz
   282,  //  28.2 kHz
   322,  //  32.2 kHz
   347,  //  34.7 kHz
   377,  //  37.7 kHz
   417,  //  41.7 kHz
   452,  //  45.2 kHz
   479,  //  47.9 kHz
   562,  //  56.2 kHz
   641,  //  64.1 kHz
   692,  //  69.2 kHz
   752,  //  75.2 kHz
   832,  //  83.2 kHz
   900,  //  90.0 kHz
   953,  //  95.3 kHz
   1121, // 112.1 kHz
   1279, // 127.9 kHz
   1379, // 137.9 kHz
   1428, // 142.8 kHz
   1678, // 167.8 kHz
   1811, // 181.1 kHz
   1915, // 191.5 kHz
   2251, // 225.1 kHz
   2488, // 248.8 kHz
   2693, // 269.3 kHz
   2849, // 284.9 kHz
   3355, // 335.5 kHz
   3618, // 361.8 kHz
   4202, // 420.2 kHz
   4684, // 468.4 kHz
   5188, // 518.8 kHz
   5770, // 577.0 kHz
   6207, // 620.7 kHz
};

const SEGMENT_VARIABLE (rtPhyTableIF_FilterSetting[NUMBER_OF_RX_BANDWIDTH_VALUES], U8, __code) =
{
   0x51, //   2.6 kHz, dwn3_byp = 0, ndec_exp = 5 , filset = 1
   0x52, //   2.8 kHz, dwn3_byp = 0, ndec_exp = 5 , filset = 2
   0x53, //   3.1 kHz, dwn3_byp = 0, ndec_exp = 5 , filset = 3
   0x54, //   3.2 kHz, dwn3_byp = 0, ndec_exp = 5 , filset = 4
   0x55, //   3.7 kHz, dwn3_byp = 0, ndec_exp = 5 , filset = 5
   0x56, //   4.2 kHz, dwn3_byp = 0, ndec_exp = 5 , filset = 6
   0x57, //   4.5 kHz, dwn3_byp = 0, ndec_exp = 5 , filset = 7
   0x41, //   4.9 kHz, dwn3_byp = 0, ndec_exp = 4 , filset = 1
   0x42, //   5.4 kHz, dwn3_byp = 0, ndec_exp = 4 , filset = 2
   0x43, //   5.9 kHz, dwn3_byp = 0, ndec_exp = 4 , filset = 3
   0x44, //   6.1 kHz, dwn3_byp = 0, ndec_exp = 4 , filset = 4
   0x45, //   7.2 kHz, dwn3_byp = 0, ndec_exp = 4 , filset = 5
   0x46, //   8.2 kHz, dwn3_byp = 0, ndec_exp = 4 , filset = 6
   0x47, //   8.8 kHz, dwn3_byp = 0, ndec_exp = 4 , filset = 7
   0x31, //   9.5 kHz, dwn3_byp = 0, ndec_exp = 3 , filset = 1
   0x32, //  10.6 kHz, dwn3_byp = 0, ndec_exp = 3 , filset = 2
   0x33, //  11.5 kHz, dwn3_byp = 0, ndec_exp = 3 , filset = 3
   0x34, //  12.1 kHz, dwn3_byp = 0, ndec_exp = 3 , filset = 4
   0x35, //  14.2 kHz, dwn3_byp = 0, ndec_exp = 3 , filset = 5
   0x36, //  16.2 kHz, dwn3_byp = 0, ndec_exp = 3 , filset = 6
   0x37, //  17.5 kHz, dwn3_byp = 0, ndec_exp = 3 , filset = 7
   0x21, //  18.9 kHz, dwn3_byp = 0, ndec_exp = 2 , filset = 1
   0x22, //  21.0 kHz, dwn3_byp = 0, ndec_exp = 2 , filset = 2
   0x23, //  22.7 kHz, dwn3_byp = 0, ndec_exp = 2 , filset = 3
   0x24, //  24.0 kHz, dwn3_byp = 0, ndec_exp = 2 , filset = 4
   0x25, //  28.2 kHz, dwn3_byp = 0, ndec_exp = 2 , filset = 5
   0x26, //  32.2 kHz, dwn3_byp = 0, ndec_exp = 2 , filset = 6
   0x27, //  34.7 kHz, dwn3_byp = 0, ndec_exp = 2 , filset = 7
   0x11, //  37.7 kHz, dwn3_byp = 0, ndec_exp = 1 , filset = 1
   0x12, //  41.7 kHz, dwn3_byp = 0, ndec_exp = 1 , filset = 2
   0x13, //  45.2 kHz, dwn3_byp = 0, ndec_exp = 1 , filset = 3
   0x14, //  47.9 kHz, dwn3_byp = 0, ndec_exp = 1 , filset = 4
   0x15, //  56.2 kHz, dwn3_byp = 0, ndec_exp = 1 , filset = 5
   0x16, //  64.1 kHz, dwn3_byp = 0, ndec_exp = 1 , filset = 6
   0x17, //  69.2 kHz, dwn3_byp = 0, ndec_exp = 1 , filset = 7
   0x01, //  75.2 kHz, dwn3_byp = 0, ndec_exp = 0 , filset = 1
   0x02, //  83.2 kHz, dwn3_byp = 0, ndec_exp = 0 , filset = 2
   0x03, //  90.0 kHz, dwn3_byp = 0, ndec_exp = 0 , filset = 3
   0x04, //  95.3 kHz, dwn3_byp = 0, ndec_exp = 0 , filset = 4
   0x05, // 112.1 kHz, dwn3_byp = 0, ndec_exp = 0 , filset = 5
   0x06, // 127.9 kHz, dwn3_byp = 0, ndec_exp = 0 , filset = 6
   0x07, // 137.9 kHz, dwn3_byp = 0, ndec_exp = 0 , filset = 7
   0x94, // 142.8 kHz, dwn3_byp = 1, ndec_exp = 1 , filset = 4
   0x95, // 167.8 kHz, dwn3_byp = 1, ndec_exp = 1 , filset = 5
   0x99, // 181.1 kHz, dwn3_byp = 1, ndec_exp = 1 , filset = 9
   0x8F, // 191.5 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 15
   0x81, // 225.1 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 1
   0x82, // 248.8 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 2
   0x83, // 269.3 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 3
   0x84, // 284.9 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 4
   0x88, // 335.5 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 8
   0x89, // 361.8 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 9
   0x8A, // 420.2 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 10
   0x8B, // 468.4 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 11
   0x8C, // 518.8 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 12
   0x8D, // 577.0 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 13
   0x8E, // 620.7 kHz, dwn3_byp = 1, ndec_exp = 0 , filset = 14
};
//=================================================================================================
//=================================================================================================

