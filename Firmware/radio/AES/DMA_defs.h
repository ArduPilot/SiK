//-----------------------------------------------------------------------------
// DMA_defs.h
//-----------------------------------------------------------------------------
// Copyright 2011 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// DMA definitions for Si102x/3x family.
//
// Target:         Si102x/3x
// Tool chain:     Generic
// Command Line:   None
//
//-----------------------------------------------------------------------------
// Include compiler_defs.h if not already defined.
//-----------------------------------------------------------------------------
#ifndef COMPILER_DEFS_H
#include <compiler_defs.h>
#endif
//-----------------------------------------------------------------------------
// Header file applied only if not already defined.
//-----------------------------------------------------------------------------
#ifndef DMA_DEFS_H
#define DMA_DEFS_H
//-----------------------------------------------------------------------------
// DMA transfers use Idle mode
//-----------------------------------------------------------------------------
//#define DMA_TRANSFERS_USE_IDLE
//=============================================================================
// Static DMA Channel Allocations (Static)
//
// These defines are used for a Static DMA allocation. The DMA channels are
// assigned for a specific purpose.
//
// These settings reuse the AES DMA channels for the encoder/decoder.
// So these operations cannot be done simultaneously.
//
//=============================================================================
#define  SPI1_IN_CHANNEL   0x0
#define  SPI1_OUT_CHANNEL  0x1
#define  CRC1_IN_CHANNEL   0x2
#define  ENC0_IN_CHANNEL   0x3
#define  ENC0_OUT_CHANNEL  0x4
#define  AES0KIN_CHANNEL   0x3
#define  AES0BIN_CHANNEL   0x4
#define  AES0XIN_CHANNEL   0x5
#define  AES0YOUT_CHANNEL  0x6

//=============================================================================
// DMA Peripheral Requests
//
// IN/OUT defined from the peripheral's perspective.
//
// IN    =  XRAM -> SFR
// OUT   =  SFR -> XRAM
//
// SPI1 Master mode
// SPI1_IN  =  XRAM -> SFR = SPI Write = MOSI data
// SPI1_OUT =  SFR -> XRAM = SPI Read  = MISO data
//
// SPI1 Slave mode
// SPI1_IN  =  XRAM -> SFR = SPI Write = MISO data
// SPI1_OUT =  SFR -> XRAM = SPI Read  = MOSI data
//
//-----------------------------------------------------------------------------
enum PERIPHERAL_REQUEST_Enum
{
   ENC0_IN_PERIPHERAL_REQUEST = 0,     // 0x0
   ENC0_OUT_PERIPHERAL_REQUEST,        // 0x1
   CRC1_PERIPHERAL_REQUEST,            // 0x2
   SPI1_IN_PERIPHERAL_REQUEST,         // 0x3
   SPI1_OUT_PERIPHERAL_REQUEST,        // 0x4
   AES0KIN_PERIPHERAL_REQUEST,         // 0x5
   AES0BIN_PERIPHERAL_REQUEST,         // 0x6
   AES0XIN_PERIPHERAL_REQUEST,         // 0x7
   AES0YOUT_PERIPHERAL_REQUEST         // 0x8
};
//-----------------------------------------------------------------------------
// defines used with DMA0NCF sfr
//-----------------------------------------------------------------------------
#define  DMA_BIG_ENDIAN    0x10
#define  DMA_INT_EN        0x80
//-----------------------------------------------------------------------------
// defines used with DMA0NMD sfr
//-----------------------------------------------------------------------------
#define  WRAPPING          0x1
#define  NO_WRAPPING       0x0
//-----------------------------------------------------------------------------
// DMA Bits
//
// Enable/Disable and Interrupt bits based on above static allocations.
//
//-----------------------------------------------------------------------------
#define  ENC0_IN_MASK      (1<<ENC0_IN_CHANNEL)
#define  ENC0_OUT_MASK     (1<<ENC0_OUT_CHANNEL)
#define  ENC0_MASK         (ENC0_IN_MASK|ENC0_OUT_MASK)
#define  CRC1_IN_MASK      (1<<CRC1_IN_CHANNEL)
#define  SPI1_IN_MASK      (1<<SPI1_IN_CHANNEL)
#define  SPI1_OUT_MASK     (1<<SPI1_OUT_CHANNEL)
#define  SPI1_MASK         (SPI1_IN_MASK|SPI1_OUT_MASK)
#define  AES0KIN_MASK      (1<<AES0KIN_CHANNEL)
#define  AES0BIN_MASK      (1<<AES0BIN_CHANNEL)
#define  AES0XIN_MASK      (1<<AES0XIN_CHANNEL)
#define  AES0YOUT_MASK     (1<<AES0YOUT_CHANNEL)
#define  AES0_KBXY_MASK    (AES0KIN_MASK|AES0BIN_MASK|AES0XIN_MASK|AES0YOUT_MASK)
#define  AES0_KBY_MASK     (AES0KIN_MASK|AES0BIN_MASK|AES0YOUT_MASK)
//-----------------------------------------------------------------------------
// DMA transfer Sizes
//-----------------------------------------------------------------------------
#define  MANCHESTER_ENC_IN_SIZE        0x1
#define  MANCHESTER_ENC_OUT_SIZE       0x2
#define  MANCHESTER_DEC_IN_SIZE        0x2
#define  MANCHESTER_DEC_OUT_SIZE       0x1
#define  THREEOUTOFSIX_ENC_IN_SIZE     0x2
#define  THREEOUTOFSIX_ENC_OUT_SIZE    0x3
#define  THREEOUTOFSIX_DEC_IN_SIZE     0x3
#define  THREEOUTOFSIX_DEC_OUT_SIZE    0x2
#define  CRC1_IN_SIZE                  0x1
#define  SPI1_IN_SIZE                  0x1
#define  SPI1_OUT_SIZE                 0x1
#define  AESK_IN_SIZE                  0x1
#define  AESB_IN_SIZE                  0x1
#define  AESX_IN_SIZE                  0x1
#define  AESY_OUT_SIZE                 0x1
//-----------------------------------------------------------------------------
// End DMA_defs.h
//-----------------------------------------------------------------------------
#endif                                 // DMA_defs.h