//-----------------------------------------------------------------------------
// Si1000_defs.h
//-----------------------------------------------------------------------------
// Copyright 2010 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// Register/bit definitions for the Si100x family.
//
//
// Target:         Si100x
// Tool chain:     Keil, SDCC
// Command Line:   None
//
//
// Release 1.0 
//    -EZRadioPRO definitions created by Ken Berrenger.
//    -26 JAN 2010  (FB)
//

//-----------------------------------------------------------------------------
// Header File Preprocessor Directive
//-----------------------------------------------------------------------------

#ifndef SI1000_DEFS_H
#define SI1000_DEFS_H

#include "cdt.h"
#include "compiler_defs.h"

//-----------------------------------------------------------------------------
// Byte Registers
//-----------------------------------------------------------------------------

SFR (P0, 0x80);                        // Port 0 Latch
SFR (SP, 0x81);                        // Stack Pointer
SFR (DPL, 0x82);                       // Data Pointer Low
SFR (DPH, 0x83);                       // Data Pointer High
SFR (SPI1CFG, 0x84);                   // SPI1 Configuration
SFR (SPI1CKR, 0x85);                   // SPI1 Clock Rate Control
SFR (TOFFL, 0x85);                     // Temperature Offset Low
SFR (SPI1DAT, 0x86);                   // SPI1 Data
SFR (TOFFH, 0x86);                     // Temperature Offset High
SFR (PCON, 0x87);                      // Power Control
SFR (TCON, 0x88);                      // Timer/Counter Control
SFR (TMOD, 0x89);                      // Timer/Counter Mode
SFR (TL0, 0x8A);                       // Timer/Counter 0 Low
SFR (TL1, 0x8B);                       // Timer/Counter 1 Low
SFR (TH0, 0x8C);                       // Timer/Counter 0 High
SFR (TH1, 0x8D);                       // Timer/Counter 1 High
SFR (CKCON, 0x8E);                     // Clock Control
SFR (PSCTL, 0x8F);                     // Program Store R/W Control
SFR (P1, 0x90);                        // Port 1 Latch
SFR (TMR3CN, 0x91);                    // Timer/Counter 3 Control
SFR (CRC0DAT, 0x91);                   // CRC0 Data
SFR (TMR3RLL, 0x92);                   // Timer/Counter 3 Reload Low
SFR (CRC0CN, 0x92);                    // CRC0 Control
SFR (TMR3RLH, 0x93);                   // Timer/Counter 3 Reload High
SFR (CRC0IN, 0x93);                    // CRC0 Input
SFR (TMR3L, 0x94);                     // Timer/Counter 3 Low
SFR (CRC0FLIP, 0x95);                  // CRC0 Flip
SFR (TMR3H, 0x95);                     // Timer/Counter 3 High
SFR (DC0CF, 0x96);                     // DC0 (DC/DC Converter) Configuration
SFR (CRC0AUTO, 0x96);                  // CRC0 Automatic Control
SFR (DC0CN, 0x97);                     // DC0 (DC/DC Converter) Control
SFR (CRC0CNT, 0x97);                   // CRC0 Automatic Flash Sector Count
SFR (SCON0, 0x98);                     // UART0 Control
SFR (SBUF0, 0x99);                     // UART0 Data Buffer
SFR (CPT1CN, 0x9A);                    // Comparator1 Control
SFR (CPT0CN, 0x9B);                    // Comparator0 Control
SFR (CPT1MD, 0x9C);                    // Comparator1 Mode Selection
SFR (CPT0MD, 0x9D);                    // Comparator0 Mode Selection
SFR (CPT1MX, 0x9E);                    // Comparator1 Mux Selection
SFR (CPT0MX, 0x9F);                    // Comparator0 Mux Selection
SFR (P2, 0xA0);                        // Port 2 Latch
SFR (SPI0CFG, 0xA1);                   // SPI0 Configuration
SFR (SPI0CKR, 0xA2);                   // SPI0 Clock Rate Control
SFR (SPI0DAT, 0xA3);                   // SPI0 Data
SFR (P0MDOUT, 0xA4);                   // Port 0 Output Mode Configuration
SFR (P0DRV, 0xA4);                     // Port 0 Drive Strength
SFR (P1MDOUT, 0xA5);                   // Port 1 Output Mode Configuration
SFR (P1DRV, 0xA5);                     // Port 1 Drive Strength
SFR (P2MDOUT, 0xA6);                   // Port 2 Output Mode Configuration
SFR (P2DRV, 0xA6);                     // Port 2 Drive Strength
SFR (SFRPAGE, 0xA7);                   // SFR Page
SFR (IE, 0xA8);                        // Interrupt Enable
SFR (CLKSEL, 0xA9);                    // Clock Select
SFR (EMI0CN, 0xAA);                    // EMIF Control
SFR (EMI0CF, 0xAB);                    // EMIF Configuration
SFR (RTC0ADR, 0xAC);                   // RTC0 Address
SFR (RTC0DAT, 0xAD);                   // RTC0 Data
SFR (RTC0KEY, 0xAE);                   // RTC0 Key
SFR (EMI0TC, 0xAF);                    // EMIF Timing Control
SFR (ONESHOT, 0xAF);                   // ONESHOT Timing Control
SFR (SPI1CN, 0xB0);                    // SPI1 Control
SFR (OSCXCN, 0xB1);                    // External Oscillator Control
SFR (OSCICN, 0xB2);                    // Internal Oscillator Control
SFR (OSCICL, 0xB3);                    // Internal Oscillator Calibration
SFR (PMU0CF, 0xB5);                    // PMU0 Configuration
SFR (FLSCL, 0xB6);                     // Flash Scale Register
SFR (FLKEY, 0xB7);                     // Flash Lock And Key
SFR (IP, 0xB8);                        // Interrupt Priority
SFR (IREF0CN, 0xB9);                   // Current Reference IREF0 Control
SFR (ADC0AC, 0xBA);                    // ADC0 Accumulator Configuration
SFR (ADC0PWR, 0xBA);                   // ADC0 Burst Mode Power-Up Time
SFR (ADC0MX, 0xBB);                    // AMUX0 Channel Select
SFR (ADC0CF, 0xBC);                    // ADC0 Configuration
SFR (ADC0TK, 0xBD);                    // ADC0 Tracking Control
SFR (ADC0L, 0xBD);                     // ADC0 Low
SFR (ADC0H, 0xBE);                     // ADC0 High
SFR (P1MASK, 0xBF);                    // Port 1 Mask
SFR (SMB0CN, 0xC0);                    // SMBus0 Control
SFR (SMB0CF, 0xC1);                    // SMBus0 Configuration
SFR (SMB0DAT, 0xC2);                   // SMBus0 Data
SFR (ADC0GTL, 0xC3);                   // ADC0 Greater-Than Compare Low
SFR (ADC0GTH, 0xC4);                   // ADC0 Greater-Than Compare High
SFR (ADC0LTL, 0xC5);                   // ADC0 Less-Than Compare Word Low
SFR (ADC0LTH, 0xC6);                   // ADC0 Less-Than Compare Word High
SFR (P0MASK, 0xC7);                    // Port 0 Mask
SFR (TMR2CN, 0xC8);                    // Timer/Counter 2 Control
SFR (REG0CN, 0xC9);                    // Voltage Regulator (REG0) Control
SFR (TMR2RLL, 0xCA);                   // Timer/Counter 2 Reload Low
SFR (TMR2RLH, 0xCB);                   // Timer/Counter 2 Reload High
SFR (TMR2L, 0xCC);                     // Timer/Counter 2 Low
SFR (TMR2H, 0xCD);                     // Timer/Counter 2 High
SFR (PCA0CPM5, 0xCE);                  // PCA0 Module 5 Mode Register
SFR (P1MAT, 0xCF);                     // Port 1 Match
SFR (PSW, 0xD0);                       // Program Status Word
SFR (REF0CN, 0xD1);                    // Voltage Reference Control
SFR (PCA0CPL5, 0xD2);                  // PCA0 Capture 5 Low
SFR (PCA0CPH5, 0xD3);                  // PCA0 Capture 5 High
SFR (P0SKIP, 0xD4);                    // Port 0 Skip
SFR (P1SKIP, 0xD5);                    // Port 1 Skip
SFR (P2SKIP, 0xD6);                    // Port 2 Skip
SFR (P0MAT, 0xD7);                     // Port 0 Match
SFR (PCA0CN, 0xD8);                    // PCA0 Control
SFR (PCA0MD, 0xD9);                    // PCA0 Mode
SFR (PCA0CPM0, 0xDA);                  // PCA0 Module 0 Mode Register
SFR (PCA0CPM1, 0xDB);                  // PCA0 Module 1 Mode Register
SFR (PCA0CPM2, 0xDC);                  // PCA0 Module 2 Mode Register
SFR (PCA0CPM3, 0xDD);                  // PCA0 Module 3 Mode Register
SFR (PCA0CPM4, 0xDE);                  // PCA0 Module 4 Mode Register
SFR (PCA0PWM, 0xDF);                   // PCA0 PWM Configuration
SFR (ACC, 0xE0);                       // Accumulator
SFR (XBR0, 0xE1);                      // Port I/O Crossbar Control 0
SFR (XBR1, 0xE2);                      // Port I/O Crossbar Control 1
SFR (XBR2, 0xE3);                      // Port I/O Crossbar Control 2
SFR (IT01CF, 0xE4);                    // INT0/INT1 Configuration
SFR (FLWR, 0xE5);                      // Flash Write Only Register
SFR (EIE1, 0xE6);                      // Extended Interrupt Enable 1
SFR (EIE2, 0xE7);                      // Extended Interrupt Enable 2
SFR (ADC0CN, 0xE8);                    // ADC0 Control
SFR (PCA0CPL1, 0xE9);                  // PCA0 Capture 1 Low
SFR (PCA0CPH1, 0xEA);                  // PCA0 Capture 1 High
SFR (PCA0CPL2, 0xEB);                  // PCA0 Capture 2 Low
SFR (PCA0CPH2, 0xEC);                  // PCA0 Capture 2 High
SFR (PCA0CPL3, 0xED);                  // PCA0 Capture 3 Low
SFR (PCA0CPH3, 0xEE);                  // PCA0 Capture 3 High
SFR (RSTSRC, 0xEF);                    // Reset Source Configuration/Status
SFR (B, 0xF0);                         // B Register
SFR (P0MDIN, 0xF1);                    // Port 0 Input Mode Configuration
SFR (P1MDIN, 0xF2);                    // Port 1 Input Mode Configuration
SFR (P2MDIN, 0xF3);                    // Port 2 Input Mode Configuration
SFR (SMB0ADR, 0xF4);                   // SMBus Slave Address
SFR (SMB0ADM, 0xF5);                   // SMBus Slave Address Mask
SFR (EIP1, 0xF6);                      // Extended Interrupt Priority 1
SFR (EIP2, 0xF7);                      // Extended Interrupt Priority 2
SFR (SPI0CN, 0xF8);                    // SPI0 Control
SFR (PCA0L, 0xF9);                     // PCA0 Counter Low
SFR (PCA0H, 0xFA);                     // PCA0 Counter High
SFR (PCA0CPL0, 0xFB);                  // PCA0 Capture 0 Low
SFR (PCA0CPH0, 0xFC);                  // PCA0 Capture 0 High
SFR (PCA0CPL4, 0xFD);                  // PCA0 Capture 4 Low
SFR (PCA0CPH4, 0xFE);                  // PCA0 Capture 4 High
SFR (VDM0CN, 0xFF);                    // VDD Monitor Control



//-----------------------------------------------------------------------------
// 16-bit Register Definitions (might not be supported by all compilers)
//-----------------------------------------------------------------------------

SFR16 (DP, 0x82);                      // Data Pointer
SFR16 (TOFF, 0x85);                    // Temperature Sensor Offset
SFR16 (TMR3RL, 0x92);                  // Timer 3 Reload
SFR16 (TMR3, 0x94);                    // Timer 3 Counter
SFR16 (ADC0, 0xBD);                    // ADC0 Data
SFR16 (ADC0GT, 0xC3);                  // ADC0 Greater-Than Compare
SFR16 (ADC0LT, 0xC5);                  // ADC0 Less-Than Compare
SFR16 (TMR2RL, 0xCA);                  // Timer 2 Reload
SFR16 (TMR2, 0xCC);                    // Timer 2 Counter
SFR16 (PCA0CP5, 0xD2);                 // PCA0 Module 5 Capture/Compare
SFR16 (PCA0CP1, 0xE9);                 // PCA0 Module 1 Capture/Compare
SFR16 (PCA0CP2, 0xEB);                 // PCA0 Module 2 Capture/Compare
SFR16 (PCA0CP3, 0xED);                 // PCA0 Module 3 Capture/Compare
SFR16 (PCA0, 0xF9);                    // PCA0 Counter
SFR16 (PCA0CP0, 0xFB);                 // PCA0 Module 0 Capture/Compare
SFR16 (PCA0CP4, 0xFD);                 // PCA0 Module 4 Capture/Compare


//-----------------------------------------------------------------------------
// Indirect RTC Register Addresses
//-----------------------------------------------------------------------------

#define CAPTURE0  0x00                 // RTC address of CAPTURE0 register
#define CAPTURE1  0x01                 // RTC address of CAPTURE1 register
#define CAPTURE2  0x02                 // RTC address of CAPTURE2 register
#define CAPTURE3  0x03                 // RTC address of CAPTURE3 register
#define RTC0CN    0x04                 // RTC address of RTC0CN register                
#define RTC0XCN   0x05                 // RTC address of RTC0XCN register 
#define RTC0XCF   0x06                 // RTC address of RTC0XCF register
#define RTC0PIN   0x07                 // RTC address of RTC0PIN register
#define ALARM0    0x08                 // RTC address of ALARM0 register
#define ALARM1    0x09                 // RTC address of ALARM1 register
#define ALARM2    0x0A                 // RTC address of ALARM2 register
#define ALARM3    0x0B                 // RTC address of ALARM3 register

//-----------------------------------------------------------------------------
// Address Definitions for Bit-addressable Registers
//-----------------------------------------------------------------------------

#define SFR_P0       0x80
#define SFR_TCON     0x88
#define SFR_P1       0x90
#define SFR_CRC0CN   0x92
#define SFR_SCON0    0x98
#define SFR_P2       0xA0
#define SFR_IE       0xA8
#define SFR_SPI1CN   0xB0
#define SFR_IP       0xB8
#define SFR_SMB0CN   0xC0
#define SFR_TMR2CN   0xC8
#define SFR_PSW      0xD0
#define SFR_PCA0CN   0xD8
#define SFR_ACC      0xE0
#define SFR_ADC0CN   0xE8
#define SFR_B        0xF0
#define SFR_SPI0CN   0xF8

//-----------------------------------------------------------------------------
// Bit Definitions
//-----------------------------------------------------------------------------

// TCON 0x88
SBIT (TF1, SFR_TCON, 7);               // Timer 1 Overflow Flag
SBIT (TR1, SFR_TCON, 6);               // Timer 1 On/Off Control
SBIT (TF0, SFR_TCON, 5);               // Timer 0 Overflow Flag
SBIT (TR0, SFR_TCON, 4);               // Timer 0 On/Off Control
SBIT (IE1, SFR_TCON, 3);               // Ext. Interrupt 1 Edge Flag
SBIT (IT1, SFR_TCON, 2);               // Ext. Interrupt 1 Type
SBIT (IE0, SFR_TCON, 1);               // Ext. Interrupt 0 Edge Flag
SBIT (IT0, SFR_TCON, 0);               // Ext. Interrupt 0 Type

// CRC0CN 0x92
SBIT (CRC0SEL,  SFR_CRC0CN, 4);
SBIT (CRC0INIT, SFR_CRC0CN, 3);
SBIT (CRC0VAL,  SFR_CRC0CN, 2);

// SCON0 0x98
SBIT (S0MODE, SFR_SCON0, 7);           // UART0 Mode
                                       // Bit6 UNUSED
SBIT (MCE0, SFR_SCON0, 5);             // UART0 MCE
SBIT (REN0, SFR_SCON0, 4);             // UART0 RX Enable
SBIT (TB80, SFR_SCON0, 3);             // UART0 TX Bit 8
SBIT (RB80, SFR_SCON0, 2);             // UART0 RX Bit 8
SBIT (TI0, SFR_SCON0, 1);              // UART0 TX Interrupt Flag
SBIT (RI0, SFR_SCON0, 0);              // UART0 RX Interrupt Flag

// IE 0xA8
SBIT (EA, SFR_IE, 7);                  // Global Interrupt Enable
SBIT (ESPI0, SFR_IE, 6);               // SPI0 Interrupt Enable
SBIT (ET2, SFR_IE, 5);                 // Timer 2 Interrupt Enable
SBIT (ES0, SFR_IE, 4);                 // UART0 Interrupt Enable
SBIT (ET1, SFR_IE, 3);                 // Timer 1 Interrupt Enable
SBIT (EX1, SFR_IE, 2);                 // External Interrupt 1 Enable
SBIT (ET0, SFR_IE, 1);                 // Timer 0 Interrupt Enable
SBIT (EX0, SFR_IE, 0);                 // External Interrupt 0 Enable

// SPI1CN 0xB0
SBIT (SPIF1, SFR_SPI1CN, 7);           // SPI1 Interrupt Flag
SBIT (WCOL1, SFR_SPI1CN, 6);           // SPI1 Write Collision Flag
SBIT (MODF1, SFR_SPI1CN, 5);           // SPI1 Mode Fault Flag
SBIT (RXOVRN1, SFR_SPI1CN, 4);         // SPI1 RX Overrun Flag
SBIT (NSS1MD1, SFR_SPI1CN, 3);         // SPI1 Slave Select Mode 1
SBIT (NSS1MD0, SFR_SPI1CN, 2);         // SPI1 Slave Select Mode 0
SBIT (TXBMT1, SFR_SPI1CN, 1);          // SPI1 TX Buffer Empty Flag
SBIT (SPI1EN, SFR_SPI1CN, 0);          // SPI1 Enable

// IP 0xB8
                                       // Bit7 UNUSED
SBIT (PSPI0, SFR_IP, 6);               // SPI0 Priority
SBIT (PT2, SFR_IP, 5);                 // Timer 2 Priority
SBIT (PS0, SFR_IP, 4);                 // UART0 Priority
SBIT (PT1, SFR_IP, 3);                 // Timer 1 Priority
SBIT (PX1, SFR_IP, 2);                 // External Interrupt 1 Priority
SBIT (PT0, SFR_IP, 1);                 // Timer 0 Priority
SBIT (PX0, SFR_IP, 0);                 // External Interrupt 0 Priority

// SMB0CN 0xC0
SBIT (MASTER, SFR_SMB0CN, 7);          // SMBus0 Master/Slave
SBIT (TXMODE, SFR_SMB0CN, 6);          // SMBus0 Transmit Mode
SBIT (STA, SFR_SMB0CN, 5);             // SMBus0 Start Flag
SBIT (STO, SFR_SMB0CN, 4);             // SMBus0 Stop Flag
SBIT (ACKRQ, SFR_SMB0CN, 3);           // SMBus0 Acknowledge Request
SBIT (ARBLOST, SFR_SMB0CN, 2);         // SMBus0 Arbitration Lost
SBIT (ACK, SFR_SMB0CN, 1);             // SMBus0 Acknowledge Flag
SBIT (SI, SFR_SMB0CN, 0);              // SMBus0 Interrupt Pending Flag

// TMR2CN 0xC8
SBIT (TF2H, SFR_TMR2CN, 7);            // Timer 2 High Byte Overflow Flag
SBIT (TF2L, SFR_TMR2CN, 6);            // Timer 2 Low Byte Overflow Flag
SBIT (TF2LEN, SFR_TMR2CN, 5);          // Timer 2 Low Byte Interrupt Enable
SBIT (TF2CEN, SFR_TMR2CN, 4);          // Timer 2 Lfo Capture Enable
SBIT (T2SPLIT, SFR_TMR2CN, 3);         // Timer 2 Split Mode Enable
SBIT (TR2, SFR_TMR2CN, 2);             // Timer 2 On/Off Control
SBIT (T2RCLK, SFR_TMR2CN, 1);          // Timer 2 Capture Mode
SBIT (T2XCLK, SFR_TMR2CN, 0);          // Timer 2 External Clock Select

// PSW 0xD0
SBIT (CY, SFR_PSW, 7);                 // Carry Flag
SBIT (AC, SFR_PSW, 6);                 // Auxiliary Carry Flag
SBIT (F0, SFR_PSW, 5);                 // User Flag 0
SBIT (RS1, SFR_PSW, 4);                // Register Bank Select 1
SBIT (RS0, SFR_PSW, 3);                // Register Bank Select 0
SBIT (OV, SFR_PSW, 2);                 // Overflow Flag
SBIT (F1, SFR_PSW, 1);                 // User Flag 1
SBIT (P, SFR_PSW, 0);                  // Accumulator Parity Flag

// PCA0CN 0xD8
SBIT (CF, SFR_PCA0CN, 7);              // PCA0 Counter Overflow Flag
SBIT (CR, SFR_PCA0CN, 6);              // PCA0 Counter Run Control Bit
SBIT (CCF5, SFR_PCA0CN, 5);            // PCA0 Module 5 Interrupt Flag
SBIT (CCF4, SFR_PCA0CN, 4);            // PCA0 Module 4 Interrupt Flag
SBIT (CCF3, SFR_PCA0CN, 3);            // PCA0 Module 3 Interrupt Flag
SBIT (CCF2, SFR_PCA0CN, 2);            // PCA0 Module 2 Interrupt Flag
SBIT (CCF1, SFR_PCA0CN, 1);            // PCA0 Module 1 Interrupt Flag
SBIT (CCF0, SFR_PCA0CN, 0);            // PCA0 Module 0 Interrupt Flag

// ADC0CN 0xE8
SBIT (AD0EN, SFR_ADC0CN, 7);           // ADC0 Enable
SBIT (BURSTEN, SFR_ADC0CN, 6);         // ADC0 Burst Enable
SBIT (AD0INT, SFR_ADC0CN, 5);          // ADC0 EOC Interrupt Flag
SBIT (AD0BUSY, SFR_ADC0CN, 4);         // ADC0 Busy Flag
SBIT (AD0WINT, SFR_ADC0CN, 3);         // ADC0 Window Interrupt Flag
SBIT (AD0CM2, SFR_ADC0CN, 2);          // ADC0 Convert Start Mode Bit 2
SBIT (AD0CM1, SFR_ADC0CN, 1);          // ADC0 Convert Start Mode Bit 1
SBIT (AD0CM0, SFR_ADC0CN, 0);          // ADC0 Convert Start Mode Bit 0

// SPI0CN 0xF8
SBIT (SPIF0, SFR_SPI0CN, 7);           // SPI0 Interrupt Flag
SBIT (WCOL0, SFR_SPI0CN, 6);           // SPI0 Write Collision Flag
SBIT (MODF0, SFR_SPI0CN, 5);           // SPI0 Mode Fault Flag
SBIT (RXOVRN0, SFR_SPI0CN, 4);         // SPI0 RX Overrun Flag
SBIT (NSS0MD1, SFR_SPI0CN, 3);         // SPI0 Slave Select Mode 1
SBIT (NSS0MD0, SFR_SPI0CN, 2);         // SPI0 Slave Select Mode 0
SBIT (TXBMT0, SFR_SPI0CN, 1);          // SPI0 TX Buffer Empty Flag
SBIT (SPI0EN, SFR_SPI0CN, 0);          // SPI0 Enable

//-----------------------------------------------------------------------------
// Interrupt Priorities
//-----------------------------------------------------------------------------

#define INTERRUPT_INT0             0   // External Interrupt 0
#define INTERRUPT_TIMER0           1   // Timer0 Overflow
#define INTERRUPT_INT1             2   // External Interrupt 1
#define INTERRUPT_TIMER1           3   // Timer1 Overflow
#define INTERRUPT_UART0            4   // Serial Port 0
#define INTERRUPT_TIMER2           5   // Timer2 Overflow
#define INTERRUPT_SPI0             6   // Serial Peripheral Interface 0
#define INTERRUPT_SMBUS0           7   // SMBus0 Interface
#define INTERRUPT_RTC0ALARM        8   // RTC0 (SmaRTClock) Alarm
#define INTERRUPT_ADC0_WINDOW      9   // ADC0 Window Comparison
#define INTERRUPT_ADC0_EOC         10  // ADC0 End Of Conversion
#define INTERRUPT_PCA0             11  // PCA0 Peripheral
#define INTERRUPT_COMPARATOR0      12  // Comparator0
#define INTERRUPT_COMPARATOR1      13  // Comparator1
#define INTERRUPT_TIMER3           14  // Timer3 Overflow
#define INTERRUPT_VDDMON           15  // VDD Monitor Early Warning
#define INTERRUPT_PORT_MATCH       16  // Port Match
#define INTERRUPT_RTC0_OSC_FAIL    17  // RTC0 (smaRTClock) Osc. Fail
#define INTERRUPT_SPI1             18  // Serial Peripheral Interface 1

//-----------------------------------------------------------------------------
// SFR Page Definitions
//-----------------------------------------------------------------------------
#define CONFIG_PAGE       0x0F         // SYSTEM AND PORT CONFIGURATION PAGE
#define LEGACY_PAGE       0x00         // LEGACY SFR PAGE
#define CRC0_PAGE         0x0F         // CRC0
#define TOFF_PAGE         0x0F         // TEMPERATURE SENSOR OFFSET PAGE

//=============================================================================
//
// EZRadioPRO Register Defines
//
// The register names are defined exactly as listed in the Function/
// Description field of the Si1000 data sheet. Upper Case is used for
// constants. Spaces and miscellaneous characters are replaced with
// underscores. EZRADIOPRO_ prefix added for Si1000.
//
//=============================================================================
#define  EZRADIOPRO_DEVICE_TYPE                                0x00
#define  EZRADIOPRO_DEVICE_VERSION                             0x01
#define  EZRADIOPRO_DEVICE_STATUS                              0x02
#define  EZRADIOPRO_INTERRUPT_STATUS_1                         0x03
#define  EZRADIOPRO_INTERRUPT_STATUS_2                         0x04
#define  EZRADIOPRO_INTERRUPT_ENABLE_1                         0x05
#define  EZRADIOPRO_INTERRUPT_ENABLE_2                         0x06
#define  EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1           0x07
#define  EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2           0x08
#define  EZRADIOPRO_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE        0x09
#define  EZRADIOPRO_MICROCONTROLLER_OUTPUT_CLOCK               0x0A
#define  EZRADIOPRO_GPIO0_CONFIGURATION                        0x0B
#define  EZRADIOPRO_GPIO1_CONFIGURATION                        0x0C
#define  EZRADIOPRO_GPIO2_CONFIGURATION                        0x0D
#define  EZRADIOPRO_IO_PORT_CONFIGURATION                      0x0E
#define  EZRADIOPRO_ADC_CONFIGURATION                          0x0F
#define  EZRADIOPRO_ADC_SENSOR_AMPLIFIER_OFFSET                0x10
#define  EZRADIOPRO_ADC_VALUE                                  0x11
#define  EZRADIOPRO_TEMPERATURE_SENSOR_CONTROL                 0x12
#define  EZRADIOPRO_TEMPERATURE_VALUE_OFFSET                   0x13
#define  EZRADIOPRO_WAKE_UP_TIMER_PERIOD_1                     0x14
#define  EZRADIOPRO_WAKE_UP_TIMER_PERIOD_2                     0x15
#define  EZRADIOPRO_WAKE_UP_TIMER_PERIOD_3                     0x16
#define  EZRADIOPRO_WAKE_UP_TIMER_VALUE_1                      0x17
#define  EZRADIOPRO_WAKE_UP_TIMER_VALUE_2                      0x18
#define  EZRADIOPRO_LOW_DUTY_CYCLE_MODE_DURATION               0x19
#define  EZRADIOPRO_LOW_BATTERY_DETECTOR_THRESHOLD             0x1A
#define  EZRADIOPRO_BATTERY_VOLTAGE_LEVEL                      0x1B
#define  EZRADIOPRO_IF_FILTER_BANDWIDTH                        0x1C
#define  EZRADIOPRO_AFC_LOOP_GEARSHIFT_OVERRIDE                0x1D
#define  EZRADIOPRO_AFC_TIMING_CONTROL                         0x1E
#define  EZRADIOPRO_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE          0x1F
#define  EZRADIOPRO_CLOCK_RECOVERY_OVERSAMPLING_RATIO          0x20
#define  EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2                    0x21
#define  EZRADIOPRO_CLOCK_RECOVERY_OFFSET_1                    0x22
#define  EZRADIOPRO_CLOCK_RECOVERY_OFFSET_0                    0x23
#define  EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_1          0x24
#define  EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_0          0x25
#define  EZRADIOPRO_RECEIVED_SIGNAL_STRENGTH_INDICATOR         0x26
#define  EZRADIOPRO_RSSI_THRESHOLD                             0x27
#define  EZRADIOPRO_ANTENNA_DIVERSITY_REGISTER_1               0x28
#define  EZRADIOPRO_ANTENNA_DIVERSITY_REGISTER_2               0x29
#define  EZRADIOPRO_DATA_ACCESS_CONTROL                        0x30
#define  EZRADIOPRO_EZMAC_STATUS                               0x31
#define  EZRADIOPRO_HEADER_CONTROL_1                           0x32
#define  EZRADIOPRO_HEADER_CONTROL_2                           0x33
#define  EZRADIOPRO_PREAMBLE_LENGTH                            0x34
#define  EZRADIOPRO_PREAMBLE_DETECTION_CONTROL                 0x35
#define  EZRADIOPRO_SYNC_WORD_3                                0x36
#define  EZRADIOPRO_SYNC_WORD_2                                0x37
#define  EZRADIOPRO_SYNC_WORD_1                                0x38
#define  EZRADIOPRO_SYNC_WORD_0                                0x39
#define  EZRADIOPRO_TRANSMIT_HEADER_3                          0x3A
#define  EZRADIOPRO_TRANSMIT_HEADER_2                          0x3B
#define  EZRADIOPRO_TRANSMIT_HEADER_1                          0x3C
#define  EZRADIOPRO_TRANSMIT_HEADER_0                          0x3D
#define  EZRADIOPRO_TRANSMIT_PACKET_LENGTH                     0x3E
#define  EZRADIOPRO_CHECK_HEADER_3                             0x3F
#define  EZRADIOPRO_CHECK_HEADER_2                             0x40
#define  EZRADIOPRO_CHECK_HEADER_1                             0x41
#define  EZRADIOPRO_CHECK_HEADER_0                             0x42
#define  EZRADIOPRO_HEADER_ENABLE_3                            0x43
#define  EZRADIOPRO_HEADER_ENABLE_2                            0x44
#define  EZRADIOPRO_HEADER_ENABLE_1                            0x45
#define  EZRADIOPRO_HEADER_ENABLE_0                            0x46
#define  EZRADIOPRO_RECEIVED_HEADER_3                          0x47
#define  EZRADIOPRO_RECEIVED_HEADER_2                          0x48
#define  EZRADIOPRO_RECEIVED_HEADER_1                          0x49
#define  EZRADIOPRO_RECEIVED_HEADER_0                          0x4A
#define  EZRADIOPRO_RECEIVED_PACKET_LENGTH                     0x4B
#define  EZRADIOPRO_ANALOG_TEST_BUS                            0x50
#define  EZRADIOPRO_DIGITAL_TEST_BUS                           0x51
#define  EZRADIOPRO_TX_RAMP_CONTROL                            0x52
#define  EZRADIOPRO_PLL_TUNE_TIME                              0x53
#define  EZRADIOPRO_CALIBRATION_CONTROL                        0x55
#define  EZRADIOPRO_MODEM_TEST                                 0x56
#define  EZRADIOPRO_CHARGEPUMP_TEST                            0x57
#define  EZRADIOPRO_CHARGEPUMP_CURRENT_TRIMMING_OVERRIDE       0x58
#define  EZRADIOPRO_DIVIDER_CURRENT_TRIMMING                   0x59
#define  EZRADIOPRO_VCO_CURRENT_TRIMMING                       0x5A
#define  EZRADIOPRO_VCO_CALIBRATION_OVERRIDE                   0x5B
#define  EZRADIOPRO_SYNTHESIZER_TEST                           0x5C
#define  EZRADIOPRO_BLOCK_ENABLE_OVERRIDE_1                    0x5D
#define  EZRADIOPRO_BLOCK_ENABLE_OVERRIDE_2                    0x5E
#define  EZRADIOPRO_BLOCK_ENABLE_OVERRIDE_3                    0x5F
#define  EZRADIOPRO_CHANNEL_FILTER_COEFFICIENT_ADDRESS         0x60
#define  EZRADIOPRO_CHANNEL_FILTER_COEFFICIENT_VALUE           0x61
#define  EZRADIOPRO_CRYSTAL_OSCILLATOR_CONTROL_TEST            0x62
#define  EZRADIOPRO_RC_OSCILLATOR_COARSE_CALIBRATION_OVERRIDE  0x63
#define  EZRADIOPRO_RC_OSCILLATOR_FINE_CALIBRATION_OVERRIDE    0x64
#define  EZRADIOPRO_LDO_CONTROL_OVERRIDE                       0x65
#define  EZRADIOPRO_LDO_LEVEL_SETTING                          0x66
#define  EZRADIOPRO_DELTASIGMA_ADC_TUNING_1                    0x67
#define  EZRADIOPRO_DELTASIGMA_ADC_TUNING_2                    0x68
#define  EZRADIOPRO_AGC_OVERRIDE_1                             0x69
#define  EZRADIOPRO_AGC_OVERRIDE_2                             0x6A
#define  EZRADIOPRO_GFSK_FIR_FILTER_COEFFICIENT_ADDRESS        0x6B
#define  EZRADIOPRO_GFSK_FIR_FILTER_COEFFICIENT_VALUE          0x6C
#define  EZRADIOPRO_TX_POWER                                   0x6D
#define  EZRADIOPRO_TX_DATA_RATE_1                             0x6E
#define  EZRADIOPRO_TX_DATA_RATE_0                             0x6F
#define  EZRADIOPRO_MODULATION_MODE_CONTROL_1                  0x70
#define  EZRADIOPRO_MODULATION_MODE_CONTROL_2                  0x71
#define  EZRADIOPRO_FREQUENCY_DEVIATION                        0x72
#define  EZRADIOPRO_FREQUENCY_OFFSET_1                         0x73
#define  EZRADIOPRO_FREQUENCY_OFFSET_2                         0x74
#define  EZRADIOPRO_FREQUENCY_BAND_SELECT                      0x75
#define  EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_1                0x76
#define  EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_0                0x77
#define  EZRADIOPRO_FREQUENCY_HOPPING_CHANNEL_SELECT           0x79
#define  EZRADIOPRO_FREQUENCY_HOPPING_STEP_SIZE                0x7A
#define  EZRADIOPRO_TX_FIFO_CONTROL_1                          0x7C
#define  EZRADIOPRO_TX_FIFO_CONTROL_2                          0x7D
#define  EZRADIOPRO_RX_FIFO_CONTROL                            0x7E
#define  EZRADIOPRO_FIFO_ACCESS                                0x7F
//------------------------------------------------------------------------------------------------
// new registers for B1 radio
#define  EZRADIOPRO_AFC_LIMITER                                0x2A
#define  EZRADIOPRO_AFC_CORRECTION                             0x2B
#define  EZRADIOPRO_OOK_COUNTER_VALUE_1                        0x2C
#define  EZRADIOPRO_OOK_COUNTER_VALUE_2                        0X2D
#define  EZRADIOPRO_SLICER_PEAK_HOLD                           0X2E
#define  EZRADIOPRO_ADC8_CONTROL                               0x4F
#define  EZRADIOPRO_INVALID_PREAMBLE_THRESHOLD_AND_PA_MISC     0x54
#define  EZRADIOPRO_MISCELLANEOUS_SETTINGS                     0x78
#define  EZRADIOPRO_TURN_AROUND_AND_15_4_LENGTH_COMPLIANCE     0x7B
//=============================================================================
//
// Register Bit Masks
//
//=============================================================================
// EZRADIOPRO_DEVICE_TYPE                                      0x00
#define  EZRADIOPRO_DT_MASK            0x1F
// EZRADIOPRO_DEVICE_VERSION                                   0x01
#define  EZRADIOPRO_VC_MASK            0x1F
// EZRADIOPRO_DEVICE_STATUS                                    0x02
#define  EZRADIOPRO_CPS_MASK           0x03
#define  EZRADIOPRO_LOCKDET            0x04
#define  EZRADIOPRO_FREQERR            0x08
#define  EZRADIOPRO_HEADERR            0x10
#define  EZRADIOPRO_RXFFEM             0x20
#define  EZRADIOPRO_FFUNFL             0x40
#define  EZRADIOPRO_FFOVFL             0x80

// EZRADIOPRO_INTERRUPT_STATUS_1                               0x03
#define  EZRADIOPRO_ICRCERROR          0x01
#define  EZRADIOPRO_IPKVALID           0x02
#define  EZRADIOPRO_IPKSENT            0x04
#define  EZRADIOPRO_IEXT               0x08
#define  EZRADIOPRO_IRXFFAFULL         0x10
#define  EZRADIOPRO_ITXFFAEM           0x20
#define  EZRADIOPRO_ITXFFAFULL         0x40
#define  EZRADIOPRO_IFFERR             0x80

// EZRADIOPRO_INTERRUPT_STATUS_2                               0x04
#define  EZRADIOPRO_IPOR               0x01
#define  EZRADIOPRO_ICHIPRDY           0x02
#define  EZRADIOPRO_ILBD               0x04
#define  EZRADIOPRO_IWUT               0x08
#define  EZRADIOPRO_IRSSI              0x10
#define  EZRADIOPRO_IPREAINVAL         0x20
#define  EZRADIOPRO_IPREAVAL           0x40
#define  EZRADIOPRO_ISWDET             0x80

// EZRADIOPRO_INTERRUPT_ENABLE_1                               0x05
#define  EZRADIOPRO_ENCRCERROR         0x01
#define  EZRADIOPRO_ENPKVALID          0x02
#define  EZRADIOPRO_ENPKSENT           0x04
#define  EZRADIOPRO_ENEXT              0x08
#define  EZRADIOPRO_ENRXFFAFULL        0x10
#define  EZRADIOPRO_ENTXFFAEM          0x20
#define  EZRADIOPRO_ENTXFFAFULL        0x40
#define  EZRADIOPRO_ENFFERR            0x80

// EZRADIOPRO_INTERRUPT_ENABLE_2                               0x06
#define  EZRADIOPRO_ENPOR              0x01
#define  EZRADIOPRO_ENCHIPRDY          0x02
#define  EZRADIOPRO_ENLBDI             0x04 // added I to make unique
#define  EZRADIOPRO_ENWUT              0x08
#define  EZRADIOPRO_ENRSSI             0x10
#define  EZRADIOPRO_ENPREAINVAL        0x20
#define  EZRADIOPRO_ENPREAVAL          0x40
#define  EZRADIOPRO_ENSWDET            0x80

// EZRADIOPRO_OPERATING_FUNCTION_CONTROL_1                     0x07
#define  EZRADIOPRO_XTON               0x01
#define  EZRADIOPRO_PLLON              0x02
#define  EZRADIOPRO_RXON               0x04
#define  EZRADIOPRO_TXON               0x08
#define  EZRADIOPRO_X32KSEL            0x10
#define  EZRADIOPRO_ENWT               0x20
#define  EZRADIOPRO_ENLBD              0x40
#define  EZRADIOPRO_SWRES              0x80

// EZRADIOPRO_OPERATING_FUNCTION_CONTROL_2                     0x08
#define  EZRADIOPRO_FFCLRTX            0x01
#define  EZRADIOPRO_FFCLRRX            0x02
#define  EZRADIOPRO_ENLDM              0x04
#define  EZRADIOPRO_AUTOTX             0x08
#define  EZRADIOPRO_RXMPK              0x10
#define  EZRADIOPRO_ANTDIV_MASK        0xE0

// EZRADIOPRO_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE              0x09
#define  EZRADIOPRO_XLC_MASK           0x7F
#define  EZRADIOPRO_XTALSHFT           0x80

// EZRADIOPRO_MICROCONTROLLER_OUTPUT_CLOCK                     0x0A
#define  EZRADIOPRO_MCLK_MASK          0x07
#define  EZRADIOPRO_ENLFC              0x08
#define  EZRADIOPRO_CLKT_MASK          0x30

// EZRADIOPRO_GPIO0_CONFIGURATION                              0x0B
#define  EZRADIOPRO_GPIO0_MASK         0x1F
#define  EZRADIOPRO_PUP0               0x20
#define  EZRADIOPRO_GPIO0DRV_MASK      0xC0

// EZRADIOPRO_GPIO1_CONFIGURATION                              0x0C
#define  EZRADIOPRO_GPIO1_MASK         0x1F
#define  EZRADIOPRO_PUP1               0x20
#define  EZRADIOPRO_GPIO1DRV_MASK      0xC0

// EZRADIOPRO_GPIO2_CONFIGURATION                              0x0D
#define  EZRADIOPRO_GPIO2_MASK         0x1F
#define  EZRADIOPRO_PUP2               0x20
#define  EZRADIOPRO_GPIO2DRV_MASK      0xC0

// EZRADIOPRO_IO_PORT_CONFIGURATION                            0x0E
#define  EZRADIOPRO_DIO_MASK           0x07
#define  EZRADIOPRO_ITSDO              0x08
#define  EZRADIOPRO_EXTITST_MASK       0x70

// EZRADIOPRO_ADC_CONFIGURATION                                0x0F
#define  EZRADIOPRO_ADCGAIN_MASK        0x03
#define  EZRADIOPRO_ADCREF_MASK         0x0C
#define  EZRADIOPRO_ADCSEL_MASK         0x70
#define  EZRADIOPRO_ADCSTART            0x80  //W
#define  EZRADIOPRO_ADCDONE             0x80  //R

// EZRADIOPRO_ADC_SENSOR_AMPLIFIER_OFFSET                      0x10
#define  EZRADIOPRO_ADCOFFS_MASK       0x0F

// EZRADIOPRO_ADC_VALUE                                        0x11
// no bits or mask

// EZRADIOPRO_TEMPERATURE_SENSOR_CONTROL                       0x12
#define  EZRADIOPRO_TSTRIM_MASK         0x0F
#define  EZRADIOPRO_ENTSTRIM            0x10
#define  EZRADIOPRO_ENTSOFFS            0x20
#define  EZRADIOPRO_TSRANGE_MASK        0xC0

// EZRADIOPRO_TEMPERATURE_VALUE_OFFSET                         0x13
// no bits or mask

// EZRADIOPRO_WAKE_UP_TIMER_PERIOD_1                           0x14
#define  EZRADIOPRO_WTD_MASK           0x03
#define  EZRADIOPRO_WTR_MASK           0x3C

// EZRADIOPRO_WAKE_UP_TIMER_PERIOD_2                           0x15
// no bits or mask

// EZRADIOPRO_WAKE_UP_TIMER_PERIOD_3                           0x16
#define  EZRADIOPRO_WTM_MASK           0x80

// EZRADIOPRO_WAKE_UP_TIMER_VALUE_1                            0x17
// no bits or mask

// EZRADIOPRO_WAKE_UP_TIMER_VALUE_2                            0x18
// no bits or mask

// EZRADIOPRO_LOW_DUTY_CYCLE_MODE_DURATION                     0x19
// no bits or mask

// EZRADIOPRO_LOW_BATTERY_DETECTOR_THRESHOLD                   0x1A
#define  EZRADIOPRO_LBDT_MASK          0x1F

// EZRADIOPRO_BATTERY_VOLTAGE_LEVEL                            0x1B
#define  EZRADIOPRO_VBAT_MASK          0x1F

// EZRADIOPRO_I_F_FILTER_BANDWIDTH                             0x1C
#define  EZRADIOPRO_FILSET_MASK        0x0F
#define  EZRADIOPRO_NDEC_MASK          0x70
#define  EZRADIOPRO_DWN3_BYPASS        0x80

// EZRADIOPRO_A_F_C_LOOP_GEARSHIFT_OVERRIDE                    0x1D
#define  EZRADIOPRO_AFCGEARH_MASK      0x3F
#define  EZRADIOPRO_ENAFC              0x40
#define  EZRADIOPRO_AFCBD              0x80

// EZRADIOPRO_A_F_C_TIMING_CONTROL                             0x1E
#define  EZRADIOPRO_LGWAIT_MASK        0x07
#define  EZRADIOPRO_SHWAIT_MASK        0x38

// EZRADIOPRO_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE                0x1F
#define  EZRADIOPRO_CRSLOW_MASK        0x07
#define  EZRADIOPRO_CRFAST_MASK        0x38
#define  EZRADIOPRO_RXREADY            0x40

// EZRADIOPRO_CLOCK_RECOVERY_OVERSAMPLING_RATIO                0x20
// no bits or mask

// EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2 0x21
#define  EZRADIOPRO_NCOFF_MASK         0x0F
#define  EZRADIOPRO_STALLCTRL          0x10
#define  EZRADIOPRO_RXOSR_MASK         0xE0

// EZRADIOPRO_CLOCK_RECOVERY_OFFSET_1                          0x22
// no bits or mask

// EZRADIOPRO_CLOCK_RECOVERY_OFFSET_0                          0x23
// no bits or mask

// EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_1                0x24
#define  EZRADIOPRO_CRGAIN_MASK        0x07

// EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_0                0x25
// no bits or mask

// EZRADIOPRO_RECEIVED_SIGNAL_STRENGTH_INDICATOR               0x26
// no bits or mask

// EZRADIOPRO_RSSI_THRESHOLD                                   0x27
// no bits or mask

// EZRADIOPRO_ANTENNA_DIVERSITY_REGISTER_1                     0x28
// no bits or mask

// EZRADIOPRO_ANTENNA_DIVERSITY_REGISTER_2                     0x29
// no bits or mask

// EZRADIOPRO_DATA_ACCESS_CONTROL                              0x30
#define  EZRADIOPRO_CRC_MASK           0x03
#define  EZRADIOPRO_CRC_16             0x01
#define  EZRADIOPRO_ENCRC              0x04
#define  EZRADIOPRO_ENPACTX            0x08
#define  EZRADIOPRO_CRCDONLY           0x20
#define  EZRADIOPRO_LSBFRST            0x40
#define  EZRADIOPRO_ENPACRX            0x80

// EZRADIOPRO_EZ_MAC_STATUS                                    0x31
#define  EZRADIOPRO_PKSENT             0x01
#define  EZRADIOPRO_PKTX               0x02
#define  EZRADIOPRO_CRCERROR           0x04
#define  EZRADIOPRO_PKVALID            0x08
#define  EZRADIOPRO_PKRX               0x10
#define  EZRADIOPRO_PKSRCH             0x20
#define  EZRADIOPRO_RXCRC1             0x40

// EZRADIOPRO_HEADER_CONTROL_1                                 0x32
#define  EZRADIOPRO_HDCH_MASK          0x0F
#define  EZRADIOPRO_BCEN_MASK          0xF0
#define  EZRADIOPRO_BCEN               0xF0
#define  EZRADIOPRO_DISABLE_HFILTERS   0x00

// EZRADIOPRO_HEADER_CONTROL_2                                 0x33
#define  EZRADIOPRO_PREALEN_MASK       0x01
#define  EZRADIOPRO_SYNCLEN_MASK       0x06
#define  EZRADIOPRO_FIXPKLEN           0x08
#define  EZRADIOPRO_HDLEN_MASK         0x70
#define  EZRADIOPRO_SYNCLEN_1BYTE      0x00
#define  EZRADIOPRO_SYNCLEN_2BYTE      0x02
#define  EZRADIOPRO_SYNCLEN_3BYTE      0x04
#define  EZRADIOPRO_SYNCLEN_4BYTE      0x06
#define  EZRADIOPRO_HDLEN_0BYTE        0x00
#define  EZRADIOPRO_HDLEN_1BYTE        0x10
#define  EZRADIOPRO_HDLEN_2BYTE        0x20
#define  EZRADIOPRO_HDLEN_3BYTE        0x30
#define  EZRADIOPRO_HDLEN_4BYTE        0x40
#define  EZRADIOPRO_SKIPSYN            0x80

// EZRADIOPRO_PREAMBLE_LENGTH                                  0x34
// no bits or mask

// EZRADIOPRO_SYNC_WORD_3                                      0x36
// no bits or mask

// EZRADIOPRO_SYNC_WORD_2                                      0x37
// no bits or mask

// EZRADIOPRO_SYNC_WORD_1                                      0x38
// no bits or mask

// EZRADIOPRO_SYNC_WORD_0                                      0x39
// no bits or mask

// EZRADIOPRO_TRANSMIT_HEADER_3                                0x3A
// no bits or mask

// EZRADIOPRO_TRANSMIT_HEADER_2                                0x3B
// no bits or mask

// EZRADIOPRO_TRANSMIT_HEADER_1                                0x3C
// no bits or mask

// EZRADIOPRO_TRANSMIT_HEADER_0                                0x3D
// no bits or mask

// EZRADIOPRO_TRANSMIT_PACKET_LENGTH                           0x3E
// no bits or mask

// EZRADIOPRO_CHECK_HEADER_3                                   0x3F
// no bits or mask

// EZRADIOPRO_CHECK_HEADER_2                                   0x40
// no bits or mask

// EZRADIOPRO_CHECK_HEADER_1                                   0x41
// no bits or mask

// EZRADIOPRO_CHECK_HEADER_0                                   0x42
// no bits or mask

// EZRADIOPRO_HEADER_ENABLE_3                                  0x43
// no bits or mask

// EZRADIOPRO_HEADER_ENABLE_2                                  0x44
// no bits or mask

// EZRADIOPRO_HEADER_ENABLE_1                                  0x45
// no bits or mask

// EZRADIOPRO_HEADER_ENABLE_0                                  0x46
// no bits or mask

// EZRADIOPRO_RECEIVED_HEADER_3                                0x47
// no bits or mask

// EZRADIOPRO_RECEIVED_HEADER_2                                0x48
// no bits or mask

// EZRADIOPRO_RECEIVED_HEADER_1                                0x49
// no bits or mask

// EZRADIOPRO_RECEIVED_HEADER_0                                0x4A
// no bits or mask

// EZRADIOPRO_RECEIVED_PACKET_LENGTH                           0x4B
// no bits or mask

// EZRADIOPRO_ANALOG_TEST_BUS                                  0x50
#define  EZRADIOPRO_ATB_MASK           0x1F

// EZRADIOPRO_DIGITAL_TEST_BUS                                 0x51
#define  EZRADIOPRO_DTB_MASK           0x2F
#define  EZRADIOPRO_ENSCTEST           0x40

// EZRADIOPRO_TX_RAMP_CONTROL                                  0x52
#define  EZRADIOPRO_TXRAMP_MASK        0x03
#define  EZRADIOPRO_LDORAMP_MASK       0x0C
#define  EZRADIOPRO_TXMOD_MASK         0x70

// EZRADIOPRO_PLL_TUNE_TIME                                    0x53
#define  EZRADIOPRO_PLLT0_MASK         0x07
#define  EZRADIOPRO_PLLTS_MASK         0xF8

// EZRADIOPRO_CALIBRATION_CONTROL                              0x55
#define  EZRADIOPRO_SKIPVCO            0x01
#define  EZRADIOPRO_VCOCAL             0x02
#define  EZRADIOPRO_VCOCALDP           0x04
#define  EZRADIOPRO_RCCAL              0x08
#define  EZRADIOPRO_ENRCFCAL           0x10
#define  EZRADIOPRO_ADCCALDONE         0x20
#define  EZRADIOPRO_XTALSTARTHALF      0x40

// EZRADIOPRO_MODEM_TEST                                       0x56
#define  EZRADIOPRO_IQSWITCH           0x01
#define  EZRADIOPRO_REFCLKINV          0x02
#define  EZRADIOPRO_REFCLKSEL          0x04
#define  EZRADIOPRO_AFCPOL             0x10
#define  EZRADIOPRO_DTTYPE             0x20
#define  EZRADIOPRO_SLICFBYP           0x40
#define  EZRADIOPRO_BCRFBYP            0x80

// EZRADIOPRO_CHARGEPUMP_TEST                                  0x57
#define  EZRADIOPRO_CDCCUR_MASK        0x07
#define  EZRADIOPRO_CDCONLY            0x08
#define  EZRADIOPRO_CPFORCEDN          0x10
#define  EZRADIOPRO_CPFORCEUP          0x20
#define  EZRADIOPRO_FBDIV_RST          0x40
#define  EZRADIOPRO_PFDRST             0x80

// EZRADIOPRO_CHARGEPUMP_CURRENT_TRIMMING_OVERRIDE             0x58
#define  EZRADIOPRO_CPCORR_MASK        0x1F
#define  EZRADIOPRO_CPCORROV           0x20
#define  EZRADIOPRO_CPCURR_MASK        0xC0

// EZRADIOPRO_DIVIDER_CURRENT_TRIMMING                         0x59
#define  EZRADIOPRO_D1P5TRIM_MASK      0x03
#define  EZRADIOPRO_D2TRIM_MASK        0x0C
#define  EZRADIOPRO_D3TRIM_MASK        0x30
#define  EZRADIOPRO_FBDIVHC            0x40
#define  EZRADIOPRO_TXCORBOOSTEN       0x80

// EZRADIOPRO_VCO_CURRENT_TRIMMING                             0x5A
#define  EZRADIOPRO_VCOCUR_MASK        0x03
#define  EZRADIOPRO_VCOCORR_MASK       0x3C
#define  EZRADIOPRO_VCOCORROV          0x40
#define  EZRADIOPRO_TXCURBOOSTEN       0x80

// EZRADIOPRO_VCO_CALIBRATION_OVERRIDE                         0x5B
#define  EZRADIOPRO_VCOCAL_MASK        0x7F
#define  EZRADIOPRO_VCOCALOV           0x80  //W
#define  EZRADIOPRO_VCDONE             0x80  //R

// EZRADIOPRO_SYNTHESIZER_TEST                                 0x5C
#define  EZRADIOPRO_DSRST               0x01
#define  EZRADIOPRO_DSRSTMOD            0x02
#define  EZRADIOPRO_DSORDER_MASK        0x0C
#define  EZRADIOPRO_DSMOD               0x10
#define  EZRADIOPRO_ENOLOOP             0x20
#define  EZRADIOPRO_VCOTYPE             0x40
#define  EZRADIOPRO_DSMDT               0x80

// EZRADIOPRO_BLOCK_ENABLE_OVERRIDE_1                          0x5D
#define  EZRADIOPRO_ENMX2              0x01
#define  EZRADIOPRO_ENBF12             0x02
#define  EZRADIOPRO_ENDV32             0x04
#define  EZRADIOPRO_ENBF5              0x08
#define  EZRADIOPRO_ENPA               0x10
#define  EZRADIOPRO_ENPGA              0x20
#define  EZRADIOPRO_ENLNA              0x40
#define  EZRADIOPRO_ENMIX              0x80

// EZRADIOPRO_BLOCK_ENABLE_OVERRIDE_2                          0x5E
#define  EZRADIOPRO_PLLRESET           0x01
#define  EZRADIOPRO_ENBF2              0x02
#define  EZRADIOPRO_ENBF11             0x04
#define  EZRADIOPRO_ENBF3              0x08
#define  EZRADIOPRO_ENBF4              0x10
#define  EZRADIOPRO_ENMX3              0x20
#define  EZRADIOPRO_ENLDET             0x40
#define  EZRADIOPRO_ENDS               0x80

// EZRADIOPRO_BLOCK_ENABLE_OVERRIDE_3                          0x5F
#define  EZRADIOPRO_ENBG               0x01
#define  EZRADIOPRO_ENCP               0x02
#define  EZRADIOPRO_ENVCO              0x04
#define  EZRADIOPRO_DVBSHUNT           0x08
#define  EZRADIOPRO_ENDV1P5            0x10
#define  EZRADIOPRO_ENDV2              0x20
#define  EZRADIOPRO_ENDV31             0x40
#define  EZRADIOPRO_ENFRDV             0x80

// EZRADIOPRO_CHANNEL_FILTER_COEFFICIENT_ADDRESS               0x60
#define  EZRADIOPRO_CHFILADD_MASK      0x0F

// EZRADIOPRO_CHANNEL_FILTER_COEFFICIENT_VALUE                 0x61
#define  EZRADIOPRO_CHFILVAL_MASK      0x3F

// EZRADIOPRO_CRYSTAL_OSCILLATOR_CONTROL_TEST                  0x62
#define  EZRADIOPRO_ENBUF              0x01
#define  EZRADIOPRO_BUFOVR             0x02
#define  EZRADIOPRO_ENAMP2X            0x04
#define  EZRADIOPRO_ENBIAS2X           0x08
#define  EZRADIOPRO_CLKHYST            0x10
#define  EZRADIOPRO_PWST_MASK          0xE0

// EZRADIOPRO_RC_OSCILLATOR_COARSE_CALIBRATION_OVERRIDE        0x63
#define  EZRADIOPRO_RCC_MASK           0x7F
#define  EZRADIOPRO_RCCOV              0x80

// EZRADIOPRO_RC_OSCILLATOR_FINE_CALIBRATION_OVERRIDE          0x64
#define  EZRADIOPRO_RCF_MASK           0x7F
#define  EZRADIOPRO_RCFOV              0x80

// EZRADIOPRO_LDO_CONTROL_OVERRIDE   0x65
#define  EZRADIOPRO_ENDIGPWDN          0x01
#define  EZRADIOPRO_ENDIGLDO           0x02
#define  EZRADIOPRO_ENPLLLDO           0x04
#define  EZRADIOPRO_ENRFLDO            0x08
#define  EZRADIOPRO_ENIFLDO            0x10
#define  EZRADIOPRO_ENVCOLDO           0x20
#define  EZRADIOPRO_ENBIAS             0x40
#define  EZRADIOPRO_ENSPOR             0x80

// EZRADIOPRO_LDO_LEVEL_SETTING   0x66
#define  EZRADIOPRO_DIGLVL_MASK        0x07
#define  EZRADIOPRO_ENRC32             0x10
#define  EZRADIOPRO_ENTS               0x20
#define  EZRADIOPRO_ENXTAL             0x40
#define  EZRADIOPRO_ENOVR              0x80

// EZRADIOPRO_DELTASIGMA_ADC_TUNING_1                          0x67
#define  EZRADIOPRO_ADCTUNE_MASK       0x0F
#define  EZRADIOPRO_ADCTUNEOVR         0x10
#define  EZRADIOPRO_ENADC              0x20
#define  EZRADIOPRO_ENREFDAC           0x40
#define  EZRADIOPRO_ADCRST             0x80

// EZRADIOPRO_DELTASIGMA_ADC_TUNING_2                          0x68
#define  EZRADIOPRO_DSADCREF_MASK      0x07 // added DS to make unique
#define  EZRADIOPRO_ADCOLOOP           0x08
#define  EZRADIOPRO_ENVCM              0x10

// EZRADIOPRO_AGC_OVERRIDE_1                                   0x69
#define  EZRADIOPRO_PGA_MASK           0x0F
#define  EZRADIOPRO_LNAGAIN            0x10
#define  EZRADIOPRO_AGCEN              0x20

// EZRADIOPRO_AGC_OVERRIDE_2   0x6A
#define  EZRADIOPRO_PGATH_MASK         0x03
#define  EZRADIOPRO_LNACOMP_MASK       0x3F
#define  EZRADIOPRO_AGCSLOW            0x40
#define  EZRADIOPRO_AGCOVPM            0x80

// EZRADIOPRO_GFSK_FIR_FILTER_COEFFICIENT_ADDRESS              0x6B
#define  EZRADIOPRO_FIRADD_MASK        0x07

// EZRADIOPRO_GFSK_FIR_FILTER_COEFFICIENT_VALUE                0x6C
#define  EZRADIOPRO_FIRVAL_MASK        0x3F

// EZRADIOPRO_TX_POWER   0x6D
#define  EZRADIOPRO_TXPOW_MASK         0x07

// EZRADIOPRO_TX_DATA_RATE_1                                   0x6E
// no bits or mask

// EZRADIOPRO_TX_DATA_RATE_0                                   0x6F
// no bits or mask

// EZRADIOPRO_MODULATION_MODE_CONTROL_1                        0x70
#define  EZRADIOPRO_ENWHITE            0x01
#define  EZRADIOPRO_ENMANCH            0x02
#define  EZRADIOPRO_ENMANINV           0x04
#define  EZRADIOPRO_MANPPOL            0x08
#define  EZRADIOPRO_ENPHPWDN           0x10
#define  EZRADIOPRO_TXDTRTSCALE        0x20

// EZRADIOPRO_MODULATION_MODE_CONTROL_2                        0x71
#define  EZRADIOPRO_MODTYP_MASK        0x03
#define  EZRADIOPRO_FD_MASK            0x04
#define  EZRADIOPRO_ENINV              0x08
#define  EZRADIOPRO_DTMOD_MASK         0x30
#define  EZRADIOPRO_TRCLK_MASK         0xC0
#define  EZRADIOPRO_MODTYP_GFSK        0x03
#define  EZRADIOPRO_FIFO_MODE          0x20
#define  EZRADIOPRO_TX_DATA_CLK_GPIO   0x40


// EZRADIOPRO_FREQUENCY_DEVIATION                              0x72
// no bits or mask

// EZRADIOPRO_FREQUENCY_OFFSET_1                               0x73
// no bits or mask

// EZRADIOPRO_FREQUENCY_OFFSET_2                               0x74
#define  EZRADIOPRO_FO_MASK            0x03

// EZRADIOPRO_FREQUENCY_BAND_SELECTRESERVED                    0x75
#define  EZRADIOPRO_FB_MASK            0x1F
#define  EZRADIOPRO_HBSEL              0x20
#define  EZRADIOPRO_SBSEL              0x40

// EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_1                      0x76
// no bits or mask

// EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_0                      0x77
// no bits or mask

// EZRADIOPRO_FREQUENCY_HOPPING_CHANNEL_SELECT                 0x79
// no bits or mask

// EZRADIOPRO_FREQUENCY_HOPPING_STEP_SIZE                      0x7A
// no bits or mask

// EZRADIOPRO_TX_FIFO_CONTROL_1                                0x7C
#define  EZRADIOPRO_TXAFTHR_MASK       0x3F

// EZRADIOPRO_TX_FIFO_CONTROL_2                                0x7D
#define  EZRADIOPRO_TXAETHR_MASK       0x3F

// EZRADIOPRO_RX_FIFO_CONTROL                                  0x7E
#define  EZRADIOPRO_RXAFTHR_MASK       0x3F
#define  EZRADIOPRO_RESERVED           0x40

// EZRADIOPRO_FIFO_ACCESS                                      0x7F
// no bits or mask

//-----------------------------------------------------------------------------
// Header File PreProcessor Directive
//-----------------------------------------------------------------------------

#endif                                 // #define SI1000_DEFS_H

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
