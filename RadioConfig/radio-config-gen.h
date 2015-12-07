// Copyright 2015 Silicon Laboratories, Inc.
//
//

// Modem configuration parameters
// version: 1.3.0.94
// afc_en: 0
// ant_div: 0
// BER_mode: 0
// Ch_Fil_Bw_AFC: 0
// Chip_Version: 2
// crystal_tol: 20
// dsa_mode: 0
// dsource: 0
// ETSI: 0
// fc: 915000000
// Fdev: 96000
// Fdev_error: 0
// fhch: 10
// fhst: 250000
// freq_xo: 30000000
// hi_pfm_div_mode: 0
// High_perf_Ch_Fil: 1
// if_mode: 2
// inputBW: 0
// Manchester: 0
// Max_Rb_Error: 0
// modulation_type: 3
// OSRtune: 0
// pm_len: 16
// pm_pattern: 0
// raw_demod: 0
// Rsymb: 64000
// RXBW: 150000
// TC: 29

// Modem configuration calculated values
// if_freq: -468750.0
// nb_filter: 1
// nb_filter_bw: 305.23333333333335
// wb_filter: 1
// wb_filter_bw: 305.23333333333335

// This file contains the generated ObjectAPI byte arrays for the given configuration
// Enclosing macro to prevent multiple inclusion
#ifndef __SLAB_RADIO_CONFIG__
#define __SLAB_RADIO_CONFIG__


#define RADIO_CONFIG_POWER_UP { \
  0x02 /* CMD: Power up                                               */,\
  0x01 /* BOOT_OPTIONS,PATCH[7],FUNC[5:0]                             */,\
  0x00 /* XTAL_OPTIONS,TCXO[0]                                        */,\
  0x01 /* XO_FREQ,XO_FREQ[7:0],XO_FREQ[7:0],XO_FREQ[7:0],XO_FREQ[7:0] */,\
  0xC9 /*                                                             */,\
  0xC3 /*                                                             */,\
  0x80 /*                                                             */\
}

#define RADIO_CONFIG_POWER_UP_LEN (7)


#define RADIO_CONFIG_GPIO_PIN_CFG { \
  0x13 /* CMD: Gpio pin cfg                */,\
  0x03 /* GPIO0,PULL_CTL[6],GPIO_MODE[5:0] */,\
  0x02 /* GPIO1,PULL_CTL[6],GPIO_MODE[5:0] */,\
  0x20 /* GPIO2,PULL_CTL[6],GPIO_MODE[5:0] */,\
  0x21 /* GPIO3,PULL_CTL[6],GPIO_MODE[5:0] */,\
  0x00 /* NIRQ,PULL_CTL[6],NIRQ_MODE[5:0]  */,\
  0x00 /* SDO,PULL_CTL[6],SDO_MODE[5:0]    */,\
  0x00 /* GEN_CONFIG,DRV_STRENGTH[6:5]     */\
}

#define RADIO_CONFIG_GPIO_PIN_CFG_LEN (8)


#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_XO_TUNE { \
  0x11 /* CMD: Set property              */,\
  0x00 /* GROUP: Global                  */,\
  0x01 /* NUM_PROPS                      */,\
  0x00 /* START_PROP                     */,\
  0x45 /* GLOBAL_XO_TUNE,TUNE_VALUE[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_XO_TUNE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_A_MODE { \
  0x11 /* CMD: Set property              */,\
  0x02 /* GROUP: Frr ctl                 */,\
  0x01 /* NUM_PROPS                      */,\
  0x00 /* START_PROP                     */,\
  0x01 /* FRR_CTL_A_MODE,FRR_A_MODE[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_A_MODE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_B_MODE { \
  0x11 /* CMD: Set property              */,\
  0x02 /* GROUP: Frr ctl                 */,\
  0x01 /* NUM_PROPS                      */,\
  0x01 /* START_PROP                     */,\
  0x02 /* FRR_CTL_B_MODE,FRR_B_MODE[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_B_MODE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_C_MODE { \
  0x11 /* CMD: Set property              */,\
  0x02 /* GROUP: Frr ctl                 */,\
  0x01 /* NUM_PROPS                      */,\
  0x02 /* START_PROP                     */,\
  0x09 /* FRR_CTL_C_MODE,FRR_C_MODE[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_C_MODE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_D_MODE { \
  0x11 /* CMD: Set property              */,\
  0x02 /* GROUP: Frr ctl                 */,\
  0x01 /* NUM_PROPS                      */,\
  0x03 /* START_PROP                     */,\
  0x00 /* FRR_CTL_D_MODE,FRR_D_MODE[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_FRR_CTL_D_MODE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_ENABLE { \
  0x11 /* CMD: Set property                                                               */,\
  0x01 /* GROUP: Int ctl                                                                  */,\
  0x01 /* NUM_PROPS                                                                       */,\
  0x00 /* START_PROP                                                                      */,\
  0x07 /* INT_CTL_ENABLE,CHIP_INT_STATUS_EN[2],MODEM_INT_STATUS_EN[1],PH_INT_STATUS_EN[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_ENABLE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_PH_ENABLE { \
  0x11 /* CMD: Set property                                                                                                                                                                 */,\
  0x01 /* GROUP: Int ctl                                                                                                                                                                    */,\
  0x01 /* NUM_PROPS                                                                                                                                                                         */,\
  0x01 /* START_PROP                                                                                                                                                                        */,\
  0x3B /* INT_CTL_PH_ENABLE,FILTER_MATCH_EN[7],FILTER_MISS_EN[6],PACKET_SENT_EN[5],PACKET_RX_EN[4],CRC_ERROR_EN[3],ALT_CRC_ERROR_EN[2],TX_FIFO_ALMOST_EMPTY_EN[1],RX_FIFO_ALMOST_FULL_EN[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_PH_ENABLE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_MODEM_ENABLE { \
  0x11 /* CMD: Set property                                                                                                                                                         */,\
  0x01 /* GROUP: Int ctl                                                                                                                                                            */,\
  0x01 /* NUM_PROPS                                                                                                                                                                 */,\
  0x02 /* START_PROP                                                                                                                                                                */,\
  0x06 /* INT_CTL_MODEM_ENABLE,RSSI_LATCH_EN[7],POSTAMBLE_DETECT_EN[6],INVALID_SYNC_EN[5],RSSI_JUMP_EN[4],RSSI_EN[3],INVALID_PREAMBLE_EN[2],PREAMBLE_DETECT_EN[1],SYNC_DETECT_EN[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_MODEM_ENABLE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_CHIP_ENABLE { \
  0x11 /* CMD: Set property                                                                                                                              */,\
  0x01 /* GROUP: Int ctl                                                                                                                                 */,\
  0x01 /* NUM_PROPS                                                                                                                                      */,\
  0x03 /* START_PROP                                                                                                                                     */,\
  0x20 /* INT_CTL_CHIP_ENABLE,CAL_EN[6],FIFO_UNDERFLOW_OVERFLOW_ERROR_EN[5],STATE_CHANGE_EN[4],CMD_ERROR_EN[3],CHIP_READY_EN[2],LOW_BATT_EN[1],WUT_EN[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_INT_CTL_CHIP_ENABLE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PA_MODE { \
  0x11 /* CMD: Set property                                                           */,\
  0x22 /* GROUP: Pa                                                                   */,\
  0x01 /* NUM_PROPS                                                                   */,\
  0x00 /* START_PROP                                                                  */,\
  0x08 /* PA_MODE,EXT_PA_RAMP[7],DIG_PWR_SEQ[6],PA_SEL[5:2],PA_OV_CLAMP[1],PA_MODE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PA_MODE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PA_PWR_LVL { \
  0x11 /* CMD: Set property    */,\
  0x22 /* GROUP: Pa            */,\
  0x01 /* NUM_PROPS            */,\
  0x01 /* START_PROP           */,\
  0x7F /* PA_PWR_LVL,DDAC[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PA_PWR_LVL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PA_BIAS_CLKDUTY { \
  0x11 /* CMD: Set property                     */,\
  0x22 /* GROUP: Pa                             */,\
  0x01 /* NUM_PROPS                             */,\
  0x02 /* START_PROP                            */,\
  0x0C /* PA_BIAS_CLKDUTY,CLK_DUTY[7:6],OB[5:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PA_BIAS_CLKDUTY_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PA_RAMP_EX { \
  0x11 /* CMD: Set property            */,\
  0x22 /* GROUP: Pa                    */,\
  0x01 /* NUM_PROPS                    */,\
  0x04 /* START_PROP                   */,\
  0x0E /* PA_RAMP_EX,VSET[7:4],TC[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PA_RAMP_EX_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PA_RAMP_DOWN_DELAY { \
  0x11 /* CMD: Set property                       */,\
  0x22 /* GROUP: Pa                               */,\
  0x01 /* NUM_PROPS                               */,\
  0x05 /* START_PROP                              */,\
  0x23 /* PA_RAMP_DOWN_DELAY,RAMP_DOWN_DELAY[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PA_RAMP_DOWN_DELAY_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PA_DIG_PWR_SEQ_CONFIG { \
  0x11 /* CMD: Set property                                             */,\
  0x22 /* GROUP: Pa                                                     */,\
  0x01 /* NUM_PROPS                                                     */,\
  0x06 /* START_PROP                                                    */,\
  0x03 /* PA_DIG_PWR_SEQ_CONFIG,INC_STEP_SIZE[7],DIG_PWR_SEQ_DELAY[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PA_DIG_PWR_SEQ_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_TX_LENGTH { \
  0x11 /* CMD: Set property                 */,\
  0x10 /* GROUP: Preamble                   */,\
  0x01 /* NUM_PROPS                         */,\
  0x00 /* START_PROP                        */,\
  0x10 /* PREAMBLE_TX_LENGTH,TX_LENGTH[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_TX_LENGTH_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG_STD_1 { \
  0x11 /* CMD: Set property                                         */,\
  0x10 /* GROUP: Preamble                                           */,\
  0x01 /* NUM_PROPS                                                 */,\
  0x01 /* START_PROP                                                */,\
  0x14 /* PREAMBLE_CONFIG_STD_1,SKIP_SYNC_TIMEOUT[7],RX_THRESH[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG_STD_1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG_NSTD { \
  0x11 /* CMD: Set property                                       */,\
  0x10 /* GROUP: Preamble                                         */,\
  0x01 /* NUM_PROPS                                               */,\
  0x02 /* START_PROP                                              */,\
  0x00 /* PREAMBLE_CONFIG_NSTD,RX_ERRORS[7:5],PATTERN_LENGTH[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG_NSTD_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG_STD_2 { \
  0x11 /* CMD: Set property                                                              */,\
  0x10 /* GROUP: Preamble                                                                */,\
  0x01 /* NUM_PROPS                                                                      */,\
  0x03 /* START_PROP                                                                     */,\
  0x0F /* PREAMBLE_CONFIG_STD_2,RX_PREAMBLE_TIMEOUT_EXTEND[7:4],RX_PREAMBLE_TIMEOUT[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG_STD_2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG { \
  0x11 /* CMD: Set property                                                                                                     */,\
  0x10 /* GROUP: Preamble                                                                                                       */,\
  0x01 /* NUM_PROPS                                                                                                             */,\
  0x04 /* START_PROP                                                                                                            */,\
  0x21 /* PREAMBLE_CONFIG,RX_PREAM_SRC[7],PREAM_FIRST_1_OR_0[5],LENGTH_CONFIG[4],MAN_CONST[3],MAN_ENABLE[2],STANDARD_PREAM[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_PATTERN { \
  0x11 /* CMD: Set property                                                    */,\
  0x10 /* GROUP: Preamble                                                      */,\
  0x04 /* NUM_PROPS                                                            */,\
  0x05 /* START_PROP                                                           */,\
  0x00 /* PREAMBLE_PATTERN,PATTERN[7:0],PATTERN[7:0],PATTERN[7:0],PATTERN[7:0] */,\
  0x00 /* DATA1                                                                */,\
  0x00 /* DATA2                                                                */,\
  0x00 /* DATA3                                                                */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_PATTERN_LEN (8)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_POSTAMBLE_CONFIG { \
  0x11 /* CMD: Set property                                                                           */,\
  0x10 /* GROUP: Preamble                                                                             */,\
  0x01 /* NUM_PROPS                                                                                   */,\
  0x09 /* START_PROP                                                                                  */,\
  0x00 /* PREAMBLE_POSTAMBLE_CONFIG,POSTAMBLE_ENABLE[7],PKT_VALID_ON_POSTAMBLE[6],POSTAMBLE_SIZE[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_POSTAMBLE_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_POSTAMBLE_PATTERN { \
  0x11 /* CMD: Set property                                                                                                      */,\
  0x10 /* GROUP: Preamble                                                                                                        */,\
  0x04 /* NUM_PROPS                                                                                                              */,\
  0x0A /* START_PROP                                                                                                             */,\
  0x00 /* PREAMBLE_POSTAMBLE_PATTERN,POSTAMBLE_PATTERN[7:0],POSTAMBLE_PATTERN[7:0],POSTAMBLE_PATTERN[7:0],POSTAMBLE_PATTERN[7:0] */,\
  0x00 /* DATA1                                                                                                                  */,\
  0x00 /* DATA2                                                                                                                  */,\
  0x00 /* DATA3                                                                                                                  */\
}

#define RADIO_CONFIG_SET_PROPERTY_PREAMBLE_POSTAMBLE_PATTERN_LEN (8)


#define RADIO_CONFIG_SET_PROPERTY_SYNC_CONFIG { \
  0x11 /* CMD: Set property                                                  */,\
  0x11 /* GROUP: Sync                                                        */,\
  0x01 /* NUM_PROPS                                                          */,\
  0x00 /* START_PROP                                                         */,\
  0x01 /* SYNC_CONFIG,SKIP_TX[7],RX_ERRORS[6:4],4FSK[3],MANCH[2],LENGTH[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNC_BITS { \
  0x11 /* CMD: Set property                                 */,\
  0x11 /* GROUP: Sync                                       */,\
  0x04 /* NUM_PROPS                                         */,\
  0x01 /* START_PROP                                        */,\
  0xB4 /* SYNC_BITS,BITS[7:0],BITS[7:0],BITS[7:0],BITS[7:0] */,\
  0x2B /* DATA1                                             */,\
  0x00 /* DATA2                                             */,\
  0x00 /* DATA3                                             */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNC_BITS_LEN (8)


#define RADIO_CONFIG_SET_PROPERTY_SYNC_CONFIG2 { \
  0x11 /* CMD: Set property                                                                         */,\
  0x11 /* GROUP: Sync                                                                               */,\
  0x01 /* NUM_PROPS                                                                                 */,\
  0x05 /* START_PROP                                                                                */,\
  0x00 /* SYNC_CONFIG2,SYNC_ERROR_ONLY_BEGIN[7],SYNC_TIMEOUT_RST[5],DUAL_SYNC_EN[4],LENGTH_SUB[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNC_CONFIG2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                      */,\
  0x12 /* GROUP: Pkt                                                             */,\
  0x01 /* NUM_PROPS                                                              */,\
  0x00 /* START_PROP                                                             */,\
  0x04 /* PKT_CRC_CONFIG,CRC_SEED[7],ALT_CRC_POLYNOMIAL[6:4],CRC_POLYNOMIAL[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_WHT_POLY { \
  0x11 /* CMD: Set property                        */,\
  0x12 /* GROUP: Pkt                               */,\
  0x02 /* NUM_PROPS                                */,\
  0x01 /* START_PROP                               */,\
  0x05 /* PKT_WHT_POLY,WHT_POLY[7:0],WHT_POLY[7:0] */,\
  0x00 /* DATA1                                    */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_WHT_POLY_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_WHT_SEED { \
  0x11 /* CMD: Set property                                                    */,\
  0x12 /* GROUP: Pkt                                                           */,\
  0x02 /* NUM_PROPS                                                            */,\
  0x03 /* START_PROP                                                           */,\
  0xFF /* PKT_WHT_SEED,WHT_SEED[7:0],WHT_SEED[7:0],WHT_SEED[7:0],WHT_SEED[7:0] */,\
  0xFF /* DATA1                                                                */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_WHT_SEED_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_WHT_BIT_NUM { \
  0x11 /* CMD: Set property                                                                                                                             */,\
  0x12 /* GROUP: Pkt                                                                                                                                    */,\
  0x01 /* NUM_PROPS                                                                                                                                     */,\
  0x05 /* START_PROP                                                                                                                                    */,\
  0x20 /* PKT_WHT_BIT_NUM,SW_WHT_CTRL[7],SW_CRC_CTRL[6],PN_DIRECTION[5],WHT_BIT_NUM[3:0],SW_WHT_CTRL[7],SW_CRC_CTRL[6],PN_DIRECTION[5],WHT_BIT_NUM[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_WHT_BIT_NUM_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_CONFIG1 { \
  0x11 /* CMD: Set property                                                                                                                                                                                                                                   */,\
  0x12 /* GROUP: Pkt                                                                                                                                                                                                                                          */,\
  0x01 /* NUM_PROPS                                                                                                                                                                                                                                           */,\
  0x06 /* START_PROP                                                                                                                                                                                                                                          */,\
  0x80 /* PKT_CONFIG1,PH_FIELD_SPLIT[7],PH_RX_DISABLE[6],4FSK_EN[5],RX_MULTI_PKT[4],MANCH_POL[3],CRC_INVERT[2],CRC_ENDIAN[1],BIT_ORDER[0],PH_FIELD_SPLIT[7],PH_RX_DISABLE[6],4FSK_EN[5],RX_MULTI_PKT[4],MANCH_POL[3],CRC_INVERT[2],CRC_ENDIAN[1],BIT_ORDER[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_CONFIG1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_CONFIG2 { \
  0x11 /* CMD: Set property                                                                                                                       */,\
  0x12 /* GROUP: Pkt                                                                                                                              */,\
  0x01 /* NUM_PROPS                                                                                                                               */,\
  0x07 /* START_PROP                                                                                                                              */,\
  0x00 /* PKT_CONFIG2,CRC_BIT_ENDIAN[7],CRC_PADDING[6],ALT_CRC_SEED[5],EN_3_OF_6[4],CRC_BIT_ENDIAN[7],CRC_PADDING[6],ALT_CRC_SEED[5],EN_3_OF_6[4] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_CONFIG2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_LEN { \
  0x11 /* CMD: Set property                                                   */,\
  0x12 /* GROUP: Pkt                                                          */,\
  0x01 /* NUM_PROPS                                                           */,\
  0x08 /* START_PROP                                                          */,\
  0x2B /* PKT_LEN,INFINITE_LEN[6],ENDIAN[5],SIZE[4],IN_FIFO[3],DST_FIELD[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_LEN_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_LEN_FIELD_SOURCE { \
  0x11 /* CMD: Set property                   */,\
  0x12 /* GROUP: Pkt                          */,\
  0x01 /* NUM_PROPS                           */,\
  0x09 /* START_PROP                          */,\
  0x02 /* PKT_LEN_FIELD_SOURCE,SRC_FIELD[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_LEN_FIELD_SOURCE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_LEN_ADJUST { \
  0x11 /* CMD: Set property              */,\
  0x12 /* GROUP: Pkt                     */,\
  0x01 /* NUM_PROPS                      */,\
  0x0A /* START_PROP                     */,\
  0x00 /* PKT_LEN_ADJUST,LEN_ADJUST[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_LEN_ADJUST_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_TX_THRESHOLD { \
  0x11 /* CMD: Set property                  */,\
  0x12 /* GROUP: Pkt                         */,\
  0x01 /* NUM_PROPS                          */,\
  0x0B /* START_PROP                         */,\
  0x30 /* PKT_TX_THRESHOLD,TX_THRESHOLD[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_TX_THRESHOLD_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_THRESHOLD { \
  0x11 /* CMD: Set property                  */,\
  0x12 /* GROUP: Pkt                         */,\
  0x01 /* NUM_PROPS                          */,\
  0x0C /* START_PROP                         */,\
  0x30 /* PKT_RX_THRESHOLD,RX_THRESHOLD[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_THRESHOLD_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_LENGTH { \
  0x11 /* CMD: Set property                                          */,\
  0x12 /* GROUP: Pkt                                                 */,\
  0x02 /* NUM_PROPS                                                  */,\
  0x0D /* START_PROP                                                 */,\
  0x00 /* PKT_FIELD_1_LENGTH,FIELD_1_LENGTH[4:0],FIELD_1_LENGTH[7:0] */,\
  0x02 /* DATA1                                                      */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_CONFIG { \
  0x11 /* CMD: Set property                                         */,\
  0x12 /* GROUP: Pkt                                                */,\
  0x01 /* NUM_PROPS                                                 */,\
  0x0F /* START_PROP                                                */,\
  0x00 /* PKT_FIELD_1_CONFIG,4FSK[4],PN_START[2],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                                                                              */,\
  0x12 /* GROUP: Pkt                                                                                                                                     */,\
  0x01 /* NUM_PROPS                                                                                                                                      */,\
  0x10 /* START_PROP                                                                                                                                     */,\
  0x82 /* PKT_FIELD_1_CRC_CONFIG,CRC_START[7],ALT_CRC_START[6],SEND_CRC[5],SEND_ALT_CRC[4],CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_LENGTH { \
  0x11 /* CMD: Set property                                          */,\
  0x12 /* GROUP: Pkt                                                 */,\
  0x02 /* NUM_PROPS                                                  */,\
  0x11 /* START_PROP                                                 */,\
  0x00 /* PKT_FIELD_2_LENGTH,FIELD_2_LENGTH[4:0],FIELD_2_LENGTH[7:0] */,\
  0x01 /* DATA1                                                      */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_CONFIG { \
  0x11 /* CMD: Set property                             */,\
  0x12 /* GROUP: Pkt                                    */,\
  0x01 /* NUM_PROPS                                     */,\
  0x13 /* START_PROP                                    */,\
  0x00 /* PKT_FIELD_2_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                                                */,\
  0x12 /* GROUP: Pkt                                                                                                       */,\
  0x01 /* NUM_PROPS                                                                                                        */,\
  0x14 /* START_PROP                                                                                                       */,\
  0x0A /* PKT_FIELD_2_CRC_CONFIG,SEND_CRC[5],SEND_ALT_CRC[4],CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_3_LENGTH { \
  0x11 /* CMD: Set property                                          */,\
  0x12 /* GROUP: Pkt                                                 */,\
  0x02 /* NUM_PROPS                                                  */,\
  0x15 /* START_PROP                                                 */,\
  0x00 /* PKT_FIELD_3_LENGTH,FIELD_3_LENGTH[4:0],FIELD_3_LENGTH[7:0] */,\
  0xFC /* DATA1                                                      */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_3_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_3_CONFIG { \
  0x11 /* CMD: Set property                             */,\
  0x12 /* GROUP: Pkt                                    */,\
  0x01 /* NUM_PROPS                                     */,\
  0x17 /* START_PROP                                    */,\
  0x00 /* PKT_FIELD_3_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_3_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_3_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                                                */,\
  0x12 /* GROUP: Pkt                                                                                                       */,\
  0x01 /* NUM_PROPS                                                                                                        */,\
  0x18 /* START_PROP                                                                                                       */,\
  0x22 /* PKT_FIELD_3_CRC_CONFIG,SEND_CRC[5],SEND_ALT_CRC[4],CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_3_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_4_LENGTH { \
  0x11 /* CMD: Set property                                          */,\
  0x12 /* GROUP: Pkt                                                 */,\
  0x02 /* NUM_PROPS                                                  */,\
  0x19 /* START_PROP                                                 */,\
  0x00 /* PKT_FIELD_4_LENGTH,FIELD_4_LENGTH[4:0],FIELD_4_LENGTH[7:0] */,\
  0x00 /* DATA1                                                      */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_4_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_4_CONFIG { \
  0x11 /* CMD: Set property                             */,\
  0x12 /* GROUP: Pkt                                    */,\
  0x01 /* NUM_PROPS                                     */,\
  0x1B /* START_PROP                                    */,\
  0x00 /* PKT_FIELD_4_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_4_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_4_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                                                */,\
  0x12 /* GROUP: Pkt                                                                                                       */,\
  0x01 /* NUM_PROPS                                                                                                        */,\
  0x1C /* START_PROP                                                                                                       */,\
  0x00 /* PKT_FIELD_4_CRC_CONFIG,SEND_CRC[5],SEND_ALT_CRC[4],CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_4_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_5_LENGTH { \
  0x11 /* CMD: Set property                                          */,\
  0x12 /* GROUP: Pkt                                                 */,\
  0x02 /* NUM_PROPS                                                  */,\
  0x1D /* START_PROP                                                 */,\
  0x00 /* PKT_FIELD_5_LENGTH,FIELD_5_LENGTH[4:0],FIELD_5_LENGTH[7:0] */,\
  0x00 /* DATA1                                                      */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_5_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_5_CONFIG { \
  0x11 /* CMD: Set property                             */,\
  0x12 /* GROUP: Pkt                                    */,\
  0x01 /* NUM_PROPS                                     */,\
  0x1F /* START_PROP                                    */,\
  0x00 /* PKT_FIELD_5_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_5_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_5_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                                                */,\
  0x12 /* GROUP: Pkt                                                                                                       */,\
  0x01 /* NUM_PROPS                                                                                                        */,\
  0x20 /* START_PROP                                                                                                       */,\
  0x00 /* PKT_FIELD_5_CRC_CONFIG,SEND_CRC[5],SEND_ALT_CRC[4],CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_5_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_1_LENGTH { \
  0x11 /* CMD: Set property                                                   */,\
  0x12 /* GROUP: Pkt                                                          */,\
  0x02 /* NUM_PROPS                                                           */,\
  0x21 /* START_PROP                                                          */,\
  0x00 /* PKT_RX_FIELD_1_LENGTH,RX_FIELD_1_LENGTH[4:0],RX_FIELD_1_LENGTH[7:0] */,\
  0x02 /* DATA1                                                               */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_1_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_1_CONFIG { \
  0x11 /* CMD: Set property                                            */,\
  0x12 /* GROUP: Pkt                                                   */,\
  0x01 /* NUM_PROPS                                                    */,\
  0x23 /* START_PROP                                                   */,\
  0x00 /* PKT_RX_FIELD_1_CONFIG,4FSK[4],PN_START[2],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_1_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_1_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                                                     */,\
  0x12 /* GROUP: Pkt                                                                                                            */,\
  0x01 /* NUM_PROPS                                                                                                             */,\
  0x24 /* START_PROP                                                                                                            */,\
  0x82 /* PKT_RX_FIELD_1_CRC_CONFIG,CRC_START[7],ALT_CRC_START[6],CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_1_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_2_LENGTH { \
  0x11 /* CMD: Set property                                                   */,\
  0x12 /* GROUP: Pkt                                                          */,\
  0x02 /* NUM_PROPS                                                           */,\
  0x25 /* START_PROP                                                          */,\
  0x00 /* PKT_RX_FIELD_2_LENGTH,RX_FIELD_2_LENGTH[4:0],RX_FIELD_2_LENGTH[7:0] */,\
  0x01 /* DATA1                                                               */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_2_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_2_CONFIG { \
  0x11 /* CMD: Set property                                */,\
  0x12 /* GROUP: Pkt                                       */,\
  0x01 /* NUM_PROPS                                        */,\
  0x27 /* START_PROP                                       */,\
  0x00 /* PKT_RX_FIELD_2_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_2_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_2_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                       */,\
  0x12 /* GROUP: Pkt                                                                              */,\
  0x01 /* NUM_PROPS                                                                               */,\
  0x28 /* START_PROP                                                                              */,\
  0x02 /* PKT_RX_FIELD_2_CRC_CONFIG,CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_2_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_3_LENGTH { \
  0x11 /* CMD: Set property                                                   */,\
  0x12 /* GROUP: Pkt                                                          */,\
  0x02 /* NUM_PROPS                                                           */,\
  0x29 /* START_PROP                                                          */,\
  0x00 /* PKT_RX_FIELD_3_LENGTH,RX_FIELD_3_LENGTH[4:0],RX_FIELD_3_LENGTH[7:0] */,\
  0xFC /* DATA1                                                               */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_3_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_3_CONFIG { \
  0x11 /* CMD: Set property                                */,\
  0x12 /* GROUP: Pkt                                       */,\
  0x01 /* NUM_PROPS                                        */,\
  0x2B /* START_PROP                                       */,\
  0x00 /* PKT_RX_FIELD_3_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_3_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_3_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                       */,\
  0x12 /* GROUP: Pkt                                                                              */,\
  0x01 /* NUM_PROPS                                                                               */,\
  0x2C /* START_PROP                                                                              */,\
  0x0A /* PKT_RX_FIELD_3_CRC_CONFIG,CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_3_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_4_LENGTH { \
  0x11 /* CMD: Set property                                                   */,\
  0x12 /* GROUP: Pkt                                                          */,\
  0x02 /* NUM_PROPS                                                           */,\
  0x2D /* START_PROP                                                          */,\
  0x00 /* PKT_RX_FIELD_4_LENGTH,RX_FIELD_4_LENGTH[4:0],RX_FIELD_4_LENGTH[7:0] */,\
  0x00 /* DATA1                                                               */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_4_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_4_CONFIG { \
  0x11 /* CMD: Set property                                */,\
  0x12 /* GROUP: Pkt                                       */,\
  0x01 /* NUM_PROPS                                        */,\
  0x2F /* START_PROP                                       */,\
  0x00 /* PKT_RX_FIELD_4_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_4_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_4_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                       */,\
  0x12 /* GROUP: Pkt                                                                              */,\
  0x01 /* NUM_PROPS                                                                               */,\
  0x30 /* START_PROP                                                                              */,\
  0x00 /* PKT_RX_FIELD_4_CRC_CONFIG,CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_4_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_5_LENGTH { \
  0x11 /* CMD: Set property                                                   */,\
  0x12 /* GROUP: Pkt                                                          */,\
  0x02 /* NUM_PROPS                                                           */,\
  0x31 /* START_PROP                                                          */,\
  0x00 /* PKT_RX_FIELD_5_LENGTH,RX_FIELD_5_LENGTH[4:0],RX_FIELD_5_LENGTH[7:0] */,\
  0x00 /* DATA1                                                               */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_5_LENGTH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_5_CONFIG { \
  0x11 /* CMD: Set property                                */,\
  0x12 /* GROUP: Pkt                                       */,\
  0x01 /* NUM_PROPS                                        */,\
  0x33 /* START_PROP                                       */,\
  0x00 /* PKT_RX_FIELD_5_CONFIG,4FSK[4],WHITEN[1],MANCH[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_5_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_5_CRC_CONFIG { \
  0x11 /* CMD: Set property                                                                       */,\
  0x12 /* GROUP: Pkt                                                                              */,\
  0x01 /* NUM_PROPS                                                                               */,\
  0x34 /* START_PROP                                                                              */,\
  0x00 /* PKT_RX_FIELD_5_CRC_CONFIG,CHECK_CRC[3],CHECK_ALT_CRC[2],CRC_ENABLE[1],ALT_CRC_ENABLE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_RX_FIELD_5_CRC_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PKT_CRC_SEED { \
  0x11 /* CMD: Set property                                                    */,\
  0x12 /* GROUP: Pkt                                                           */,\
  0x04 /* NUM_PROPS                                                            */,\
  0x36 /* START_PROP                                                           */,\
  0x00 /* PKT_CRC_SEED,CRC_SEED[7:0],CRC_SEED[7:0],CRC_SEED[7:0],CRC_SEED[7:0] */,\
  0x00 /* DATA1                                                                */,\
  0x00 /* DATA2                                                                */,\
  0x00 /* DATA3                                                                */\
}

#define RADIO_CONFIG_SET_PROPERTY_PKT_CRC_SEED_LEN (8)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_1 { \
  0x11 /* CMD: Set property          */,\
  0x30 /* GROUP: Match               */,\
  0x01 /* NUM_PROPS                  */,\
  0x00 /* START_PROP                 */,\
  0x00 /* MATCH_VALUE_1,VALUE_1[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_1 { \
  0x11 /* CMD: Set property        */,\
  0x30 /* GROUP: Match             */,\
  0x01 /* NUM_PROPS                */,\
  0x01 /* START_PROP               */,\
  0x00 /* MATCH_MASK_1,MASK_1[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_1 { \
  0x11 /* CMD: Set property                                */,\
  0x30 /* GROUP: Match                                     */,\
  0x01 /* NUM_PROPS                                        */,\
  0x02 /* START_PROP                                       */,\
  0x40 /* MATCH_CTRL_1,POLARITY[7],MATCH_EN[6],OFFSET[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_2 { \
  0x11 /* CMD: Set property          */,\
  0x30 /* GROUP: Match               */,\
  0x01 /* NUM_PROPS                  */,\
  0x03 /* START_PROP                 */,\
  0x00 /* MATCH_VALUE_2,VALUE_2[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_2 { \
  0x11 /* CMD: Set property        */,\
  0x30 /* GROUP: Match             */,\
  0x01 /* NUM_PROPS                */,\
  0x04 /* START_PROP               */,\
  0x00 /* MATCH_MASK_2,MASK_2[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_2 { \
  0x11 /* CMD: Set property                             */,\
  0x30 /* GROUP: Match                                  */,\
  0x01 /* NUM_PROPS                                     */,\
  0x05 /* START_PROP                                    */,\
  0x00 /* MATCH_CTRL_2,POLARITY[7],LOGIC[6],OFFSET[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_3 { \
  0x11 /* CMD: Set property          */,\
  0x30 /* GROUP: Match               */,\
  0x01 /* NUM_PROPS                  */,\
  0x06 /* START_PROP                 */,\
  0x55 /* MATCH_VALUE_3,VALUE_3[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_3_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_3 { \
  0x11 /* CMD: Set property        */,\
  0x30 /* GROUP: Match             */,\
  0x01 /* NUM_PROPS                */,\
  0x07 /* START_PROP               */,\
  0xFF /* MATCH_MASK_3,MASK_3[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_3_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_3 { \
  0x11 /* CMD: Set property                             */,\
  0x30 /* GROUP: Match                                  */,\
  0x01 /* NUM_PROPS                                     */,\
  0x08 /* START_PROP                                    */,\
  0x00 /* MATCH_CTRL_3,POLARITY[7],LOGIC[6],OFFSET[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_3_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_4 { \
  0x11 /* CMD: Set property          */,\
  0x30 /* GROUP: Match               */,\
  0x01 /* NUM_PROPS                  */,\
  0x09 /* START_PROP                 */,\
  0xAA /* MATCH_VALUE_4,VALUE_4[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_VALUE_4_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_4 { \
  0x11 /* CMD: Set property        */,\
  0x30 /* GROUP: Match             */,\
  0x01 /* NUM_PROPS                */,\
  0x0A /* START_PROP               */,\
  0xFF /* MATCH_MASK_4,MASK_4[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_MASK_4_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_4 { \
  0x11 /* CMD: Set property                             */,\
  0x30 /* GROUP: Match                                  */,\
  0x01 /* NUM_PROPS                                     */,\
  0x0B /* START_PROP                                    */,\
  0x01 /* MATCH_CTRL_4,POLARITY[7],LOGIC[6],OFFSET[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MATCH_CTRL_4_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_8 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x0F /* START_PROP                          */,\
  0x67 /* MODEM_TX_FILTER_COEFF_8,TXCOE8[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_8_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_7 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x10 /* START_PROP                          */,\
  0x60 /* MODEM_TX_FILTER_COEFF_7,TXCOE7[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_7_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_6 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x11 /* START_PROP                          */,\
  0x4D /* MODEM_TX_FILTER_COEFF_6,TXCOE6[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_6_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_5 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x12 /* START_PROP                          */,\
  0x36 /* MODEM_TX_FILTER_COEFF_5,TXCOE5[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_5_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_4 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x13 /* START_PROP                          */,\
  0x21 /* MODEM_TX_FILTER_COEFF_4,TXCOE4[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_4_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_3 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x14 /* START_PROP                          */,\
  0x11 /* MODEM_TX_FILTER_COEFF_3,TXCOE3[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_3_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_2 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x15 /* START_PROP                          */,\
  0x08 /* MODEM_TX_FILTER_COEFF_2,TXCOE2[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_1 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x16 /* START_PROP                          */,\
  0x03 /* MODEM_TX_FILTER_COEFF_1,TXCOE1[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_0 { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x01 /* NUM_PROPS                           */,\
  0x17 /* START_PROP                          */,\
  0x01 /* MODEM_TX_FILTER_COEFF_0,TXCOE0[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_FILTER_COEFF_0_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_MOD_TYPE { \
  0x11 /* CMD: Set property                                                                            */,\
  0x20 /* GROUP: Modem                                                                                 */,\
  0x01 /* NUM_PROPS                                                                                    */,\
  0x00 /* START_PROP                                                                                   */,\
  0x03 /* MODEM_MOD_TYPE,TX_DIRECT_MODE_TYPE[7],TX_DIRECT_MODE_GPIO[6:5],MOD_SOURCE[4:3],MOD_TYPE[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_MOD_TYPE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_MAP_CONTROL { \
  0x11 /* CMD: Set property                                                                    */,\
  0x20 /* GROUP: Modem                                                                         */,\
  0x01 /* NUM_PROPS                                                                            */,\
  0x01 /* START_PROP                                                                           */,\
  0x00 /* MODEM_MAP_CONTROL,ENMANCH[7],ENINV_RXBIT[6],ENINV_TXBIT[5],ENINV_FD[4],ENINV_ADCQ[3] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_MAP_CONTROL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSM_CTRL { \
  0x11 /* CMD: Set property                                                                                    */,\
  0x20 /* GROUP: Modem                                                                                         */,\
  0x01 /* NUM_PROPS                                                                                            */,\
  0x02 /* START_PROP                                                                                           */,\
  0x07 /* MODEM_DSM_CTRL,DSMCLK_SEL[7],DSM_MODE[6],DSMDT_EN[5],DSMDTTP[4],DSM_RST[3],DSM_LSB[2],DSM_ORDER[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSM_CTRL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_CLKGEN_BAND { \
  0x11 /* CMD: Set property                                       */,\
  0x20 /* GROUP: Modem                                            */,\
  0x01 /* NUM_PROPS                                               */,\
  0x51 /* START_PROP                                              */,\
  0x08 /* MODEM_CLKGEN_BAND,FORCE_SY_RECAL[4],SY_SEL[3],BAND[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_CLKGEN_BAND_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPFF { \
  0x11 /* CMD: Set property                                 */,\
  0x23 /* GROUP: Synth                                      */,\
  0x01 /* NUM_PROPS                                         */,\
  0x00 /* START_PROP                                        */,\
  0x2C /* SYNTH_PFDCP_CPFF,CP_FF_CUR_TEST[6],CP_FF_CUR[5:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPFF_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPINT { \
  0x11 /* CMD: Set property                 */,\
  0x23 /* GROUP: Synth                      */,\
  0x01 /* NUM_PROPS                         */,\
  0x01 /* START_PROP                        */,\
  0x0E /* SYNTH_PFDCP_CPINT,CP_INT_CUR[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPINT_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNTH_VCO_KV { \
  0x11 /* CMD: Set property                                */,\
  0x23 /* GROUP: Synth                                     */,\
  0x01 /* NUM_PROPS                                        */,\
  0x02 /* START_PROP                                       */,\
  0x0B /* SYNTH_VCO_KV,RESERVED[4],KV_DIR[3:2],KV_INT[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNTH_VCO_KV_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT3 { \
  0x11 /* CMD: Set property            */,\
  0x23 /* GROUP: Synth                 */,\
  0x01 /* NUM_PROPS                    */,\
  0x03 /* START_PROP                   */,\
  0x04 /* SYNTH_LPFILT3,LPF_FF_R2[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT3_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT2 { \
  0x11 /* CMD: Set property            */,\
  0x23 /* GROUP: Synth                 */,\
  0x01 /* NUM_PROPS                    */,\
  0x04 /* START_PROP                   */,\
  0x0C /* SYNTH_LPFILT2,LPF_FF_C2[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT1 { \
  0x11 /* CMD: Set property                                               */,\
  0x23 /* GROUP: Synth                                                    */,\
  0x01 /* NUM_PROPS                                                       */,\
  0x05 /* START_PROP                                                      */,\
  0x73 /* SYNTH_LPFILT1,LPF_FF_C1[6:4],LPF_FF_C1_CODE[3:2],LPF_FF_C3[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT0 { \
  0x11 /* CMD: Set property              */,\
  0x23 /* GROUP: Synth                   */,\
  0x01 /* NUM_PROPS                      */,\
  0x06 /* START_PROP                     */,\
  0x03 /* SYNTH_LPFILT0,LPF_FF_BIAS[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT0_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DATA_RATE { \
  0x11 /* CMD: Set property                                            */,\
  0x20 /* GROUP: Modem                                                 */,\
  0x03 /* NUM_PROPS                                                    */,\
  0x03 /* START_PROP                                                   */,\
  0x13 /* MODEM_DATA_RATE,DATA_RATE[7:0],DATA_RATE[7:0],DATA_RATE[7:0] */,\
  0x88 /* DATA1                                                        */,\
  0x00 /* DATA2                                                        */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DATA_RATE_LEN (7)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_NCO_MODE { \
  0x11 /* CMD: Set property                                                            */,\
  0x20 /* GROUP: Modem                                                                 */,\
  0x04 /* NUM_PROPS                                                                    */,\
  0x06 /* START_PROP                                                                   */,\
  0x09 /* MODEM_TX_NCO_MODE,TXOSR[3:2],NCOMOD[1:0],NCOMOD[7:0],NCOMOD[7:0],NCOMOD[7:0] */,\
  0xC9 /* DATA1                                                                        */,\
  0xC3 /* DATA2                                                                        */,\
  0x80 /* DATA3                                                                        */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_NCO_MODE_LEN (8)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV { \
  0x11 /* CMD: Set property                                   */,\
  0x20 /* GROUP: Modem                                        */,\
  0x03 /* NUM_PROPS                                           */,\
  0x0A /* START_PROP                                          */,\
  0x00 /* MODEM_FREQ_DEV,FREQDEV[0],FREQDEV[7:0],FREQDEV[7:0] */,\
  0x0D /* DATA1                                               */,\
  0x1B /* DATA2                                               */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV_LEN (7)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_RAMP_DELAY { \
  0x11 /* CMD: Set property                 */,\
  0x20 /* GROUP: Modem                      */,\
  0x01 /* NUM_PROPS                         */,\
  0x18 /* START_PROP                        */,\
  0x01 /* MODEM_TX_RAMP_DELAY,RAMP_DLY[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_TX_RAMP_DELAY_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_PA_TC { \
  0x11 /* CMD: Set property              */,\
  0x22 /* GROUP: Pa                      */,\
  0x01 /* NUM_PROPS                      */,\
  0x03 /* START_PROP                     */,\
  0x1D /* PA_TC,FSK_MOD_DLY[7:5],TC[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_PA_TC_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_INTE { \
  0x11 /* CMD: Set property           */,\
  0x40 /* GROUP: Freq control         */,\
  0x01 /* NUM_PROPS                   */,\
  0x00 /* START_PROP                  */,\
  0x3C /* FREQ_CONTROL_INTE,INTE[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_INTE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC { \
  0x11 /* CMD: Set property                               */,\
  0x40 /* GROUP: Freq control                             */,\
  0x03 /* NUM_PROPS                                       */,\
  0x01 /* START_PROP                                      */,\
  0x08 /* FREQ_CONTROL_FRAC,FRAC[3:0],FRAC[7:0],FRAC[7:0] */,\
  0x00 /* DATA1                                           */,\
  0x00 /* DATA2                                           */\
}

#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_LEN (7)


#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE { \
  0x11 /* CMD: Set property                                                            */,\
  0x40 /* GROUP: Freq control                                                          */,\
  0x02 /* NUM_PROPS                                                                    */,\
  0x04 /* START_PROP                                                                   */,\
  0x22 /* FREQ_CONTROL_CHANNEL_STEP_SIZE,CHANNEL_STEP_SIZE[7:0],CHANNEL_STEP_SIZE[7:0] */,\
  0x22 /* DATA1                                                                        */\
}

#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_W_SIZE { \
  0x11 /* CMD: Set property               */,\
  0x40 /* GROUP: Freq control             */,\
  0x01 /* NUM_PROPS                       */,\
  0x06 /* START_PROP                      */,\
  0x20 /* FREQ_CONTROL_W_SIZE,W_SIZE[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_W_SIZE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_VCOCNT_RX_ADJ { \
  0x11 /* CMD: Set property                             */,\
  0x40 /* GROUP: Freq control                           */,\
  0x01 /* NUM_PROPS                                     */,\
  0x07 /* START_PROP                                    */,\
  0xFF /* FREQ_CONTROL_VCOCNT_RX_ADJ,VCOCNT_RX_ADJ[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_VCOCNT_RX_ADJ_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_MDM_CTRL { \
  0x11 /* CMD: Set property                            */,\
  0x20 /* GROUP: Modem                                 */,\
  0x01 /* NUM_PROPS                                    */,\
  0x19 /* START_PROP                                   */,\
  0x80 /* MODEM_MDM_CTRL,PH_SRC_SEL[7],FIFO_SRC_SEL[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_MDM_CTRL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_IF_CONTROL { \
  0x11 /* CMD: Set property                                  */,\
  0x20 /* GROUP: Modem                                       */,\
  0x01 /* NUM_PROPS                                          */,\
  0x1A /* START_PROP                                         */,\
  0x08 /* MODEM_IF_CONTROL,ZEROIF[4],FIXIF[3],ETSI_MODE[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_IF_CONTROL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_IF_FREQ { \
  0x11 /* CMD: Set property                                    */,\
  0x20 /* GROUP: Modem                                         */,\
  0x03 /* NUM_PROPS                                            */,\
  0x1B /* START_PROP                                           */,\
  0x03 /* MODEM_IF_FREQ,IF_FREQ[1:0],IF_FREQ[7:0],IF_FREQ[7:0] */,\
  0xC0 /* DATA1                                                */,\
  0x00 /* DATA2                                                */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_IF_FREQ_LEN (7)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG1 { \
  0x11 /* CMD: Set property                                      */,\
  0x20 /* GROUP: Modem                                           */,\
  0x01 /* NUM_PROPS                                              */,\
  0x1E /* START_PROP                                             */,\
  0x00 /* MODEM_DECIMATION_CFG1,NDEC2[7:6],NDEC1[5:4],NDEC0[3:1] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG0 { \
  0x11 /* CMD: Set property                                                                    */,\
  0x20 /* GROUP: Modem                                                                         */,\
  0x01 /* NUM_PROPS                                                                            */,\
  0x1F /* START_PROP                                                                           */,\
  0x10 /* MODEM_DECIMATION_CFG0,CHFLT_LOPW[7],DROOPFLTBYP[6],DWN3BYP[5],DWN2BYP[4],RXGAINX2[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG0_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_OSR { \
  0x11 /* CMD: Set property                   */,\
  0x20 /* GROUP: Modem                        */,\
  0x02 /* NUM_PROPS                           */,\
  0x22 /* START_PROP                          */,\
  0x00 /* MODEM_BCR_OSR,RXOSR[3:0],RXOSR[7:0] */,\
  0x9C /* DATA1                               */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_OSR_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET { \
  0x11 /* CMD: Set property                                     */,\
  0x20 /* GROUP: Modem                                          */,\
  0x03 /* NUM_PROPS                                             */,\
  0x24 /* START_PROP                                            */,\
  0x03 /* MODEM_BCR_NCO_OFFSET,NCOFF[5:0],NCOFF[7:0],NCOFF[7:0] */,\
  0x46 /* DATA1                                                 */,\
  0xDC /* DATA2                                                 */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET_LEN (7)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GAIN { \
  0x11 /* CMD: Set property                      */,\
  0x20 /* GROUP: Modem                           */,\
  0x02 /* NUM_PROPS                              */,\
  0x27 /* START_PROP                             */,\
  0x01 /* MODEM_BCR_GAIN,CRGAIN[2:0],CRGAIN[7:0] */,\
  0xA4 /* DATA1                                  */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GAIN_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GEAR { \
  0x11 /* CMD: Set property                      */,\
  0x20 /* GROUP: Modem                           */,\
  0x01 /* NUM_PROPS                              */,\
  0x29 /* START_PROP                             */,\
  0x02 /* MODEM_BCR_GEAR,CRFAST[5:3],CRSLOW[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GEAR_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_MISC0 { \
  0x11 /* CMD: Set property                                                                            */,\
  0x20 /* GROUP: Modem                                                                                 */,\
  0x01 /* NUM_PROPS                                                                                    */,\
  0x2B /* START_PROP                                                                                   */,\
  0x00 /* MODEM_BCR_MISC0,ADCWATCH[7],ADCRST[6],DISTOGG[5],PH0SIZE[4],RES_LOCKUP_BYP[3],DSA_BCR_RST[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_MISC0_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_MISC1 { \
  0x11 /* CMD: Set property                                                                                          */,\
  0x20 /* GROUP: Modem                                                                                               */,\
  0x01 /* NUM_PROPS                                                                                                  */,\
  0x2A /* START_PROP                                                                                                 */,\
  0xC2 /* MODEM_BCR_MISC1,BCRFBBYP[7],SLICEFBBYP[6],RXNCOCOMP[4],RXCOMP_LAT[3],CRGAINX2[2],DIS_MIDPT[1],ESC_MIDPT[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_MISC1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GEAR { \
  0x11 /* CMD: Set property                                       */,\
  0x20 /* GROUP: Modem                                            */,\
  0x01 /* NUM_PROPS                                               */,\
  0x2C /* START_PROP                                              */,\
  0x04 /* MODEM_AFC_GEAR,GEAR_SW[7:6],AFC_FAST[5:3],AFC_SLOW[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GEAR_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_WAIT { \
  0x11 /* CMD: Set property                      */,\
  0x20 /* GROUP: Modem                           */,\
  0x01 /* NUM_PROPS                              */,\
  0x2D /* START_PROP                             */,\
  0x32 /* MODEM_AFC_WAIT,SHWAIT[7:4],LGWAIT[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_WAIT_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN { \
  0x11 /* CMD: Set property                                                          */,\
  0x20 /* GROUP: Modem                                                               */,\
  0x02 /* NUM_PROPS                                                                  */,\
  0x2E /* START_PROP                                                                 */,\
  0x80 /* MODEM_AFC_GAIN,ENAFC[7],AFCBD[6],AFC_GAIN_DIV[5],AFCGAIN[4:0],AFCGAIN[7:0] */,\
  0x5D /* DATA1                                                                      */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_LIMITER { \
  0x11 /* CMD: Set property                         */,\
  0x20 /* GROUP: Modem                              */,\
  0x02 /* NUM_PROPS                                 */,\
  0x30 /* START_PROP                                */,\
  0x08 /* MODEM_AFC_LIMITER,AFCLIM[6:0],AFCLIM[7:0] */,\
  0xEA /* DATA1                                     */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_LIMITER_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_MISC { \
  0x11 /* CMD: Set property                                                                                                          */,\
  0x20 /* GROUP: Modem                                                                                                               */,\
  0x01 /* NUM_PROPS                                                                                                                  */,\
  0x32 /* START_PROP                                                                                                                 */,\
  0x80 /* MODEM_AFC_MISC,ENAFCFRZ[7],ENFBPLL[6],EN2TB_EST[5],ENFZPMEND[4],ENAFC_CLKSW[3],OOK_ZEROG[2],NON_FRZEN[1],LARGE_FREQ_ERR[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_MISC_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_CONTROL { \
  0x11 /* CMD: Set property                                                                                                                      */,\
  0x20 /* GROUP: Modem                                                                                                                           */,\
  0x01 /* NUM_PROPS                                                                                                                              */,\
  0x35 /* START_PROP                                                                                                                             */,\
  0xE0 /* MODEM_AGC_CONTROL,AGCOVPKT[7],IFPDSLOW[6],RFPDSLOW[5],SGI_N[4],AGC_SLOW[3],FORCE_ADC_GAIN_LOW[2],ADC_GAIN_COR_EN[1],RST_PKDT_PERIOD[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_CONTROL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_WINDOW_SIZE { \
  0x11 /* CMD: Set property                                 */,\
  0x20 /* GROUP: Modem                                      */,\
  0x01 /* NUM_PROPS                                         */,\
  0x38 /* START_PROP                                        */,\
  0x11 /* MODEM_AGC_WINDOW_SIZE,MEASWIN[7:4],SETTLEWIN[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_WINDOW_SIZE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_RFPD_DECAY { \
  0x11 /* CMD: Set property                    */,\
  0x20 /* GROUP: Modem                         */,\
  0x01 /* NUM_PROPS                            */,\
  0x39 /* START_PROP                           */,\
  0x22 /* MODEM_AGC_RFPD_DECAY,RFPD_DECAY[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_RFPD_DECAY_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_IFPD_DECAY { \
  0x11 /* CMD: Set property                    */,\
  0x20 /* GROUP: Modem                         */,\
  0x01 /* NUM_PROPS                            */,\
  0x3A /* START_PROP                           */,\
  0x22 /* MODEM_AGC_IFPD_DECAY,IFPD_DECAY[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_IFPD_DECAY_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_GAIN1 { \
  0x11 /* CMD: Set property                                     */,\
  0x20 /* GROUP: Modem                                          */,\
  0x01 /* NUM_PROPS                                             */,\
  0x3B /* START_PROP                                            */,\
  0x80 /* MODEM_FSK4_GAIN1,4FSK_ISIS_DISABLE[7],4FSK_GAIN1[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_GAIN1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_GAIN0 { \
  0x11 /* CMD: Set property                                   */,\
  0x20 /* GROUP: Modem                                        */,\
  0x01 /* NUM_PROPS                                           */,\
  0x3C /* START_PROP                                          */,\
  0x1A /* MODEM_FSK4_GAIN0,PHASE_COMP_2FSK[7],4FSK_GAIN0[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_GAIN0_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_TH { \
  0x11 /* CMD: Set property                     */,\
  0x20 /* GROUP: Modem                          */,\
  0x02 /* NUM_PROPS                             */,\
  0x3D /* START_PROP                            */,\
  0xC0 /* MODEM_FSK4_TH,4FSKTH[7:0],4FSKTH[7:0] */,\
  0x00 /* DATA1                                 */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_TH_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_MAP { \
  0x11 /* CMD: Set property           */,\
  0x20 /* GROUP: Modem                */,\
  0x01 /* NUM_PROPS                   */,\
  0x3F /* START_PROP                  */,\
  0x00 /* MODEM_FSK4_MAP,4FSKMAP[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_FSK4_MAP_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_PDTC { \
  0x11 /* CMD: Set property                     */,\
  0x20 /* GROUP: Modem                          */,\
  0x01 /* NUM_PROPS                             */,\
  0x40 /* START_PROP                            */,\
  0x29 /* MODEM_OOK_PDTC,ATTACK[6:4],DECAY[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_PDTC_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_BLOPK { \
  0x11 /* CMD: Set property          */,\
  0x20 /* GROUP: Modem               */,\
  0x01 /* NUM_PROPS                  */,\
  0x41 /* START_PROP                 */,\
  0x0C /* MODEM_OOK_BLOPK,BW_PK[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_BLOPK_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_CNT1 { \
  0x11 /* CMD: Set property                                                                            */,\
  0x20 /* GROUP: Modem                                                                                 */,\
  0x01 /* NUM_PROPS                                                                                    */,\
  0x42 /* START_PROP                                                                                   */,\
  0xA4 /* MODEM_OOK_CNT1,S2P_MAP[7:6],OOKFRZEN[5],MA_FREQDWN[4],RAW_SYN[3],SLICER_FAST[2],SQUELCH[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_CNT1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_MISC { \
  0x11 /* CMD: Set property                                                                                   */,\
  0x20 /* GROUP: Modem                                                                                        */,\
  0x01 /* NUM_PROPS                                                                                           */,\
  0x43 /* START_PROP                                                                                          */,\
  0x22 /* MODEM_OOK_MISC,OOKFASTMA[7],OOK_LIMIT_DISCHG[5],OOK_SQUELCH_EN[4],OOK_DISCHG_DIV[3:2],DETECTOR[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_OOK_MISC_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_SEARCH2 { \
  0x11 /* CMD: Set property                                                           */,\
  0x20 /* GROUP: Modem                                                                */,\
  0x01 /* NUM_PROPS                                                                   */,\
  0x50 /* START_PROP                                                                  */,\
  0x84 /* MODEM_RAW_SEARCH2,SCH_FRZEN[7],RAWFLT_SEL[6],SCHPRD_HI[5:3],SCHPRD_LOW[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_SEARCH2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_CONTROL { \
  0x11 /* CMD: Set property                                                        */,\
  0x20 /* GROUP: Modem                                                             */,\
  0x01 /* NUM_PROPS                                                                */,\
  0x45 /* START_PROP                                                               */,\
  0x83 /* MODEM_RAW_CONTROL,UNSTDPK[7],CONSCHK_BYP[6],PM_PATTERN[3:2],RAWGAIN[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_CONTROL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_EYE { \
  0x11 /* CMD: Set property                     */,\
  0x20 /* GROUP: Modem                          */,\
  0x02 /* NUM_PROPS                             */,\
  0x46 /* START_PROP                            */,\
  0x01 /* MODEM_RAW_EYE,RAWEYE[2:0],RAWEYE[7:0] */,\
  0x8F /* DATA1                                 */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_EYE_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_ANT_DIV_MODE { \
  0x11 /* CMD: Set property                                                                 */,\
  0x20 /* GROUP: Modem                                                                      */,\
  0x01 /* NUM_PROPS                                                                         */,\
  0x48 /* START_PROP                                                                        */,\
  0x02 /* MODEM_ANT_DIV_MODE,SWANT_TIMER[7:6],BYP1P5[5],SKIP2PH[4],SKIP2PHTH[3],ANWAIT[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_ANT_DIV_MODE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_ANT_DIV_CONTROL { \
  0x11 /* CMD: Set property                                                      */,\
  0x20 /* GROUP: Modem                                                           */,\
  0x01 /* NUM_PROPS                                                              */,\
  0x49 /* START_PROP                                                             */,\
  0x00 /* MODEM_ANT_DIV_CONTROL,ANT2PM_THD[7:4],MATAP[3],ANTDIV[2:1],RESERVED[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_ANT_DIV_CONTROL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_JUMP_THRESH { \
  0x11 /* CMD: Set property                      */,\
  0x20 /* GROUP: Modem                           */,\
  0x01 /* NUM_PROPS                              */,\
  0x4B /* START_PROP                             */,\
  0x06 /* MODEM_RSSI_JUMP_THRESH,RSSIJMPTHD[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_JUMP_THRESH_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_CONTROL { \
  0x11 /* CMD: Set property                                                   */,\
  0x20 /* GROUP: Modem                                                        */,\
  0x01 /* NUM_PROPS                                                           */,\
  0x4C /* START_PROP                                                          */,\
  0x09 /* MODEM_RSSI_CONTROL,CHECK_THRESH_AT_LATCH[5],AVERAGE[4:3],LATCH[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_CONTROL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_CONTROL2 { \
  0x11 /* CMD: Set property                                                                     */,\
  0x20 /* GROUP: Modem                                                                          */,\
  0x01 /* NUM_PROPS                                                                             */,\
  0x4D /* START_PROP                                                                            */,\
  0x1C /* MODEM_RSSI_CONTROL2,RSSIJMP_DWN[5],RSSIJMP_UP[4],ENRSSIJMP[3],JMPDLYLEN[2],ENJMPRX[1] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_CONTROL2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_COMP { \
  0x11 /* CMD: Set property              */,\
  0x20 /* GROUP: Modem                   */,\
  0x01 /* NUM_PROPS                      */,\
  0x4E /* START_PROP                     */,\
  0x40 /* MODEM_RSSI_COMP,RSSI_COMP[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_COMP_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0 { \
  0x11 /* CMD: Set property                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 */,\
  0x21 /* GROUP: Modem chflt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                */,\
  0x0C /* NUM_PROPS                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         */,\
  0x00 /* START_PROP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        */,\
  0xFF /* MODEM_CHFLT_RX1_CHFLT_COE,RX1_CHFLT_COE13[1:0],RX1_CHFLT_COE13[7:0],RX1_CHFLT_COE12[3:2],RX1_CHFLT_COE12[7:0],RX1_CHFLT_COE11[5:4],RX1_CHFLT_COE11[7:0],RX1_CHFLT_COE10[7:6],RX1_CHFLT_COE10[7:0],RX1_CHFLT_COE9[1:0],RX1_CHFLT_COE9[7:0],RX1_CHFLT_COE8[3:2],RX1_CHFLT_COE8[7:0],RX1_CHFLT_COE7[5:4],RX1_CHFLT_COE7[7:0],RX1_CHFLT_COE6[7:6],RX1_CHFLT_COE6[7:0],RX1_CHFLT_COE5[1:0],RX1_CHFLT_COE5[7:0],RX1_CHFLT_COE4[3:2],RX1_CHFLT_COE4[7:0],RX1_CHFLT_COE3[5:4],RX1_CHFLT_COE3[7:0],RX1_CHFLT_COE2[7:6],RX1_CHFLT_COE2[7:0],RX1_CHFLT_COE1[1:0],RX1_CHFLT_COE1[7:0],RX1_CHFLT_COE0[3:2],RX1_CHFLT_COE0[7:0] */,\
  0xBA /* DATA1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x0F /* DATA2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x51 /* DATA3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xCF /* DATA4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xA9 /* DATA5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xC9 /* DATA6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xFC /* DATA7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x1B /* DATA8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x1E /* DATA9                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x0F /* DATA10                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            */,\
  0x01 /* DATA11                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0_LEN (16)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1 { \
  0x11 /* CMD: Set property  */,\
  0x21 /* GROUP: Modem chflt */,\
  0x06 /* NUM_PROPS          */,\
  0x0C /* START_PROP         */,\
  0xFC /* DATA0              */,\
  0xFD /* DATA1              */,\
  0x15 /* DATA2              */,\
  0xFF /* DATA3              */,\
  0x00 /* DATA4              */,\
  0x0F /* DATA5              */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1_LEN (10)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0 { \
  0x11 /* CMD: Set property                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 */,\
  0x21 /* GROUP: Modem chflt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                */,\
  0x0C /* NUM_PROPS                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         */,\
  0x12 /* START_PROP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        */,\
  0xFF /* MODEM_CHFLT_RX2_CHFLT_COE,RX2_CHFLT_COE13[1:0],RX2_CHFLT_COE13[7:0],RX2_CHFLT_COE12[3:2],RX2_CHFLT_COE12[7:0],RX2_CHFLT_COE11[5:4],RX2_CHFLT_COE11[7:0],RX2_CHFLT_COE10[7:6],RX2_CHFLT_COE10[7:0],RX2_CHFLT_COE9[1:0],RX2_CHFLT_COE9[7:0],RX2_CHFLT_COE8[3:2],RX2_CHFLT_COE8[7:0],RX2_CHFLT_COE7[5:4],RX2_CHFLT_COE7[7:0],RX2_CHFLT_COE6[7:6],RX2_CHFLT_COE6[7:0],RX2_CHFLT_COE5[1:0],RX2_CHFLT_COE5[7:0],RX2_CHFLT_COE4[3:2],RX2_CHFLT_COE4[7:0],RX2_CHFLT_COE3[5:4],RX2_CHFLT_COE3[7:0],RX2_CHFLT_COE2[7:6],RX2_CHFLT_COE2[7:0],RX2_CHFLT_COE1[1:0],RX2_CHFLT_COE1[7:0],RX2_CHFLT_COE0[3:2],RX2_CHFLT_COE0[7:0] */,\
  0xBA /* DATA1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x0F /* DATA2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x51 /* DATA3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xCF /* DATA4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xA9 /* DATA5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xC9 /* DATA6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0xFC /* DATA7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x1B /* DATA8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x1E /* DATA9                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             */,\
  0x0F /* DATA10                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            */,\
  0x01 /* DATA11                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0_LEN (16)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1 { \
  0x11 /* CMD: Set property  */,\
  0x21 /* GROUP: Modem chflt */,\
  0x06 /* NUM_PROPS          */,\
  0x1E /* START_PROP         */,\
  0xFC /* DATA0              */,\
  0xFD /* DATA1              */,\
  0x15 /* DATA2              */,\
  0xFF /* DATA3              */,\
  0x00 /* DATA4              */,\
  0x0F /* DATA5              */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1_LEN (10)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_SPIKE_DET { \
  0x11 /* CMD: Set property                                       */,\
  0x20 /* GROUP: Modem                                            */,\
  0x01 /* NUM_PROPS                                               */,\
  0x54 /* START_PROP                                              */,\
  0x05 /* MODEM_SPIKE_DET,SPIKE_DETECT_EN[7],SPIKE_THRESHOLD[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_SPIKE_DET_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_CTRL1 { \
  0x11 /* CMD: Set property                                                      */,\
  0x20 /* GROUP: Modem                                                           */,\
  0x01 /* NUM_PROPS                                                              */,\
  0x5B /* START_PROP                                                             */,\
  0x40 /* MODEM_DSA_CTRL1,QUAL_SOURCE[7:6],DSA_EN[5],ADJ_SAMP_ERR_TOLERANCE[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_CTRL1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_CTRL2 { \
  0x11 /* CMD: Set property                                                                           */,\
  0x20 /* GROUP: Modem                                                                                */,\
  0x01 /* NUM_PROPS                                                                                   */,\
  0x5C /* START_PROP                                                                                  */,\
  0x04 /* MODEM_DSA_CTRL2,PREAM_QUAL[7],SYNC_QUAL[6],BCR_GEAR_SHIFT[5],SKIP_PMDET[4],ARRIVAL_THD[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_CTRL2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_ONE_SHOT_AFC { \
  0x11 /* CMD: Set property                                                                                    */,\
  0x20 /* GROUP: Modem                                                                                         */,\
  0x01 /* NUM_PROPS                                                                                            */,\
  0x55 /* START_PROP                                                                                           */,\
  0x07 /* MODEM_ONE_SHOT_AFC,ONESHOT_AFC_EN[7],BCR_ALIGN_EN[6],EST_OSR_EN[5],AFCMA_EN[4],ONESHOT_WAIT_CNT[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_ONE_SHOT_AFC_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_QUAL { \
  0x11 /* CMD: Set property                           */,\
  0x20 /* GROUP: Modem                                */,\
  0x01 /* NUM_PROPS                                   */,\
  0x5D /* START_PROP                                  */,\
  0x0C /* MODEM_DSA_QUAL,EYE_QUAL_SEL[7],ARRQUAL[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_QUAL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_RSSI { \
  0x11 /* CMD: Set property                                    */,\
  0x20 /* GROUP: Modem                                         */,\
  0x01 /* NUM_PROPS                                            */,\
  0x5E /* START_PROP                                           */,\
  0x78 /* MODEM_DSA_RSSI,SQUELCH_EN[7],DSA_RSSI_THRESHOLD[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_RSSI_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG2 { \
  0x11 /* CMD: Set property                        */,\
  0x20 /* GROUP: Modem                             */,\
  0x01 /* NUM_PROPS                                */,\
  0x20 /* START_PROP                               */,\
  0x00 /* MODEM_RESERVED_20_20,RESERVED_20_20[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_MUTE { \
  0x11 /* CMD: Set property                                 */,\
  0x20 /* GROUP: Modem                                      */,\
  0x01 /* NUM_PROPS                                         */,\
  0x57 /* START_PROP                                        */,\
  0x00 /* MODEM_RSSI_MUTE,RSSI_DELAY[3],RSSI_DELAY_CNT[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_MUTE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_MISC { \
  0x11 /* CMD: Set property                                                           */,\
  0x20 /* GROUP: Modem                                                                */,\
  0x01 /* NUM_PROPS                                                                   */,\
  0x5F /* START_PROP                                                                  */,\
  0x20 /* MODEM_DSA_MISC,CC_ASSESS_SEL[7],EYEXEST_EN[6],EYEXEST_FAST[5],LOW_DUTY[2:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_MISC_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_CLK_CFG { \
  0x11 /* CMD: Set property                                                      */,\
  0x00 /* GROUP: Global                                                          */,\
  0x01 /* NUM_PROPS                                                              */,\
  0x01 /* START_PROP                                                             */,\
  0x02 /* GLOBAL_CLK_CFG,DIVIDED_CLK_EN[6],DIVIDED_CLK_SEL[5:3],CLK_32K_SEL[1:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_CLK_CFG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_CONFIG { \
  0x11 /* CMD: Set property                                                                                                                                           */,\
  0x00 /* GROUP: Global                                                                                                                                               */,\
  0x01 /* NUM_PROPS                                                                                                                                                   */,\
  0x03 /* START_PROP                                                                                                                                                  */,\
  0x20 /* GLOBAL_CONFIG,RESERVED[6],SEQUENCER_MODE[5],FIFO_MODE[4],PROTOCOL[3:1],POWER_MODE[0],RESERVED[6],SEQUENCER_MODE[5],FIFO_MODE[4],PROTOCOL[3:1],POWER_MODE[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_CONFIG { \
  0x11 /* CMD: Set property                                                                       */,\
  0x00 /* GROUP: Global                                                                           */,\
  0x01 /* NUM_PROPS                                                                               */,\
  0x04 /* START_PROP                                                                              */,\
  0x06 /* GLOBAL_WUT_CONFIG,WUT_LDC_EN[7:6],WUT_CAL_PERIOD[5:3],WUT_LBD_EN[2],WUT_EN[1],CAL_EN[0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_CONFIG_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_M { \
  0x11 /* CMD: Set property                  */,\
  0x00 /* GROUP: Global                      */,\
  0x02 /* NUM_PROPS                          */,\
  0x05 /* START_PROP                         */,\
  0x00 /* GLOBAL_WUT_M,WUT_M[7:0],WUT_M[7:0] */,\
  0x00 /* DATA1                              */\
}

#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_M_LEN (6)


#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_R { \
  0x11 /* CMD: Set property                                            */,\
  0x00 /* GROUP: Global                                                */,\
  0x01 /* NUM_PROPS                                                    */,\
  0x07 /* START_PROP                                                   */,\
  0x60 /* GLOBAL_WUT_R,RESERVED_WRITE_ONE[7:6],WUT_SLEEP[5],WUT_R[4:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_R_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_LDC { \
  0x11 /* CMD: Set property           */,\
  0x00 /* GROUP: Global               */,\
  0x01 /* NUM_PROPS                   */,\
  0x08 /* START_PROP                  */,\
  0x00 /* GLOBAL_WUT_LDC,WUT_LDC[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_GLOBAL_WUT_LDC_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_CONTROL { \
  0x11 /* CMD: Set property                            */,\
  0x50 /* GROUP: Rx hop                                */,\
  0x01 /* NUM_PROPS                                    */,\
  0x00 /* START_PROP                                   */,\
  0x04 /* RX_HOP_CONTROL,HOP_EN[6:4],RSSI_TIMEOUT[3:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_CONTROL_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_SIZE { \
  0x11 /* CMD: Set property                        */,\
  0x50 /* GROUP: Rx hop                            */,\
  0x01 /* NUM_PROPS                                */,\
  0x01 /* START_PROP                               */,\
  0x01 /* RX_HOP_TABLE_SIZE,RX_HOP_TABLE_SIZE[6:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_SIZE_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_0 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x02 /* START_PROP                          */,\
  0x00 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_0_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_1 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x03 /* START_PROP                          */,\
  0x01 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_1_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_2 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x04 /* START_PROP                          */,\
  0x02 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_2_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_3 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x05 /* START_PROP                          */,\
  0x03 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_3_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_4 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x06 /* START_PROP                          */,\
  0x04 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_4_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_5 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x07 /* START_PROP                          */,\
  0x05 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_5_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_6 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x08 /* START_PROP                          */,\
  0x06 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_6_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_7 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x09 /* START_PROP                          */,\
  0x07 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_7_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_8 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x0A /* START_PROP                          */,\
  0x08 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_8_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_9 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x0B /* START_PROP                          */,\
  0x09 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_9_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_10 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x0C /* START_PROP                          */,\
  0x0A /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_10_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_11 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x0D /* START_PROP                          */,\
  0x0B /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_11_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_12 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x0E /* START_PROP                          */,\
  0x0C /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_12_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_13 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x0F /* START_PROP                          */,\
  0x0D /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_13_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_14 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x10 /* START_PROP                          */,\
  0x0E /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_14_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_15 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x11 /* START_PROP                          */,\
  0x0F /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_15_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_16 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x12 /* START_PROP                          */,\
  0x10 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_16_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_17 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x13 /* START_PROP                          */,\
  0x11 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_17_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_18 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x14 /* START_PROP                          */,\
  0x12 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_18_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_19 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x15 /* START_PROP                          */,\
  0x13 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_19_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_20 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x16 /* START_PROP                          */,\
  0x14 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_20_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_21 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x17 /* START_PROP                          */,\
  0x15 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_21_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_22 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x18 /* START_PROP                          */,\
  0x16 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_22_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_23 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x19 /* START_PROP                          */,\
  0x17 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_23_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_24 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x1A /* START_PROP                          */,\
  0x18 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_24_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_25 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x1B /* START_PROP                          */,\
  0x19 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_25_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_26 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x1C /* START_PROP                          */,\
  0x1A /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_26_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_27 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x1D /* START_PROP                          */,\
  0x1B /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_27_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_28 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x1E /* START_PROP                          */,\
  0x1C /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_28_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_29 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x1F /* START_PROP                          */,\
  0x1D /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_29_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_30 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x20 /* START_PROP                          */,\
  0x1E /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_30_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_31 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x21 /* START_PROP                          */,\
  0x1F /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_31_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_32 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x22 /* START_PROP                          */,\
  0x20 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_32_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_33 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x23 /* START_PROP                          */,\
  0x21 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_33_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_34 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x24 /* START_PROP                          */,\
  0x22 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_34_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_35 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x25 /* START_PROP                          */,\
  0x23 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_35_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_36 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x26 /* START_PROP                          */,\
  0x24 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_36_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_37 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x27 /* START_PROP                          */,\
  0x25 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_37_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_38 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x28 /* START_PROP                          */,\
  0x26 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_38_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_39 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x29 /* START_PROP                          */,\
  0x27 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_39_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_40 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x2A /* START_PROP                          */,\
  0x28 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_40_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_41 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x2B /* START_PROP                          */,\
  0x29 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_41_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_42 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x2C /* START_PROP                          */,\
  0x2A /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_42_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_43 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x2D /* START_PROP                          */,\
  0x2B /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_43_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_44 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x2E /* START_PROP                          */,\
  0x2C /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_44_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_45 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x2F /* START_PROP                          */,\
  0x2D /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_45_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_46 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x30 /* START_PROP                          */,\
  0x2E /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_46_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_47 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x31 /* START_PROP                          */,\
  0x2F /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_47_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_48 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x32 /* START_PROP                          */,\
  0x30 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_48_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_49 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x33 /* START_PROP                          */,\
  0x31 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_49_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_50 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x34 /* START_PROP                          */,\
  0x32 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_50_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_51 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x35 /* START_PROP                          */,\
  0x33 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_51_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_52 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x36 /* START_PROP                          */,\
  0x34 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_52_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_53 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x37 /* START_PROP                          */,\
  0x35 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_53_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_54 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x38 /* START_PROP                          */,\
  0x36 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_54_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_55 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x39 /* START_PROP                          */,\
  0x37 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_55_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_56 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x3A /* START_PROP                          */,\
  0x38 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_56_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_57 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x3B /* START_PROP                          */,\
  0x39 /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_57_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_58 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x3C /* START_PROP                          */,\
  0x3A /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_58_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_59 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x3D /* START_PROP                          */,\
  0x3B /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_59_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_60 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x3E /* START_PROP                          */,\
  0x3C /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_60_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_61 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x3F /* START_PROP                          */,\
  0x3D /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_61_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_62 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x40 /* START_PROP                          */,\
  0x3E /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_62_LEN (5)


#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_63 { \
  0x11 /* CMD: Set property                   */,\
  0x50 /* GROUP: Rx hop                       */,\
  0x01 /* NUM_PROPS                           */,\
  0x41 /* START_PROP                          */,\
  0x3F /* RX_HOP_TABLE_ENTRY,CHANNEL_NUM[7:0] */\
}

#define RADIO_CONFIG_SET_PROPERTY_RX_HOP_TABLE_ENTRY_63_LEN (5)


#define RADIO_CONFIG_COUNT (234)


#endif // __SLAB_RADIO_CONFIG__


//        _  _                          
//       | )/ )         Wireless        
//    \\ |//,' __       Application     
//    (")(_)-"()))=-    Software        
//       (\\            Platform        
