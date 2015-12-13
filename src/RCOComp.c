/*
 * RCOComp.c
 *
 *  Created on: 13/12/2015
 *      Author: kentm
 */
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "em_adc.h"
#include "em_cmu.h"
#include "RCOComp.h"
// ******************** defines and typedefs *************************
/* EFM32 temperature sensor gradient */
#define THERMOMETER_GRADIENT  (-6.3)

#define HFRCO_NOMINAL_TEMP                25
#define HFRCO_MIN_TEMP                    -40
#define HFRCO_MAX_TEMP                    85
#define HFRCO_TABLE_LENGTH (HFRCO_MAX_TEMP-HFRCO_MIN_TEMP)
#define HFRCO_TABLE_SHIFT 10

#define LIMIT(a,min,max) (a<min)?(a=min):((a>max)?(a=max):((void)a))
typedef union{
	uint8_t b[4];
	uint16_t w[2];
	uint32_t L;
} longin_t;
// ******************** local variables ******************************
// The table is in the format of % difference in hfrco value from factory ROM stored value
// table goes from -40 to +85 DegC, DI tables are calibrated @ 25DegC Vcc = 3V
// step size for lsb bit of tuning register is 0.3%
// need to scale for this well, increasing value increases frequency
// so colder temps where frequency is low will need to increase tuning value and vice versa
// actual points are calculated as bits offset from these minimal set of points
// deg	-40  ,-15  ,5    ,25  ,45   ,65   ,85
// Mhz  27.35,27.62,27.84,28.0,28.07,28.09,28.055
// N      8  ,  5  ,  2  ,  0 , -1  , -1  , -1
// TODO test that this works, and that 1 bit means 1 code and  not two codes/bit
static const int8_t RCOCompensationTable[HFRCO_TABLE_LENGTH]    =
{
	//1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
		8 ,8 ,7 ,7 ,7 ,7 ,7 ,7 ,7 ,7 ,6 ,6 ,6 ,6 ,6 ,6 ,6 ,6 ,5 ,5 ,5,5,5,5,5,
		5 ,4 ,4 ,4 ,4 ,4 ,4 ,4 ,3 ,3 ,3 ,3 ,3 ,3 ,3 ,3 ,2 ,2 ,2 ,2 ,
		2 ,2 ,2 ,2 ,2 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,0 ,0 ,0 ,0 ,0 ,
		0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,-1,-1,-1,-1,-1,-1,-1,-1,
		-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
		-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
};
static uint8_t    thermCalTemp;
static uint16_t   thermCalValue;
static int16_t 		DefTuningVal=0;
// ******************** local function prototypes ********************
static int8_t ReadThermometer(void);
// ********************* Implementation ******************************
void InitRCOCalibration(void)
{
  /* Read thermometer calibration temperature from DI page */
  thermCalTemp = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);
  /* Read thermometer calibration value from DI page */
  thermCalValue = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
#if defined(_EFM32_GIANT_FAMILY)
  /* This is a work around for Chip Rev.D Errata, Revision 0.6. */
  uint8_t prodRev = (DEVINFO->PART & _DEVINFO_PART_PROD_REV_MASK) >> _DEVINFO_PART_PROD_REV_SHIFT;
  if( (prodRev == 16) || (prodRev == 17) )
  {
      thermCalValue -= 112;
  }
#endif
  DefTuningVal = (DEVINFO->HFRCOCAL1 & _DEVINFO_HFRCOCAL1_BAND28_MASK) >> _DEVINFO_HFRCOCAL1_BAND28_SHIFT;
  /* Enable ADC peripheral clock */
  CMU_ClockEnable(cmuClock_ADC0, true);
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  /* Initialize timebases */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(400000,0);
  ADC_Init(ADC0, &init);
  UpdateRCOCalibration();
}
// read adc for temperatue and calibrate RCO based on that
// maybe every 5 degree change
void UpdateRCOCalibration(void)
{
  int8_t temperature;
  int8_t temperatureOffset;
  temperature = ReadThermometer();
  temperatureOffset = ( temperature - HFRCO_MIN_TEMP );
  LIMIT(temperatureOffset,0,HFRCO_TABLE_LENGTH);
  int16_t Val = DefTuningVal +RCOCompensationTable[temperatureOffset];
  LIMIT(Val,0,0xff);
  CMU_OscillatorTuningSet(cmuOsc_HFRCO, Val);
}

static int8_t ReadThermometer(void)
{
  uint32_t sample;
  double  temp;
	static const ADC_InitSingle_TypeDef init =
  { adcPRSSELCh0,              /* PRS ch0 (if enabled). */
    adcAcqTime2,               /* min 2uS, clock base is 400K 2.5uS*/
    adcRef1V25,                /* 1.25V internal reference. */
    adcRes12Bit,               /* 12 bit resolution. */
    adcSingleInpTemp,           /* temp input selected. */
    false,                     /* Single ended input. */
    false,                     /* PRS disabled. */
    false,                     /* Right adjust. */
    false                      /* Deactivate conversion after one scan sequence. */
  };
	ADC_InitSingle(ADC0,&init);
  ADC_Start(ADC0, adcStartSingle);
  while ( !(ADC0->STATUS & ADC_STATUS_SINGLEDV ) );
  sample = ADC_DataSingleGet(ADC0);
  temp = thermCalTemp - (((double)thermCalValue -(double)sample)/THERMOMETER_GRADIENT);
  return (int8_t)round(temp);
}

// ********************* end of RCOComp.c ****************************
