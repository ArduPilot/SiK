
///
/// @file	pins_user.c
///
/// AT Controlled Pins
///

#include <stdint.h>
#include "pins_user.h"
#include "parameters.h"
#include "em_gpio.h"
#include "em_timer.h"

#if PIN_MAX > 0

typedef struct {
  GPIO_Port_Pin_TypeDef   GpioData;
  GPIO_Mode_TypeDef   Mode;
  unsigned int        DefaultState;
} GPIO_Config_t;

typedef struct {
  GPIO_Port_Pin_TypeDef   GpioData;
} PINS_USER_Config_t;

typedef enum {
  PINS_USER_P1_0				,
  PINS_USER_P1_1				,
  PINS_USER_P1_2				,
  PINS_USER_P1_3				,
  PINS_USER_P3_3				,
  PINS_USER_P3_4				,
  PINS_USER_LAST
} PINS_USER_TypeDef;


GPIO_Config_t gpioValues[GPIO_LAST] =
{   {LED_RED	   ,gpioModePushPull,0},
		{LED_GREEN   ,gpioModePushPull,0},
		{PIN_CONFIG	 ,gpioModeInputPull,1},
		{PIN_ENABLE  ,gpioModePushPull,0},
		{SERIAL_CTS  ,gpioModePushPull,0},
		{SERIAL_RTS  ,gpioModeInputPull,0},

};

PINS_USER_Config_t pins_user_map[PINS_USER_LAST] =
{		{P1_0},
		{P1_1},
		{P1_2},
		{P1_3},
		{P3_3},
		{P3_4},
};


void pins_user_init(void)
{
	uint8_t i;
	
	// Set the Default pin behaviour
	for(i=0; i<PIN_MAX; i++)
	{
		pins_user_set_io(i, pin_values[i].output);
		pins_user_set_value(i, pin_values[i].pin_dir);
	}
	for(i=0; i<GPIO_LAST; i++)
	{
		GPIO_PinModeSet(gpioValues[i].GpioData.Port,gpioValues[i].GpioData.Pin,
				gpioValues[i].Mode, gpioValues[i].DefaultState);
	}
// Client Application Hack
//	pins_user_set_value(3,PIN_HIGH);
}

bool pins_user_set_io(uint8_t pin, bool in_out)
{
	if (PIN_MAX > pin)
	{
		pin_values[pin].output = in_out;
		pin_values[pin].pin_mirror = PIN_NULL;
		
		GPIO_PinModeSet(pins_user_map[pin].GpioData.Port,pins_user_map[pin].GpioData.Pin,
				(in_out)?(gpioModePushPull):(gpioModeInputPull),(in_out)?(0):(1));
		return true;
	}
	return false;
}

bool pins_user_get_io(uint8_t pin)
{
	return pin_values[pin].output;
}

bool pins_user_set_value(uint8_t pin, bool high_low)
{
	pin_values[pin].pin_dir = high_low;
	if(PIN_MAX > pin && pin_values[pin].pin_mirror == PIN_NULL)
	{
		if(high_low)
		{
			GPIO_PinOutSet(pins_user_map[pin].GpioData.Port, pins_user_map[pin].GpioData.Pin);
		}
		else
		{
			GPIO_PinOutClear(pins_user_map[pin].GpioData.Port, pins_user_map[pin].GpioData.Pin);
		}
		return true;
	}
	return false;
}

bool pins_user_get_value(uint8_t pin)
{
	return pin_values[pin].pin_dir;
}

uint8_t pins_user_get_adc(uint8_t pin)
{
	if(PIN_MAX > pin && pin_values[pin].output == PIN_INPUT)
	{
		return(GPIO_PinInGet(pins_user_map[pin].GpioData.Port,pins_user_map[pin].GpioData.Pin));
	}
	return PIN_ERROR;
}

void pins_user_check()
{
//	static uint8_t p, p_count;
//	if (pins_user_get_adc(5) != p || p_count != 0) {
//		if(pins_user_get_adc(5) != p)
//		{
//			p = pins_user_get_adc(5);
//			p_count = 100;
//		}
//		at_cmd[0] = 'R';
//		at_cmd[1] = 'T';
//		at_cmd[2] = 'P';
//		at_cmd[3] = 'C';
//		at_cmd[4] = '=';
//		at_cmd[5] = '4';
//		at_cmd[6] = ',';
//		at_cmd[7] = '0'+(p?1:0);
//		at_cmd[8] = 0;
//		tdm_remote_at();
//		p_count --;
//	}
}

uint8_t GPIOGet(GPIO_SELECT_TypeDef select)
{
	if(select < GPIO_LAST)
	{
		return(GPIO_PinInGet(gpioValues[select].GpioData.Port, gpioValues[select].GpioData.Pin));
	}
	return 0;
}
void GPIOSet(GPIO_SELECT_TypeDef select,uint8_t ONOFF)
{
	if(select < GPIO_LAST)
	{
		(ONOFF)?(GPIO_PinOutSet(gpioValues[select].GpioData.Port, gpioValues[select].GpioData.Pin)):
				(GPIO_PinOutClear(gpioValues[select].GpioData.Port, gpioValues[select].GpioData.Pin));
	}
}
void GPIOToggle(GPIO_SELECT_TypeDef select)
{
	if(select < GPIO_LAST)
	{
		GPIO_PinOutToggle(gpioValues[select].GpioData.Port, gpioValues[select].GpioData.Pin);
	}
}

void LED_RADIO_TOGGLE(void)
{
	GPIOToggle(GPIO_LED_RADIO);
}

void LED_RADIO(uint8_t LED_ONOFF)
{
	GPIOSet(GPIO_LED_RADIO,LED_ONOFF);
}
void LED_ACTIVITY_TOGGLE(void)
{
	GPIOToggle(GPIO_LED_ACTIVITY);
}
void LED_ACTIVITY(uint8_t LED_ONOFF)
{
	GPIOSet(GPIO_LED_ACTIVITY,LED_ONOFF);
}

#endif // PIN_MAX > 0
