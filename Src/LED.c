#include "LED.h"
#include "arm_math.h"

extern uint8_t sensor_online[LEDn];
extern float sensor_frequency[LEDn];
extern float sensor_temp[LEDn];
float sensor_last_frequency[LEDn];

#if LEDn == 32
	GPIO_TypeDef* LED_PORT[LEDn] = {GPIOB,GPIOE,GPIOE,GPIOC,GPIOA,GPIOA,GPIOB,GPIOE,
																	GPIOB,GPIOE,GPIOE,GPIOC,GPIOC,GPIOA,GPIOC,GPIOE, 
																	GPIOB,GPIOE,GPIOE,GPIOC,GPIOC,GPIOA,GPIOC,GPIOE,
																	GPIOB,GPIOB,GPIOE,GPIOC,GPIOC,GPIOA,GPIOA,GPIOB};
	uint16_t LED_PIN[LEDn] = {GPIO_PIN_8,GPIO_PIN_2,GPIO_PIN_6,GPIO_PIN_0,GPIO_PIN_0,GPIO_PIN_6,GPIO_PIN_0,GPIO_PIN_9,
																	GPIO_PIN_7,GPIO_PIN_1,GPIO_PIN_5,GPIO_PIN_15,GPIO_PIN_3,GPIO_PIN_5,GPIO_PIN_5,GPIO_PIN_8, 
																	GPIO_PIN_6,GPIO_PIN_0,GPIO_PIN_4,GPIO_PIN_14,GPIO_PIN_2,GPIO_PIN_4,GPIO_PIN_4,GPIO_PIN_7,
																	GPIO_PIN_5,GPIO_PIN_9,GPIO_PIN_3,GPIO_PIN_13,GPIO_PIN_1,GPIO_PIN_1,GPIO_PIN_7,GPIO_PIN_1};

#else
	GPIO_TypeDef* LED_PORT[LEDn] = {GPIOE,GPIOC,GPIOA,GPIOE,
																	GPIOE,GPIOC,GPIOA,GPIOE, 
																	GPIOE,GPIOC,GPIOA,GPIOE,
																	GPIOB,GPIOC,GPIOA,GPIOB};
	uint16_t LED_PIN[LEDn] = {GPIO_PIN_2,GPIO_PIN_0,GPIO_PIN_6,GPIO_PIN_9,
																	GPIO_PIN_1,GPIO_PIN_15,GPIO_PIN_5,GPIO_PIN_8, 
																	GPIO_PIN_0,GPIO_PIN_14,GPIO_PIN_4,GPIO_PIN_7,
																	GPIO_PIN_9,GPIO_PIN_13,GPIO_PIN_1,GPIO_PIN_1};													 
#endif



void test_sensor()
{
	int i;
	for(i = 0;i < LEDn;i ++)
	{
		if((uint16_t)(sensor_temp[i]) != 0)
		{
			sensor_online[i] = 1;
		}
		else sensor_online[i] = 0;
		sensor_last_frequency[i] = sensor_frequency[i];
	}
}
