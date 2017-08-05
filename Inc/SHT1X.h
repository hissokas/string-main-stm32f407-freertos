#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f4xx_hal.h"

#define IIC_SDA0    GPIOD->BSRR=(uint32_t)GPIO_PIN_9<<16U
#define IIC_SDA1    GPIOD->BSRR=GPIO_PIN_9
#define IIC_SCL0    GPIOD->BSRR=(uint32_t)GPIO_PIN_10<<16U
#define IIC_SCL1    GPIOD->BSRR=GPIO_PIN_10

#define SDA_SET_IN()	  GPIOD->MODER &= ~(0x03 << 18)
#define SDA_SET_OUT()   GPIOD->MODER |= (0X01 << 18)

#define noACK 0            
#define ACK   1            
                            //adr  command  r/w 
#define STATUS_REG_W 0x06   //000   0011    0 
#define STATUS_REG_R 0x07   //000   0011    1 
#define MEASURE_TEMP 0x03   //000   0001    1 
#define MEASURE_HUMI 0x05   //000   0010    1 
#define RESET        0x1e   //000   1111    0 

#define SCL_Pin_OUT(x) do{if(x){IIC_SCL1;}else{IIC_SCL0;}}while(0)
#define SDA_Pin_OUT(x) do{if(x){IIC_SDA1;}else{IIC_SDA0;}}while(0)
#define SDA_Pin_IN() ((GPIOD->IDR&GPIO_PIN_9)!=0)?1:0

typedef struct strctSHT1x
{
	int Temperature;//转换的温度
	uint16_t Humidity;   //转换的湿度
  float T_Result;
  float H_Result;
	uint8_t CRC8_Temperature; //校验码
	uint8_t CRC8_Humidity;  //校验码
}SHT1x;

void SHT1x_Init(void);
void SHT1x_Start(void);
void SHT1x_Reset(void);
char SHT1X_softreset(void);
char SHT1X_write_statusreg(unsigned char value);
char SHT1X_read_statusreg(unsigned char *p_value);
uint8_t SHT1x_Measure(uint16_t *p_value, uint8_t *p_checksum, uint8_t Mode);
uint8_t SHT1x_Write_Byte(uint8_t value);
uint8_t SHT1x_Read_Byte(uint8_t dat);
void SHT1X_Caculation(float *p_temperature,float *p_humidity );
void SHT1X_Caculation1(float *p_temperature,float *p_humidity );
void delay_us(void);

#endif
