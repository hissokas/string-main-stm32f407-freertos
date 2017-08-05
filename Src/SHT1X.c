#include "SHT1X.h"

//-----------------------------------------------------------
//��SHT10���ж�д����
/*
SHT1Xϵ�йܽ�˳��
1-->GND
2-->DATA   PD9
3-->SCK    PD10
4-->VCC
5-->NC
*/
//-----------------------------------------------------------


//��ʼ��SHT1X,��������
void SHT1x_Init(void)
{
	//ͨ�紫������Ҫ11ms��������״̬���ڴ�֮ǰ������Դ����������κ�����
	SHT1x_Reset();
}

/*=========================================
�� �� ����u8 SHT1x_Write_Byte(u8 value)�ֽڴ��ͺ���
��    ��������value Ҫ���͵�����
������������Ԫ����SHT10����һ���ֽڵ�����
�� �� ֵ������ֵerror error��0��������ȷ
					  error��1�����ʹ���
=========================================*/
uint8_t SHT1x_Write_Byte(uint8_t value)
{
	uint8_t i=0,error=0;
	
	SDA_SET_OUT();
	for(i=0x80;i>0;i/=2) 
	{
		if(i&value) SDA_Pin_OUT(1); //ѭ�����룬�����ΪҪ���͵�λ
		else SDA_Pin_OUT(0);
		
		delay_us();
		SCL_Pin_OUT(1);
		delay_us();
		SCL_Pin_OUT(0);
		delay_us();
	}

	SDA_SET_IN();//SDA�˿�ģʽΪ��������
	SCL_Pin_OUT(1);
	error = SDA_Pin_IN();
	SCL_Pin_OUT(0);

	return error;  //error��1��ͨѶ����;error��0��ͨѶ�ɹ�
}

/*=========================================
�� �� ����SHT1x_Read_Byte(u8 dat)�����ݺ���
��    ��������dat=0,������һ���ֽڵĻظ�
              dat=1,ֹͣͨѶ
��������������Ԫ����SHT10���͵�������һ���ֽڵ����ݣ�
�� �� ֵ������ֵval	��Ϊ���ܵ�������
=========================================*/
uint8_t SHT1x_Read_Byte(uint8_t dat)
{
	uint8_t i=0,val=0;
	
	SDA_SET_IN();
	for(i = 0x80;i > 0;i /= 2)
	{
		delay_us();
		SCL_Pin_OUT(1);
		delay_us();
		if(SDA_Pin_IN())
			val = (val | i);
		SCL_Pin_OUT(0);
	}
	
	SDA_SET_OUT();
	if(dat) SDA_Pin_OUT(0);
	else SDA_Pin_OUT(1);
	delay_us();
	
	SCL_Pin_OUT(1);
	delay_us();
	SCL_Pin_OUT(0);
	delay_us();
	
	return val;
}

//���������ź�
/*ʱ��ͼ��
        ____    ____
SCL: __|    |__|    |__
     ____         _____
SDA:     |_______|   
*/
void SHT1x_Start(void)
{
	SDA_SET_OUT(); //����SDA�˿�Ϊ���ģʽ
    
	SDA_Pin_OUT(1);
	SCL_Pin_OUT(0);
	delay_us();
	SCL_Pin_OUT(1);   
	delay_us();
	
	SDA_Pin_OUT(0);
	delay_us();
	SCL_Pin_OUT(0);
	delay_us();
	SCL_Pin_OUT(1);
	delay_us();
    
	SDA_Pin_OUT(1);	
	delay_us();
	SCL_Pin_OUT(0);
}

//ͨѶ��λʱ��
/*ʱ��ͼ��
        __    __    __    __    __    __    __    __    __    __
SCL: __|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__
     ______________________________________________________       _____ 
SDA:                                                       |______|
*/
void SHT1x_Reset(void)
{
	uint8_t i;
  SDA_SET_OUT(); //����SDA�˿�Ϊ���ģʽ
	SDA_Pin_OUT(1);	    
  SCL_Pin_OUT(0);

	for(i = 0;i < 9;i ++)
	{
		SCL_Pin_OUT(0);
    delay_us();
		SCL_Pin_OUT(1);
    delay_us();
	}
  SHT1x_Start(); //��������
}
/*=========================================
�� �� ����char s_softreset(void)�����λ����
��    ������
����������resets the sensor by a softreset 
�� �� ֵ��error
=========================================*/
char SHT1X_softreset(void)
{ 
	unsigned char error = 0;  
	SHT1x_Reset();              //reset communication
	error += SHT1x_Write_Byte(RESET);       //send RESET-command to sensor
	return error;                     //error=1 in case of no response form the sensor
}
/*=========================================
�� �� ����char s_write_statusreg(unsigned char value)д״̬����
��    ��������value
���������� writes the status register with checksum (8-bit)
�� �� ֵ��error
=========================================*/
char SHT1X_write_statusreg(unsigned char value)
{ 
	unsigned char error=0;
	SHT1x_Start();                   //transmission start
	error += SHT1x_Write_Byte(STATUS_REG_W);//send command to sensor
	error += SHT1x_Write_Byte(value);    //send value of status register
	return error;                     //error>=1 in case of no response form the sensor
}

/*=========================================
�� �� ����char s_write_statusreg(unsigned char value)д״̬����
��    ��������value
����������reads the status register with checksum (8-bit)
�� �� ֵ��error
=========================================*/
char SHT1X_read_statusreg(unsigned char *p_value)
{ 
	unsigned char error = 0;
	SHT1x_Start();                    //transmission start
	error = SHT1x_Write_Byte(STATUS_REG_R); //send command to sensor
	*p_value = SHT1x_Write_Byte(ACK);        //read status register (8-bit)
	SHT1x_Read_Byte(noACK);   //read checksum (8-bit)  
	return error;                     //error=1 in case of no response form the sensor
}

/*=========================================
�� �� ����u8 SHT1x_Measure(u8 *p_value, u8 *p_checksum, u8 Mode)��������
��    ��������mode	ȷ���ǲ����¶Ȼ����ǲ���ʪ��
��������������ָ�SHT10ִ���¶Ⱥ�ʪ�ȵĲ���ת��
�� �� ֵ����
=========================================*/
uint8_t SHT1x_Measure(uint16_t *p_value, uint8_t *p_checksum, uint8_t Mode)
{
	uint8_t error=0,Value_H=0,Value_L=0;
	
	SHT1x_Start();//��ʼ����ת��
	
	switch(Mode)
	{
		case 0: error += SHT1x_Write_Byte(MEASURE_TEMP);break; //���¶ȵ�ֵ  
		case 1: error += SHT1x_Write_Byte(MEASURE_HUMI);break; //��ʪ�ȵ�ֵ
		default:break;
	} 
	
	SDA_SET_IN();//����SDA�˿�Ϊ����ģʽ
	HAL_Delay(300);//�ȴ��������
	
	if(SDA_Pin_IN()) error += 1;//����ʱ��������DQû���ͣ���˵�������д���
	else
	{
		Value_H = SHT1x_Read_Byte(ACK);		 //���ݵĸ��ֽ�
		Value_L = SHT1x_Read_Byte(ACK);	     //���ݵĵ��ֽ�    		
		*p_value = (Value_H << 8) | Value_L;
		*p_checksum = SHT1x_Read_Byte(noACK);	     //CRCУ����	
	}
	
	return error;
}

/*=========================================
�� �� ����void SHT1X_Caculation(float *p_temperature,float *p_humidity )���ݴ�����
��    �����޲���
�����������¶Ⱥ�ʪ�Ȳ���������¶�ֵ�����ʪ��ֵ
�� �� ֵ���޷���ֵ
=========================================*/
void SHT1X_Caculation(float *p_temperature,float *p_humidity )
{
  const float c1=-4.0f;
  const float c2=+0.0405f;
	const float c3=-0.0000028f;			//����Ϊ12λʪ��������ʾȡֵ
	const float t1=+0.01f;
	const float t2=+0.00008f;			//����Ϊ14λ�¶�������ʾȡֵ

	
	float T=*p_temperature;
	float H=*p_humidity;
	float H_nonline_Compensation;
	float Humi_Comp;
	float Temp_Comp;
	
	Temp_Comp=T * 0.01f - 39.6f;					          //�¶ȵĲ���
		
	H_nonline_Compensation=c3*H*H  + c2*H + c1;                  //���ʪ�ȷ����Բ���
  Humi_Comp=( Temp_Comp - 25 ) * ( t1 + t2*H ) + H_nonline_Compensation; //���ʪ�ȶ����¶������Բ���
	
	if( Humi_Comp > 100 ) Humi_Comp=100;			  //���ʪ�����ֵ����
	if( Humi_Comp < 0.1f ) Humi_Comp=0.1f;			  //���ʪ����Сֵ����
	 
	*p_temperature=Temp_Comp;			//�����¶Ȳ�����Ľ��
	*p_humidity=Humi_Comp; 						  //�������ʪ�Ȳ�����Ľ��
}
/*=========================================
�� �� ����void SHT1X_Caculation1(float *p_temperature,float *p_humidity )���ݴ�����
��    �����޲���
�����������¶Ⱥ�ʪ�Ȳ���������¶�ֵ�����ʪ��ֵ
�� �� ֵ���޷���ֵ
=========================================*/
void SHT1X_Caculation1(float *p_temperature,float *p_humidity )
{
  const int c1=40000000;
  const int c2=405000;
	const int c3=28;			//����Ϊ12λʪ��������ʾȡֵ
	const int t1=1;
	const int t2=8;			//����Ϊ14λ�¶�������ʾȡֵ

	
	float T=*p_temperature;
	float H=*p_humidity;
	float H_nonline_Compensation;
	int Humi_Comp;
	int Temp_Comp;
	
	Temp_Comp=(signed long)(T * t1 - 3960);					          //�¶ȵĲ���
		
	H_nonline_Compensation=(c2*H -c3* H*H-c1)/100000;                  //���ʪ�ȷ����Բ���
  Humi_Comp=(float)( Temp_Comp - 2500 ) * ( t1 + t2*H )/100000 + H_nonline_Compensation; //���ʪ�ȶ����¶������Բ���
	
	if( Humi_Comp > 10000 ) Humi_Comp=10000;			  //���ʪ�����ֵ����
	if( Humi_Comp < 10 ) Humi_Comp=10;			  //���ʪ����Сֵ����
	 
	*p_temperature=Temp_Comp;			//�����¶Ȳ�����Ľ��
	*p_humidity=Humi_Comp; 						  //�������ʪ�Ȳ�����Ľ��
}

void delay_us()
{
  uint8_t i = 0;  
  uint8_t delay = 5;  
      
  while (delay--)  
  {  
		i = 10;  
    while (i--);  
  } 
}