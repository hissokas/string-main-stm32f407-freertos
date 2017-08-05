#include "SHT1X.h"

//-----------------------------------------------------------
//对SHT10进行读写操作
/*
SHT1X系列管脚顺序：
1-->GND
2-->DATA   PD9
3-->SCK    PD10
4-->VCC
5-->NC
*/
//-----------------------------------------------------------


//初始化SHT1X,配置引脚
void SHT1x_Init(void)
{
	//通电传感器需要11ms进入休眠状态，在此之前不允许对传感器发送任何命令
	SHT1x_Reset();
}

/*=========================================
函 数 名：u8 SHT1x_Write_Byte(u8 value)字节传送函数
参    数：参数value 要传送的数据
功能描述：向元器件SHT10传送一个字节的数据
返 回 值：返回值error error＝0，传送正确
					  error＝1，传送错误
=========================================*/
uint8_t SHT1x_Write_Byte(uint8_t value)
{
	uint8_t i=0,error=0;
	
	SDA_SET_OUT();
	for(i=0x80;i>0;i/=2) 
	{
		if(i&value) SDA_Pin_OUT(1); //循环相与，结果即为要发送的位
		else SDA_Pin_OUT(0);
		
		delay_us();
		SCL_Pin_OUT(1);
		delay_us();
		SCL_Pin_OUT(0);
		delay_us();
	}

	SDA_SET_IN();//SDA端口模式为浮空输入
	SCL_Pin_OUT(1);
	error = SDA_Pin_IN();
	SCL_Pin_OUT(0);

	return error;  //error＝1，通讯有误;error＝0，通讯成功
}

/*=========================================
函 数 名：SHT1x_Read_Byte(u8 dat)读数据函数
参    数：参数dat=0,接受完一个字节的回复
              dat=1,停止通讯
功能描述：接受元器件SHT10发送到主机的一个字节的数据，
返 回 值：返回值val	即为接受到的数据
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

//启动传输信号
/*时序图：
        ____    ____
SCL: __|    |__|    |__
     ____         _____
SDA:     |_______|   
*/
void SHT1x_Start(void)
{
	SDA_SET_OUT(); //设置SDA端口为输出模式
    
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

//通讯复位时序
/*时序图：
        __    __    __    __    __    __    __    __    __    __
SCL: __|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__
     ______________________________________________________       _____ 
SDA:                                                       |______|
*/
void SHT1x_Reset(void)
{
	uint8_t i;
  SDA_SET_OUT(); //设置SDA端口为输出模式
	SDA_Pin_OUT(1);	    
  SCL_Pin_OUT(0);

	for(i = 0;i < 9;i ++)
	{
		SCL_Pin_OUT(0);
    delay_us();
		SCL_Pin_OUT(1);
    delay_us();
	}
  SHT1x_Start(); //启动传输
}
/*=========================================
函 数 名：char s_softreset(void)软件复位函数
参    数：无
功能描述：resets the sensor by a softreset 
返 回 值：error
=========================================*/
char SHT1X_softreset(void)
{ 
	unsigned char error = 0;  
	SHT1x_Reset();              //reset communication
	error += SHT1x_Write_Byte(RESET);       //send RESET-command to sensor
	return error;                     //error=1 in case of no response form the sensor
}
/*=========================================
函 数 名：char s_write_statusreg(unsigned char value)写状态函数
参    数：参数value
功能描述： writes the status register with checksum (8-bit)
返 回 值：error
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
函 数 名：char s_write_statusreg(unsigned char value)写状态函数
参    数：参数value
功能描述：reads the status register with checksum (8-bit)
返 回 值：error
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
函 数 名：u8 SHT1x_Measure(u8 *p_value, u8 *p_checksum, u8 Mode)启动函数
参    数：参数mode	确定是测量温度或者是测量湿度
功能描述：发送指令到SHT10执行温度和湿度的测量转换
返 回 值：无
=========================================*/
uint8_t SHT1x_Measure(uint16_t *p_value, uint8_t *p_checksum, uint8_t Mode)
{
	uint8_t error=0,Value_H=0,Value_L=0;
	
	SHT1x_Start();//开始启动转换
	
	switch(Mode)
	{
		case 0: error += SHT1x_Write_Byte(MEASURE_TEMP);break; //读温度的值  
		case 1: error += SHT1x_Write_Byte(MEASURE_HUMI);break; //读湿度的值
		default:break;
	} 
	
	SDA_SET_IN();//定义SDA端口为输入模式
	HAL_Delay(300);//等待测量完成
	
	if(SDA_Pin_IN()) error += 1;//若长时间数据线DQ没拉低，则说明测量有错误
	else
	{
		Value_H = SHT1x_Read_Byte(ACK);		 //数据的高字节
		Value_L = SHT1x_Read_Byte(ACK);	     //数据的低字节    		
		*p_value = (Value_H << 8) | Value_L;
		*p_checksum = SHT1x_Read_Byte(noACK);	     //CRC校验码	
	}
	
	return error;
}

/*=========================================
函 数 名：void SHT1X_Caculation(float *p_temperature,float *p_humidity )数据处理函数
参    数：无参数
功能描述：温度和湿度补偿及输出温度值和相对湿度值
返 回 值：无返回值
=========================================*/
void SHT1X_Caculation(float *p_temperature,float *p_humidity )
{
  const float c1=-4.0f;
  const float c2=+0.0405f;
	const float c3=-0.0000028f;			//以上为12位湿度修正公示取值
	const float t1=+0.01f;
	const float t2=+0.00008f;			//以上为14位温度修正公示取值

	
	float T=*p_temperature;
	float H=*p_humidity;
	float H_nonline_Compensation;
	float Humi_Comp;
	float Temp_Comp;
	
	Temp_Comp=T * 0.01f - 39.6f;					          //温度的补偿
		
	H_nonline_Compensation=c3*H*H  + c2*H + c1;                  //相对湿度非线性补偿
  Humi_Comp=( Temp_Comp - 25 ) * ( t1 + t2*H ) + H_nonline_Compensation; //相对湿度对于温度依赖性补偿
	
	if( Humi_Comp > 100 ) Humi_Comp=100;			  //相对湿度最大值修正
	if( Humi_Comp < 0.1f ) Humi_Comp=0.1f;			  //相对湿度最小值修正
	 
	*p_temperature=Temp_Comp;			//保存温度补偿后的结果
	*p_humidity=Humi_Comp; 						  //保存相对湿度补偿后的结果
}
/*=========================================
函 数 名：void SHT1X_Caculation1(float *p_temperature,float *p_humidity )数据处理函数
参    数：无参数
功能描述：温度和湿度补偿及输出温度值和相对湿度值
返 回 值：无返回值
=========================================*/
void SHT1X_Caculation1(float *p_temperature,float *p_humidity )
{
  const int c1=40000000;
  const int c2=405000;
	const int c3=28;			//以上为12位湿度修正公示取值
	const int t1=1;
	const int t2=8;			//以上为14位温度修正公示取值

	
	float T=*p_temperature;
	float H=*p_humidity;
	float H_nonline_Compensation;
	int Humi_Comp;
	int Temp_Comp;
	
	Temp_Comp=(signed long)(T * t1 - 3960);					          //温度的补偿
		
	H_nonline_Compensation=(c2*H -c3* H*H-c1)/100000;                  //相对湿度非线性补偿
  Humi_Comp=(float)( Temp_Comp - 2500 ) * ( t1 + t2*H )/100000 + H_nonline_Compensation; //相对湿度对于温度依赖性补偿
	
	if( Humi_Comp > 10000 ) Humi_Comp=10000;			  //相对湿度最大值修正
	if( Humi_Comp < 10 ) Humi_Comp=10;			  //相对湿度最小值修正
	 
	*p_temperature=Temp_Comp;			//保存温度补偿后的结果
	*p_humidity=Humi_Comp; 						  //保存相对湿度补偿后的结果
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