#include <stc15.h>

#define uchar unsigned char
#define uint unsigned short int

//串口发送完毕标志位
bit flagTxd=0;
//串口接受计数器
uchar cntRxd=0;
//串口接受完毕标志位
uchar flagRxd=0;
//接受缓冲区
uchar pdata bufRxd[64];

//经过测试NRF24L01的引脚是可以随意定义的。软件SPI无所谓。
//作者为了烧录方便，特将P3.0-P3.1没有用作通道输出。如果已经有萝莉三代控了。直接将引脚定义改成萝莉三代一样的就可以了


//NRF24L01引脚定义
sbit CSN=P1^5;
sbit MOSI=P1^7;
sbit IRQ=P5^5;
sbit MISO=P5^4;
sbit SCK=P1^6;
sbit CE=P1^4;
sbit LED=P1^3;//LED指示灯
//PWM输出通道
sbit CH1=P3^2;
sbit CH2=P3^3;
sbit CH3=P3^4;
sbit CH4=P3^5;
sbit CH5=P3^6;
sbit CH6=P3^7;
sbit CH7=P1^0;
sbit CH8=P1^1;
//uchar TX_power=3;
//*****************NRF24L01常量**********************
#define TX_ADR_WIDTH    5    //发送地址宽度 5字节
#define RX_ADR_WIDTH    5    //接收地址宽度 5字节
#define TX_PLOAD_WIDTH  32   // 发送数据宽度 32字节
#define RX_PLOAD_WIDTH  32   //接收数据的宽度 32字节
uchar const TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};//本地地址
uchar const RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01}; //接收地址
//*****************NRF24L01寄存器指令*******************
#define READ_REG        0x00   // 读寄存器指令
#define WRITE_REG       0x20  // 写寄存器指令
#define RD_RX_PLOAD     0x61   // 读取接收数据指令
#define WR_TX_PLOAD     0xA0   // 写待发数据指令
#define FLUSH_TX        0xE1   //清空发送缓冲区
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
//**************SPI(nRF24L01)寄存器地址常量*****************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测            
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断
#define chNum 8		//采样通道数
uint recPWMvalue[16], PWMvalue[16];// 控制PWM占空比
uint i=0,startIndex=0;//nrf解包辅助变量
uchar chPacket[32];//通道数据包
uchar t_output;//PWM输出通道标记
bit flag_20ms;//指示当前T0系统是否经过20MS
uint nrf_reset=0,flag_1s=0;//指示当前NRF是否需要重置与系统是否经历1S无信号
bit IsDebug=0;//当前是否为调试模式
#define fosc 12000000UL//定义主时钟
#define time 65536-fosc/1000
//1T模式的1ms定时值（65536-fosc/1000）
//12T模式的1ms定时值（65536-fosc/1000/12）
/***********************************************
/*函数：延时1MS
/***********************************************/
void Delay1ms()		//@12.000MHz
{
	unsigned char i, j;

	i = 12;
	j = 169;
	do
	{
		while (--j);
	} while (--i);
}
/***********************************************
/*功能：延时nMs
/*参数：需要延时的长度
/***********************************************/
void delay_ms(uint ms)
{
	do{Delay1ms();}
	while(ms--);
}
/**
*串口初始化9600bps@12MHz
*@param 
*@retval
*/
/***********************************************
/*函数：串口初始化
/*功能：9600bps
/***********************************************/
void UartInit(void)		//9600bps@12MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0xC7;		//设定定时初值
	T2H = 0xFE;		//设定定时初值
	AUXR |= 0x10;		//启动定时器2
}
/***********************************************
/*功能：LED灯闪烁
/*参数：LED灯闪烁次数
/***********************************************/
void LED_FLASH(uint num)
{
	while(num)
	{
		LED=!LED;
		delay_ms(100);
		num--;
	}
	LED=0;
}
/***********************************************
/*功能：串口发送字节数据
/***********************************************/
void SendData(uchar dat)
{
    ES=0;   //发送期间关闭串口中断     
    SBUF=dat;  
    while(!TI);  
    TI=0;  
    ES=1;   //发送完成开串口中断 
}

/***********************************************
/*功能：串口发送字符串
/***********************************************/
void SendString(char *s)
{
    while (*s)                  //检测字符串结束标志
    {
        SendData(*s++);         //发送当前字符
    }
}
/**********************************************
/*函数：uint SPI_RW(uint uchar)
/*功能：NRF24L01的SPI写时序
/**********************************************/
uchar SPI(uchar byte)
{
	uchar i;
	for(i=0;i<8;i++)
	{
		MOSI=(byte&0x80);
		SCK=1;
		byte<<=1;
		byte|=MISO;
		SCK=0;
	}
	return byte;
}

/***********************************************
/*函数：uchar SPI_Read(uchar reg)
/*功能：NRF24L01的SPI读取一个字节时序
/***********************************************/
//uchar SPI_Read(uchar reg)
//{
//	uchar reg_val;
//	CSN = 0;             	//CSN置'0',允许指令操作
//	SPI(reg);            	//写一条reg指令
//	reg_val = SPI(0);    	//读取reg的值到reg_val
//	CSN = 1;                //CSN置'1',禁示操作
//	return(reg_val);        //返回读取的值
//}
/***********************************************
/*功能：NRF24L01写一个字节到寄存器函数
/***********************************************/
uchar SPI_RW_Reg(uchar reg, uchar value)
{
	uchar status;  
	CSN = 0;                   // CSN置'0',允许操作
	status = SPI(reg);      //这指令，并读STATUS
	SPI(value);             //写数据值到reg
	CSN = 1;                   // CSN置'1',禁止操作
	return(status);            // return nRF24L01 status uchar
}
/***********************************************
/*函数：初始化NRF天线
/***********************************************/
void NRF_init(void)
{
	delay_ms(1);
	CE=0;	//射频停止工作
	CSN=1;	//停止寄存器读写
	SCK=0;	//时种信号停止读写
	IRQ=1;	//中断复位
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);      //  频道0自动 ACK应答禁止  
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);      //禁止自动发送
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，   
	SPI_RW_Reg(WRITE_REG + RF_CH, 1);        //   设置信道工作为2.4GHZ，收发必须一致
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);     //设置发射速率为2MHZ，发射功率为最大值0dB  
}

uchar NRF_Write_Reg(uchar reg,uchar val)
{
	uchar status;
	CSN=0;
	status=SPI(reg);
	SPI(val);
	CSN=1;
	return (status);
}
uchar NRF_Write_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	CSN=0;
	status=SPI(reg);
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	{
		SPI(*pBuf++);
	}
	CSN=1;
	return status;
}
uchar NRF_Read_Reg(uchar reg)
{
	uchar reg_val;
	CSN=0;
	SPI(reg);
	reg_val=SPI(0XFF);
	CSN=1;
	return (reg_val);
}
uchar NRF_Read_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	CSN=0;
	status=SPI(reg);
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	{
		pBuf[u8_ctr]=SPI(0xFF);
	}
	CSN=1;
	return status;
}
//uchar NRF_TxPacket(uchar *TxBuf)
//{
//	uchar sta;
//	CE=0;
//	NRF_Read_Buf(WR_TX_PLOAD,TxBuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
//	CE=1;
//	sta=NRF_Read_Reg(STATUS);
//	NRF_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
//	if(sta&MAX_TX)//达到最大重发次数
//	{
//		NRF_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
//		return MAX_TX; 
//	}
//	if(sta&TX_OK)//发送完成
//	{
//		return TX_OK;
//	}
//	return 0xff;//其他原因发送失败
//}
/*=======================================================
* 函  数：u8 NRF24L01_RxPacket(u8 *rxbuf)
* 功  能：NRF24L01接收一次数据
* 参  数：*rxbuf：等待接收数据的首地址
* 返回值：0:接收成功;1:接收数据失败
=======================================================*/
uchar NRF_RxPacket(uchar *rxbuf)
{
	uchar sta;	
	sta=NRF_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}	
/*=======================================================
* 函  数：void NRF24L01_RX_Mode(void)
* 功  能：NRF24L01接收模式配置
* 参  数：无
* 返回值：无
* 备  注：设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
		  当CE变高后,即进入RX模式,并可以接收数据了
=======================================================*/ 
void NRF_RX_Mode(void)
{
	CE=0;	  
  	NRF_Write_Buf(WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	NRF_Write_Reg(WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  	NRF_Write_Reg(WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF_Write_Reg(WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  	NRF_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF_Write_Reg(WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF_Write_Reg(WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	CE = 1; //CE为高,进入接收模式 
}	
/*=======================================================
* 函  数：void NRF24L01_TX_Mode(void)
* 功  能：NRF24L01发射模式配置
* 参  数：无
* 返回值：无
* 备  注：设置TX地址,写TX数据宽度,设置RX自动应答的地址,
		  填充TX发送数据,选择RF频道,波特率和LNA HCURR
		  PWR_UP,CRC使能，CE为高大于10us,则启动发送。
=======================================================*/ 
 
//void NRF_TX_Mode(void)
//{														 
//	CE=0;	    
//  	NRF_Write_Buf(WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
//  	NRF_Write_Buf(WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

//  	NRF_Write_Reg(WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
//  	NRF_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
//  	NRF_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
//  	NRF_Write_Reg(WRITE_REG+RF_CH,40);       //设置RF通道为40
//  	NRF_Write_Reg(WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
//  	NRF_Write_Reg(WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
//	CE=1;//CE为高,10us后启动发送
//}
/***********************************************
/*函数：检测NRF天线是否工作
/*返回：1为错误，0：为正常
/***********************************************/
uchar NRF_check(void)
{
	uchar buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	uchar i;
	NRF_Write_Buf(WRITE_REG+TX_ADDR,buf,5);
	for(i=0;i<5;i++)
	{
		buf[i]=0;
	}
	NRF_Read_Buf(TX_ADDR,buf,5);
	for(i=0;i<5;i++)
	{
		if(buf[i]!=0XA5)break;
	}
	
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}
//初始化PWM输出和失控保护
void PWM_reset(void)
{
	for(i=0;i<chNum;i++)
	{
		PWMvalue[i] =1500;
	}
}

void pwm_init()
{
	CH1=0;
	CH2=0;
	CH3=0;
	CH4=0;
	CH5=0;
	CH6=0;
	CH7=0;
	CH8=0;
	P1M0=0;
	P1M1=0;
	P3M0 = 0;
	P3M1 = 0;
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器模式
	TMOD |= 0x01;		//设置定时器模式
	TL0 = 0x00;			//设置定时初值
	TH0 = 0x00;			//设置定时初值
	TF0 = 0;			//清除TF0标志
	TR0 = 1;			//定时器0开始计时
	ET0 = 1;			//开定时器0中断	
	ET0=1;
	EA=1;
	TR0 = 1;		//定时器0开始计时
	
	
	
	
}
/***********************************************
/*函数：获取下次中断时间
/*功能：控制舵机的PWM脉冲宽度值（范围：500~2500）
/*返回：12M晶振，12分频，所以计数器每递增一个数就是1微秒，完全满足舵机控制的精度要求
			因为定时器是TH0，TL0都要全部计数到0xFF后在计1个数就会产生中断，所以要想产生
			pwm毫秒的中断，那么TH0，TL0就应该赋值（0xFFFF-pwm）	从这个值开始计数产生定时中断
/***********************************************/
void getPwmTimerValue(uint pwm)
{
	uint val;
	val=0xffff-pwm;
	TR0=0;
	TL0=val;
	TH0=val>>8;
	TR0=1;
}
bit IsAccept=0;
void main(void)
{
	//延时50ms待系统电压稳定
	delay_ms(200);	
	UartInit();		
	LED=1;//上电开启指示灯，表示系统正常启动	
	SendString("PWM_reset...\r\n");
	//LED_FLASH(5);
	PWM_reset();
	while(NRF_check())
	{
		SendString("NRF ERROR...\r\n");
		LED_FLASH(5);
		delay_ms(200);
	}
	SendString("NRF24L01 Init...\r\n");
	NRF_init();	
	delay_ms(50);
	SendString("Set Rx Mode...\r\n");
	NRF_RX_Mode();
	delay_ms(50);
	SendString("PwmInit...\r\n");
	pwm_init();
	while(1)
	{
		IsAccept=1;
		if(NRF_RxPacket(chPacket)==0)
		{
			IsAccept=0;
			for(i=0;i<32-chNum*2;i++)
			{
				if(chPacket[i]==0x00 & chPacket[i+1]==0x00) 
				{
					startIndex = i+2;
					break;
				}
			}
			for(i=0;i<chNum;i++)//从起始位置开始处理
			{
				recPWMvalue[i] = ((uint)chPacket[startIndex] << 8) | ((uint)(chPacket[startIndex+1]));// 合并u8为u16
				if(recPWMvalue[i]!=0) PWMvalue[i] = recPWMvalue[i];
				startIndex = startIndex+2;
			}
			for (i=chNum; i<16; i++) 
			{
				PWMvalue[i] = 1500;//未用到的通道全部置中
			}
				
		}
		
		if(flag_20ms)
		{
			flag_20ms=0;
			
			if(IsAccept)
			{
				flag_1s++;
			}
			else flag_1s=0;
		}
		if(flag_1s>=200)
		{
			flag_1s=0;
			LED_FLASH(5);
			nrf_reset++;
			PWM_reset();
		}
		if(nrf_reset>=3)
		{
			//连续三秒无信号，开始重置NRF模块
			nrf_reset=0;				
			NRF_init();	
			NRF_RX_Mode();
		}	
	}
}
/***********************************************
/*功能：8路软件PWM中断服务函数
/***********************************************/
void ET0_isr()interrupt 1
{
	switch(t_output)
	{
		case 1:
			CH1=1;
			flag_20ms=1;
			getPwmTimerValue(PWMvalue[0]);			
			break;
		case 2:
			CH1=0;
			getPwmTimerValue(2500-PWMvalue[0]);
			break;
		case 3:
			CH2=1;
			getPwmTimerValue(PWMvalue[1]);
			break;
		case 4:
			CH2=0;
			getPwmTimerValue(2500-PWMvalue[1]);
			break;
		case 5:
			CH3=1;
			getPwmTimerValue(PWMvalue[2]);
			break;
		case 6:
			CH3=0;
			getPwmTimerValue(2500-PWMvalue[2]);
			break;
		case 7:
			CH4=1;
			getPwmTimerValue(PWMvalue[3]);
			break;
		case 8:
			CH4=0;
			getPwmTimerValue(2500-PWMvalue[3]);
			break;
		case 9:
			CH5=1;
			getPwmTimerValue(PWMvalue[4]);
			break;
		case 10:
			CH5=0;
			getPwmTimerValue(2500-PWMvalue[4]);
			break;
		case 11:
			CH6=1;
			getPwmTimerValue(PWMvalue[5]);
			break;
		case 12:
			CH6=0;
			getPwmTimerValue(2500-PWMvalue[5]);
			break;
		case 13:
			CH7=1;
			getPwmTimerValue(PWMvalue[6]);
			break;
		case 14:
			CH7=0;
			getPwmTimerValue(2500-PWMvalue[6]);
			break;
		case 15:
			CH8=1;
			getPwmTimerValue(PWMvalue[7]);
			break;
		case 16:
			CH8=0;
			getPwmTimerValue(2500-PWMvalue[7]);
			t_output=0;
			break;
	}
	t_output++;
}

/***********************************************
/*功能：串口中断服务函数
/***********************************************/
void Uart_Isr()interrupt 4
{
	//收到字节
	if(RI)
	{
		//清除接受中断标志
		RI=0;
		flagRxd=1;
		//接受缓冲区未用完时
		if(cntRxd<sizeof(bufRxd))
		{
			//保存接受字符，并自加计数器
			bufRxd[cntRxd++]=SBUF;
		}
	}
	//字节发送完毕
	if(TI)
	{
		TI=0;
		flagTxd=1;
	}
}
