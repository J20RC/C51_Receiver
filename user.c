#include <stc15.h>

#define uchar unsigned char
#define uint unsigned short int

//���ڷ�����ϱ�־λ
bit flagTxd=0;
//���ڽ��ܼ�����
uchar cntRxd=0;
//���ڽ�����ϱ�־λ
uchar flagRxd=0;
//���ܻ�����
uchar pdata bufRxd[64];

//��������NRF24L01�������ǿ������ⶨ��ġ����SPI����ν��
//����Ϊ����¼���㣬�ؽ�P3.0-P3.1û������ͨ�����������Ѿ��������������ˡ�ֱ�ӽ����Ŷ���ĳ���������һ���ľͿ�����


//NRF24L01���Ŷ���
sbit CSN=P1^5;
sbit MOSI=P1^7;
sbit IRQ=P5^5;
sbit MISO=P5^4;
sbit SCK=P1^6;
sbit CE=P1^4;
sbit LED=P1^3;//LEDָʾ��
//PWM���ͨ��
sbit CH1=P3^2;
sbit CH2=P3^3;
sbit CH3=P3^4;
sbit CH4=P3^5;
sbit CH5=P3^6;
sbit CH6=P3^7;
sbit CH7=P1^0;
sbit CH8=P1^1;
//uchar TX_power=3;
//*****************NRF24L01����**********************
#define TX_ADR_WIDTH    5    //���͵�ַ��� 5�ֽ�
#define RX_ADR_WIDTH    5    //���յ�ַ��� 5�ֽ�
#define TX_PLOAD_WIDTH  32   // �������ݿ�� 32�ֽ�
#define RX_PLOAD_WIDTH  32   //�������ݵĿ�� 32�ֽ�
uchar const TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};//���ص�ַ
uchar const RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01}; //���յ�ַ
//*****************NRF24L01�Ĵ���ָ��*******************
#define READ_REG        0x00   // ���Ĵ���ָ��
#define WRITE_REG       0x20  // д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61   // ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0   // д��������ָ��
#define FLUSH_TX        0xE1   //��շ��ͻ�����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
//**************SPI(nRF24L01)�Ĵ�����ַ����*****************
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define STATUS          0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���            
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������
#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�
#define chNum 8		//����ͨ����
uint recPWMvalue[16], PWMvalue[16];// ����PWMռ�ձ�
uint i=0,startIndex=0;//nrf�����������
uchar chPacket[32];//ͨ�����ݰ�
uchar t_output;//PWM���ͨ�����
bit flag_20ms;//ָʾ��ǰT0ϵͳ�Ƿ񾭹�20MS
uint nrf_reset=0,flag_1s=0;//ָʾ��ǰNRF�Ƿ���Ҫ������ϵͳ�Ƿ���1S���ź�
bit IsDebug=0;//��ǰ�Ƿ�Ϊ����ģʽ
#define fosc 12000000UL//������ʱ��
#define time 65536-fosc/1000
//1Tģʽ��1ms��ʱֵ��65536-fosc/1000��
//12Tģʽ��1ms��ʱֵ��65536-fosc/1000/12��
/***********************************************
/*��������ʱ1MS
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
/*���ܣ���ʱnMs
/*��������Ҫ��ʱ�ĳ���
/***********************************************/
void delay_ms(uint ms)
{
	do{Delay1ms();}
	while(ms--);
}
/**
*���ڳ�ʼ��9600bps@12MHz
*@param 
*@retval
*/
/***********************************************
/*���������ڳ�ʼ��
/*���ܣ�9600bps
/***********************************************/
void UartInit(void)		//9600bps@12MHz
{
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x01;		//����1ѡ��ʱ��2Ϊ�����ʷ�����
	AUXR |= 0x04;		//��ʱ��2ʱ��ΪFosc,��1T
	T2L = 0xC7;		//�趨��ʱ��ֵ
	T2H = 0xFE;		//�趨��ʱ��ֵ
	AUXR |= 0x10;		//������ʱ��2
}
/***********************************************
/*���ܣ�LED����˸
/*������LED����˸����
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
/*���ܣ����ڷ����ֽ�����
/***********************************************/
void SendData(uchar dat)
{
    ES=0;   //�����ڼ�رմ����ж�     
    SBUF=dat;  
    while(!TI);  
    TI=0;  
    ES=1;   //������ɿ������ж� 
}

/***********************************************
/*���ܣ����ڷ����ַ���
/***********************************************/
void SendString(char *s)
{
    while (*s)                  //����ַ���������־
    {
        SendData(*s++);         //���͵�ǰ�ַ�
    }
}
/**********************************************
/*������uint SPI_RW(uint uchar)
/*���ܣ�NRF24L01��SPIдʱ��
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
/*������uchar SPI_Read(uchar reg)
/*���ܣ�NRF24L01��SPI��ȡһ���ֽ�ʱ��
/***********************************************/
//uchar SPI_Read(uchar reg)
//{
//	uchar reg_val;
//	CSN = 0;             	//CSN��'0',����ָ�����
//	SPI(reg);            	//дһ��regָ��
//	reg_val = SPI(0);    	//��ȡreg��ֵ��reg_val
//	CSN = 1;                //CSN��'1',��ʾ����
//	return(reg_val);        //���ض�ȡ��ֵ
//}
/***********************************************
/*���ܣ�NRF24L01дһ���ֽڵ��Ĵ�������
/***********************************************/
uchar SPI_RW_Reg(uchar reg, uchar value)
{
	uchar status;  
	CSN = 0;                   // CSN��'0',�������
	status = SPI(reg);      //��ָ�����STATUS
	SPI(value);             //д����ֵ��reg
	CSN = 1;                   // CSN��'1',��ֹ����
	return(status);            // return nRF24L01 status uchar
}
/***********************************************
/*��������ʼ��NRF����
/***********************************************/
void NRF_init(void)
{
	delay_ms(1);
	CE=0;	//��Ƶֹͣ����
	CSN=1;	//ֹͣ�Ĵ�����д
	SCK=0;	//ʱ���ź�ֹͣ��д
	IRQ=1;	//�жϸ�λ
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);      //  Ƶ��0�Զ� ACKӦ���ֹ  
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);      //��ֹ�Զ�����
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  //  ������յ�ַֻ��Ƶ��0��   
	SPI_RW_Reg(WRITE_REG + RF_CH, 1);        //   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //���ý������ݳ��ȣ���������Ϊ32�ֽ�
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);     //���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB  
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
//	NRF_Read_Buf(WR_TX_PLOAD,TxBuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
//	CE=1;
//	sta=NRF_Read_Reg(STATUS);
//	NRF_Write_Reg(WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
//	if(sta&MAX_TX)//�ﵽ����ط�����
//	{
//		NRF_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
//		return MAX_TX; 
//	}
//	if(sta&TX_OK)//�������
//	{
//		return TX_OK;
//	}
//	return 0xff;//����ԭ����ʧ��
//}
/*=======================================================
* ��  ����u8 NRF24L01_RxPacket(u8 *rxbuf)
* ��  �ܣ�NRF24L01����һ������
* ��  ����*rxbuf���ȴ��������ݵ��׵�ַ
* ����ֵ��0:���ճɹ�;1:��������ʧ��
=======================================================*/
uchar NRF_RxPacket(uchar *rxbuf)
{
	uchar sta;	
	sta=NRF_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF_Write_Reg(WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}	
/*=======================================================
* ��  ����void NRF24L01_RX_Mode(void)
* ��  �ܣ�NRF24L01����ģʽ����
* ��  ������
* ����ֵ����
* ��  ע������RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
		  ��CE��ߺ�,������RXģʽ,�����Խ���������
=======================================================*/ 
void NRF_RX_Mode(void)
{
	CE=0;	  
  	NRF_Write_Buf(WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	NRF_Write_Reg(WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF_Write_Reg(WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF_Write_Reg(WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		  
  	NRF_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF_Write_Reg(WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  	NRF_Write_Reg(WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	CE = 1; //CEΪ��,�������ģʽ 
}	
/*=======================================================
* ��  ����void NRF24L01_TX_Mode(void)
* ��  �ܣ�NRF24L01����ģʽ����
* ��  ������
* ����ֵ����
* ��  ע������TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,
		  ���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
		  PWR_UP,CRCʹ�ܣ�CEΪ�ߴ���10us,���������͡�
=======================================================*/ 
 
//void NRF_TX_Mode(void)
//{														 
//	CE=0;	    
//  	NRF_Write_Buf(WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
//  	NRF_Write_Buf(WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

//  	NRF_Write_Reg(WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
//  	NRF_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
//  	NRF_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
//  	NRF_Write_Reg(WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
//  	NRF_Write_Reg(WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
//  	NRF_Write_Reg(WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
//	CE=1;//CEΪ��,10us����������
//}
/***********************************************
/*���������NRF�����Ƿ���
/*���أ�1Ϊ����0��Ϊ����
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
	
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}
//��ʼ��PWM�����ʧ�ر���
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

//	AUXR |= 0x80;		//��ʱ��ʱ��1Tģʽ
//	TMOD &= 0xF0;
//	TMOD |= 0x01;		//���ö�ʱ��ģʽ
//	TL0 =0;		//���ö�ʱ��ֵ
//	TH0 = 0;		//���ö�ʱ��ֵ
//	TF0 = 0;		//���TF0��־
	//LED_FLASH(5);
	AUXR &= 0x7F;		//��ʱ��ʱ��12Tģʽ
//	AUXR |= 0x00;		//T0,T1������12T
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TMOD |= 0x01;		//���ö�ʱ��ģʽ
	TL0 = 0x00;			//���ö�ʱ��ֵ
	TH0 = 0x00;			//���ö�ʱ��ֵ
	TF0 = 0;			//���TF0��־
	TR0 = 1;			//��ʱ��0��ʼ��ʱ
	ET0 = 1;			//����ʱ��0�ж�	
	ET0=1;
	EA=1;
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
	
	
	
	
}
/***********************************************
/*��������ȡ�´��ж�ʱ��
/*���ܣ����ƶ����PWM������ֵ����Χ��500~2500��
/*���أ�12M����12��Ƶ�����Լ�����ÿ����һ��������1΢�룬��ȫ���������Ƶľ���Ҫ��
			��Ϊ��ʱ����TH0��TL0��Ҫȫ��������0xFF���ڼ�1�����ͻ�����жϣ�����Ҫ�����
			pwm������жϣ���ôTH0��TL0��Ӧ�ø�ֵ��0xFFFF-pwm��	�����ֵ��ʼ����������ʱ�ж�
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
	//��ʱ50ms��ϵͳ��ѹ�ȶ�
	delay_ms(200);	
	UartInit();		
	LED=1;//�ϵ翪��ָʾ�ƣ���ʾϵͳ��������	
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
	//��ΪP3.0,P3.1��ͨ�����1��2ռ�á����Ե����е���һ���󴮿ڽ����ڴ�ӡ��Ϣ
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
			for(i=0;i<chNum;i++)//����ʼλ�ÿ�ʼ����
			{
				recPWMvalue[i] = ((uint)chPacket[startIndex] << 8) | ((uint)(chPacket[startIndex+1]));// �ϲ�u8Ϊu16
				if(recPWMvalue[i]!=0) PWMvalue[i] = recPWMvalue[i];
				startIndex = startIndex+2;
			}
			for (i=chNum; i<16; i++) 
			{
				PWMvalue[i] = 1500;//δ�õ���ͨ��ȫ������
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
			//�����������źţ���ʼ����NRFģ��
			nrf_reset=0;				
			NRF_init();	
			NRF_RX_Mode();
		}	
	}
}
/***********************************************
/*���ܣ�8·���PWM�жϷ�����
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
/*���ܣ������жϷ�����
/***********************************************/
void Uart_Isr()interrupt 4
{
	//�յ��ֽ�
	if(RI)
	{
		//��������жϱ�־
		RI=0;
		flagRxd=1;
		//���ܻ�����δ����ʱ
		if(cntRxd<sizeof(bufRxd))
		{
			//��������ַ������ԼӼ�����
			bufRxd[cntRxd++]=SBUF;
		}
	}
	//�ֽڷ������
	if(TI)
	{
		TI=0;
		flagTxd=1;
	}
}