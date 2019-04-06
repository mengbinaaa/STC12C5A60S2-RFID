//Git is a distributed version control system
//Git is a free software.
/*------------------------------------------------------------------*/
/* --- STC MCU International Limited -------------------------------*/
/* --- STC 1T Series MCU Programme Demo ----------------------------*/
/* --- Fax: 86-755-82944243 ----------------------------------------*/
/* --- Tel: 86-755-82948412 ----------------------------------------*/
/* --- Web: www.STCMCU.com -----------------------------------------*/
/* If you want to use the program or the program referenced in the  */
/* article, please specify in which data and procedures from STC    */
/*------------------------------------------------------------------*/

/*      本程序经过测试完全正常, 不提供电话技术支持, 如不能理解, 请自行补充相关基础.  */


/*************	本程序功能说明	**************

	测试方法：

	根据您的主时钟和波特率修改"用户定义参数"里的相关定义，编译后下载到MCU。

	通过串口助手向MCU发送数据，MCU收到后原样返回。
*/
										   

/*************** 用户定义参数 *****************************/

#define MAIN_Fosc		22118400L	//define main clock

#define Baudrate1		9600		//define the baudrate, 如果使用BRT做波特率发生器,则波特率跟串口2一样
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define Baudrate2		19200		//define the baudrate2,
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define		BUF_LENTH	128		//定义串口接收缓冲长度

/**********************************************************/

//#include	<reg51.h>

sfr AUXR1 = 0xA2;
sfr	AUXR = 0x8E;
sfr S2CON = 0x9A;	//12C5A60S2双串口系列
sfr S2BUF = 0x9B;	//12C5A60S2双串口系列
sfr IE2   = 0xAF;	//STC12C5A60S2系列
sfr BRT   = 0x9C;

unsigned char 	uart1_wr;		//写指针
unsigned char 	uart1_rd;		//读指针
unsigned char 	xdata RX1_Buffer[BUF_LENTH];
bit		B_TI;

unsigned char 	uart2_wr;		//写指针
unsigned char 	uart2_rd;		//读指针
unsigned char 	xdata RX2_Buffer[BUF_LENTH];
bit		B_TI2;


/****************** 编译器自动生成，用户请勿修改 ************************************/

#define T1_TimerReload	(256 - MAIN_Fosc / 192 / Baudrate1)			//Calculate the timer1 reload value	at 12T mode
#define BRT_Reload		(256 - MAIN_Fosc / 12 / 16 / Baudrate2)		//Calculate BRT reload value

#define	TimeOut1		(28800 / (unsigned long)Baudrate1 + 2)
#define	TimeOut2		(28800 / (unsigned long)Baudrate2 + 2)

#define	TI2				(S2CON & 0x02) != 0
#define	RI2				(S2CON & 0x01) != 0
#define	CLR_TI2()		S2CON &= ~0x02
#define	CLR_RI2()		S2CON &= ~0x01

/**********************************************************/

/******************** 本地函数声明 ***************/
/********************************************************************************
* STC12C5A60S2控制PN532进行寻卡操作
* STC12C5A60S2通过UART与PN532相连
* 通过中断实现接收UART数据，并通过定时判断PN532的响应是否超时
********************************************************************************/
#include "reg52.h"
#include <intrins.h>
#include <string.h>
#define ERR_NO        0
#define ERR_TIMEOUT   1
#define ERR_SENDDATA  2
#define ERR_RECVACK   3
#define ERR_RECVDATA  4
#define ERR_CARD	  5

sfr P1M1 = 0x91;	//P1M1.n,P1M0.n 	=00--->Standard,	01--->push-pull
sfr P1M0 = 0x92;	//					=10--->pure input,	11--->open drain
sfr P0M1 = 0x93;	//P0M1.n,P0M0.n 	=00--->Standard,	01--->push-pull
sfr P0M0 = 0x94;	//					=10--->pure input,	11--->open drain
sfr P2M1 = 0x95;	//P2M1.n,P2M0.n 	=00--->Standard,	01--->push-pull
sfr P2M0 = 0x96;	//					=10--->pure input,	11--->open drain
sfr P3M1  = 0xB1;	//P3M1.n,P3M0.n 	=00--->Standard,	01--->push-pull
sfr P3M0  = 0xB2;	//					=10--->pure input,	11--->open drain

//-----------------------------UART1部分使用的全局变量---------------------------
xdata unsigned char gPn532Data[100];            //13.56M RF数据缓存
xdata unsigned char *gpPn532RxBuf, gPn532RxLen; //用于中断函数处理接收buf
xdata unsigned char gi;     //用于接收数据与接收ack之间进行区别
//-------------------------------------------------------------------------------
unsigned char gTimeMult;
unsigned char gTimeOutAckFlag;
unsigned char gTimeOutDataFlag;
unsigned char gUart1RecvFlag;

sbit p25 = P2^5;
sbit LED1 = P3^5;
sbit LED2 = P3^7;
sbit BUZZER = P3^6;

unsigned char icId[4] = {0xB6,0x9B,0x26,0xEE}; 	  	//白卡

void C51InitIo(void)
{     
 P3M1 &= ~(1<<5),	P3M0 |=  (1<<5);	//P1.5 set as push-pull output mode	 
 P3M1 &= ~(1<<6),	P3M0 |=  (1<<6);	//P1.6 set as push-pull output mode
 P3M1 &= ~(1<<7),	P3M0 |=  (1<<7);	//P1.7 set as push-pull output mode
 LED1 = 0;
 LED2 = 0;
 BUZZER = 0;
 //P2M0 |= 0x20; 
 //P2M1 |= 0x00;//配置P2.5为强上拉输出
}
//定时时间=50ms
void C51InitTimer1(void)
{
	AUXR &= 0xBF;		//定时器时钟12T模式
	TMOD &= 0x0F;		//设置定时器模式
	TMOD |= 0x10;		//设置定时器模式
	TL1 = 0x00;		//设置定时初值
	TH1 = 0x4C;		//设置定时初值
	TF1 = 0;		//清除TF1标志
	TR1 = 0;		//定时器1开始计时
}
   
void C51StartTimer0()
{
 TR0 = 1;
}

void C51StartTimer1()
{
 TR1 = 1;
}
void C51StopTimer1()
{
 TR1 = 0;
}
void C51InitTimer(unsigned char num)
{
 //AUXR |= 0xC0;//bit7->T0x12:0->12T,1->1T;bit6->T1x12:0->12T,1->1T,默认情况下是传统8051速度
 TMOD |= 0x01;
 TH0 = 0x4C;
 TL0 = 0x00;//0x4C00=19456,(65536-19456)*12/22118400=0.025s=25ms
 gTimeMult = num;//25*40=1000ms=1s
 ET0 = 1;
 TR0 = 1;
}
void C51StopTimer()
{
// TR0 = 0;
}
/**************************************************
Function: init_uart();
Description:
  set uart working mode,
  使用独立波特率发生器作为波特率发生器 
**************************************************/
void C51InitUart(void)
{
// AUXR |= 0x18; //BRTR=1,启动独立波特率发生器,S2SMOD=1,串口2波特率*2，S1BRS=1，串口1使用独立波特率发生器作为波特率发生器,S1BRS=0，串口1使用定时器1作为波特率发生器
//     
// SCON |= 0x50; //UART1工作于模式1(8位uart,加上起始位与停止位共10位)，可变速率f=((2^SMOD)/32)*BRT，ren==1->允许串行接收
// PCON |= 0x80; //smod=1, 
// TMOD |= 0x20; //定时器工作与方式2，由TR1控制定时器的开启与关闭
//// TH1   = 0xFF; //22.1184MHz:0xF4->9600, 0xFA->19200, 0xFD->38400, 0xFE->57600, 0xFF->115200
//
//// TH1   = 0xFE; //11.0592MHz:0xF4->9600, 0xFA->19200, 0xFD->38400, 0xFE->57600, 0xFF->115200 
//// TL1   = 0xFF;
// TR1   = 1;  //启动定时器1   
 ES    = 1;     //允许串口1中断
// S2CON |= 0x50; //UART2工作于模式1(8位uart,加上起始位与停止位共10位)，可变速率f=((2^S2SMOD)/32)*BRT，ren==1->允许串行接收  
// BRT    = 0xF4; //BRTx12=0;0xFF->115200,0xF4->9600,0xA0->1200 
 IE2   |= 0x01; //允许串口2中断
	PCON |= 0x80;		//使能波特率倍速位SMOD
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器1时钟为Fosc,即1T
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//清除定时器1模式位
	TMOD |= 0x20;		//设定定时器1为8位自动重装方式
	TL1 = 0xFA;		//设定定时初值
	TH1 = 0xFA;		//设定定时器重装值
	ET1 = 0;		//禁止定时器1中断
	TR1 = 1;		//启动定时器1


	AUXR |= 0x08;		//使能波特率倍速位S2SMOD
	S2CON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x04;		//独立波特率发生器时钟为Fosc,即1T
	BRT = 0xFA;		//设定独立波特率发生器重装值
	AUXR |= 0x10;		//启动独立波特率发生器
}
//串口1发送一个字节函数
void C51Uart1SendByte(unsigned char num)
{
 ES = 0;  
 TI = 0;
 SBUF = num;
 while (0==TI);
 TI = 0;
 ES = 1;
}
//---------串口2发送与接收一个字节函数----------
void C51Uart2SendByte(unsigned char num)
{ 
 IE2   &= 0xFE; //允许串口2中断，需要增加关闭、打开中断语句，不然串口2工作不正常 
 S2CON = S2CON&0xFD;//清除发送中断标志
 S2BUF = num;
 while (0==(S2CON&0x02));//等于1说明发送完成
 S2CON = S2CON&0xFD;//清除发送中断标志
 IE2   |= 0x01; //允许串口2中断
}
//延时n*1us函数
void C51Delay1us(unsigned long int n)
{
 while (--n)
 {
  _nop_();
  _nop_();
  _nop_();
  _nop_();
  _nop_();
 }
}
/*****************************************************************
* 函数名：  PN532SendData
* 函数功能：主控器向PN532发送数据
* 输入参数：
 pBuf：需要发送给PN532的帧数据 
 len： pBuf的长度
* 输出参数：
        返回是否成功标志
            成功：SUCCESS
            失败：相应的错误代码
********************************************************************/
unsigned char PN532SendData(unsigned char *pBuf, unsigned char len)
{ 
 unsigned char i;
// WDT_CONTR = 0x3F;
 for (i=0; i<len; i++)
 {
  C51Uart1SendByte(pBuf[i]);
 }      
    return ERR_NO;
}
/***************************************************
* 函数名：  PN532RecvData
* 函数功能：主控器从PN532接收数据
* 输入参数：
        无 
* 输出参数：
        pBuf：
            接收帧缓存 
        返回是否成功标志
            成功：SUCCESS
            失败：相应的错误代码
***************************************************/
unsigned char PN532RecvData(unsigned char *pBuf)
{  
 gTimeOutDataFlag = 0;
 gUart1RecvFlag = 0;
// C51InitTimer(4);
 gpPn532RxBuf = pBuf;        //Start of RX buffer
    gPn532RxLen = 6;            // Load RX byte counter
 gi = 0;
 while (!(gTimeOutDataFlag||gUart1RecvFlag));//只要有一个为1则退出循环
// C51StopTimer();
 if (1==gUart1RecvFlag)
 {
     return ERR_NO;
 }
 else
 {
  return ERR_TIMEOUT;
 }
}	
/****************************************
* 函数名：  PN532SendAck
* 函数功能：主控器向PN532发送ACK帧
* 输入参数：
 无
* 输出参数：
        返回是否成功标志
            成功：SUCCESS
            失败：相应的错误代码
*****************************************/
unsigned char PN532SendAck()
{  
 unsigned char pAckBuf[6] = {0x00,0x00,0xFF,0x00,0xFF,0x00};
 unsigned char i;
 
 for (i=0; i<6; i++)
 {
  C51Uart1SendByte(pAckBuf[i]);
 }   
    return ERR_NO;
}
/***********************************************
* 函数名：  PN532RecvAck
* 函数功能：主控器从PN532接收ACK帧
* 输入参数：
 无 
* 输出参数：
        pBuf：
            接收ACK帧缓存
        返回是否成功标志
            成功：SUCCESS
            失败：相应的错误代码
*************************************************/
unsigned char PN532RecvAck(unsigned char *pBuf)
{  
 gTimeOutAckFlag = 0;
 gUart1RecvFlag = 0;
// C51InitTimer(1);
 gpPn532RxBuf = pBuf;        //Start of RX buffer
    gPn532RxLen = 6;            // Load RX byte counter
 gi = 0;
 while (!(gTimeOutAckFlag||gUart1RecvFlag));//只要有一个为1则退出循环
// C51StopTimer();
 if (1==gUart1RecvFlag)
 {
     return ERR_NO;
 }
 else
 {
  return ERR_TIMEOUT;
 }
}
/*********************************************************************************************************
* 函数名：  SAMConfiguration
* 函数功能：设置PN532的内部参数
* 输入参数：
 mode:
      0x01:normal mode
      0x02:virtual mode
      0x03:wired mode
      0x04:dual mode
 timeout:
      0x00:不使用timeout
      0x01-0xFF：超时值
 irq:
      0x00：不使用P70_IRQ
      0x01：使用P70_IRQ
* 输出参数：
 成功：ERR_NO
        失败：ERR_RECVACK，ERR_RECVDATA
* 输入输出参数：
        pRfBuf：输入，用于存放输入参数的缓存，从外部传入，用于解决在函数内部定义过大的数组问题
                输出，存放从PN532接收的结果
**********************************************************************************************************/
unsigned char SAMConfiguration(unsigned char *pRfBuf, unsigned char mode, unsigned char timeout, unsigned char irq)
{
    unsigned char temp;
    unsigned char s;
    pRfBuf[0] = 0x00; //前导码
    pRfBuf[1] = 0x00; //包开始标志2个字节
    pRfBuf[2] = 0xFF;
    pRfBuf[3] = 3 + 1 + 1;//len + 1字节的PD0 + 1字节的TFI
    pRfBuf[4] = 0x100-pRfBuf[3];
    pRfBuf[5] = 0xD4; //方向标示
    temp = pRfBuf[5];
    pRfBuf[6] = 0x14; //命令码
    temp += pRfBuf[6];
    pRfBuf[7] = mode; 
    temp += pRfBuf[7];
    pRfBuf[8] = timeout;
    temp += pRfBuf[8];
    pRfBuf[9] = irq;
    temp += pRfBuf[9];
    
    pRfBuf[10] = 0x100 - temp; //DCS
    pRfBuf[11] = 0x00;
    
    //底层发送函数
    s = PN532SendData(pRfBuf,12);
    if (s!=ERR_NO)
    {
        return ERR_SENDDATA;
    }
    s = PN532RecvAck(pRfBuf);
    if (s!=ERR_NO)
    {
        return ERR_RECVACK;
    }
    s = PN532RecvData(pRfBuf);
    if (s!=ERR_NO)
    {
        return ERR_RECVDATA;
    }
    
    return ERR_NO;
}
/****************************************************************************************
* 函数名：  InListPassiveTarget
* 函数功能：PN532检测卡
* 输入参数：
 maxTag:
            PN532最大初始化卡的个数，PN532一次只能同时处理2张卡         
 brty:
            0x00 : 106 kbps type A (ISO/IEC14443 Type A),
            0x01 : 212 kbps (FeliCa polling),
            0x02 : 424 kbps (FeliCa polling),
            0x03 : 106 kbps type B (ISO/IEC14443-3B),
            0x04 : 106 kbps Innovision Jewel tag.
        pData:
* 输出参数：
 成功：ERR_NO
        失败：ERR_RECVACK，ERR_RECVDATA
* 输入输出参数：
        pRfBuf：输入，用于存放输入参数的缓存，从外部传入，用于解决在函数内部定义过大的数组问题
                输出，存放从PN532接收的结果
*****************************************************************************************/
unsigned char InListPassiveTarget(unsigned char *pRfBuf, unsigned char maxTag, unsigned char brty, 
                                  unsigned char *pData, unsigned char len)
{
    unsigned char i,j,temp;
    unsigned char s;
	
//    unsigned char icId[4] = {0x66,0x76,0x4B,0xE8};  		

    pRfBuf[0] = 0x00; //前导码
    pRfBuf[1] = 0x00; //包开始标志2个字节
    pRfBuf[2] = 0xFF;
    pRfBuf[3] = len + 1 + 1 + 1 + 1;//len + 1字节的maxTag + 1字节的brty + 1字节的PD0 + 1字节的TFI
    pRfBuf[4] = 0x100-pRfBuf[3];
    pRfBuf[5] = 0xD4; //方向标示
    temp = pRfBuf[5];
    pRfBuf[6] = 0x4A; //命令码
    temp += pRfBuf[6];
    pRfBuf[7] = maxTag;   //1
    temp += pRfBuf[7];
    pRfBuf[8] = brty;   //0
    temp += pRfBuf[8];
    
    for (i=0; i<len; i++)
    {
        pRfBuf[9+i] = pData[i];
        temp  += pData[i];
    }
    
    pRfBuf[9+i] = 0x100 - temp; //DCS
    pRfBuf[9+i+1] = 0x00; 
    
    //底层发送函数    
    s = PN532SendData(pRfBuf,9+i+1+1);
    if (s!=ERR_NO)
    {
        return ERR_SENDDATA;
    }
    s = PN532RecvAck(pRfBuf);
    if (s!=ERR_NO)
    {				 
	//	C51Uart2SendByte(ERR_RECVACK);
        return ERR_RECVACK;
    }
    s = PN532RecvData(pRfBuf);
    if (s!=ERR_NO)
    {		 
	//	C51Uart2SendByte(ERR_RECVDATA);
        return ERR_RECVDATA;
    }

    if(memcmp(icId,&pRfBuf[13],4)==0)
	{
	//	C51Uart2SendByte(ERR_NO);
		return ERR_NO;
	}
	else
	{
	//	C51Uart2SendByte(ERR_CARD);
		return ERR_CARD;
	}
		
 //   return ERR_NO;
}

 unsigned char f_25ms = 0;
 unsigned char ms_25Cnt = 0;
 unsigned char ms_100Cnt = 0;

 unsigned char f_LED1_ERR_NO = 0;
 unsigned char f_BUZZER_ERR_NO = 0;
 unsigned char f_LED2_ERR_CARD = 0;
 unsigned char f_BUZZER_ERR_CARD = 0;
 	    		
 unsigned char preStatus;	
 
unsigned char noErrBuzzerCnt;
unsigned char errCardBuzzerCnt;		
unsigned char noErrLedCnt;   
unsigned char errCardLedCnt;	
void main()
{ 
 unsigned char cnt,buzzerCnt;
 unsigned char i,s; 
 unsigned char OK[3] = {0xAA,0xAA,0xAA};
 unsigned char ERR[3] = {0xCC,0xCC,0xCC};
 
 
 C51InitIo(); 
 C51InitUart(); 
 C51InitTimer(1);
// WDT_CONTR = 0x3F;//使能看门狗、清看门狗标志、空闲模式计数、分频系数(scale:bit2-bit0)，溢出时间=（12*scale*32768）/Fosc
    
 EA = 1;   //cpu中断总允许
 
// p25 = 0;
// C51Delay1us(60000);
// C51Delay1us(60000);
// C51Delay1us(60000);
// C51Delay1us(60000);
// C51Delay1us(60000);
// p25 = 1;
 gUart1RecvFlag = 0;
//--------------唤醒PN532----------------
 C51Uart1SendByte(0x55);
 C51Uart1SendByte(0x55);
 C51Uart1SendByte(0x00);
 C51Uart1SendByte(0x00);
 C51Uart1SendByte(0x00);		   
 SAMConfiguration(gPn532Data,1,0,1);
 for (i=0; i<gPn532Data[3]+7; i++)
 {
  C51Uart2SendByte(gPn532Data[i]);
 }		     
 LED1 = 0;
 LED2 = 0;
 preStatus = 0;
//---------------------------------------
 while(1)
 { 							  
			if(f_25ms)
			{
					f_25ms = 0; 
								
					s = InListPassiveTarget(gPn532Data,1,0,0,0);	   
//							C51Uart2SendByte(1);
						C51Uart2SendByte(s);
//							C51Uart2SendByte(preStatus);
//							C51Uart2SendByte(2);	
									  	
					  if((ERR_NO==s&&preStatus==ERR_RECVDATA)||(ERR_NO==s&&preStatus==ERR_CARD))
					  {	
					    //清错误卡状态信息	  
						LED2 = 0;	
						f_LED2_ERR_CARD = 0;
						f_BUZZER_ERR_CARD = 0;   

						LED1 = 1;		   
						BUZZER = 1;		
						f_LED1_ERR_NO = 1;
						f_BUZZER_ERR_NO = 1;
						noErrBuzzerCnt = 0;
						errCardBuzzerCnt = 0;		
						noErrLedCnt = 0;   
						errCardLedCnt = 0;		
					  }
					  else if((ERR_CARD==s&&preStatus==ERR_RECVDATA)||(ERR_CARD==s&&preStatus==ERR_NO))
					  {		
					    //清正确卡状态信息					  
						LED1 = 0;
						f_LED1_ERR_NO = 0;
						f_BUZZER_ERR_NO = 0;
							  
						LED2 = 1;
						BUZZER = 1;	   	
						f_LED2_ERR_CARD = 1;
						f_BUZZER_ERR_CARD = 1;		
						noErrBuzzerCnt = 0;
						errCardBuzzerCnt = 0;		
						noErrLedCnt = 0;   
						errCardLedCnt = 0;	     
				      }	   
						
					   preStatus = s; 
			}		 
   }
}
//串口中断服务程序
void Uart1IsrHandler(void) interrupt 4 
{ 
 if (RI)
 {
  RI = 0;  
  gPn532RxLen--; 
  if (gPn532RxLen)
  {
   *gpPn532RxBuf = SBUF;
   
   gi++;
   if (gi==4)
   {
    if (0x00==*gpPn532RxBuf)//由于ACK帧与普通帧有一定的区别，所以要区别对待
    {
     gPn532RxLen = *gpPn532RxBuf + 2;  //接收ACK
    }
    else
    {
     gPn532RxLen = *gpPn532RxBuf + 3;  //包长度 + 1字节的包长度校验 + 1字节的数据校验 + 1字节的后导码
    }
   }
   
   gpPn532RxBuf++;
  }
  else
  {
   *gpPn532RxBuf = SBUF;
   gUart1RecvFlag = 1;
  }  
 }
 if (TI)
 {          
  TI = 0; 
 }
} 
void Timer0IsrHandler(void) interrupt 1
{							 
	 static unsigned char counter;
	
	 TH0 = 0x4C;
	 TL0 = 0x00;

	 if(++counter==20)
	 {
	 	counter = 0;   
		 f_25ms = 1;
	 }
	
	if(gTimeOutAckFlag==0)
	{
		if(++ms_25Cnt==1)  //处理25ms事件
		{
			ms_25Cnt = 0;
			gTimeOutAckFlag = 1;													
		}	
	}	
	if(gTimeOutDataFlag==0)
	{				 
		if(++ms_100Cnt==4)	//处理100ms事件
		{
			ms_100Cnt = 0;
			gTimeOutDataFlag = 1;
		}
	} 
// if (--gTimeMult)
//  return;
// gTimeOutFlag = 1;
//
			 //正确的卡
		//	C51Uart2SendByte(0x33);
			//处理1s事件
			if(f_BUZZER_ERR_NO)
			{
				if(++noErrBuzzerCnt==10)//
				{
					noErrBuzzerCnt = 0;
					f_BUZZER_ERR_NO = 0;
					BUZZER = 0;								
				}
			}						
			//处理5s事件
			if(f_LED1_ERR_NO)
			{		   				
				if(++noErrLedCnt==50)		//25*200ms
				{	  	
					noErrLedCnt = 0;
					f_LED1_ERR_NO = 0;
					LED1 = 0;
				} 
			} 

			//错误的卡
			if(f_BUZZER_ERR_CARD)
			{
			    errCardBuzzerCnt++;
				if(errCardBuzzerCnt==10)
				{			 
					BUZZER = 0;
				}
				else if(errCardBuzzerCnt==20)
				{
					BUZZER = 1;
				}	
				else if(errCardBuzzerCnt==30)
				{
					BUZZER = 0;
				}
				else if(errCardBuzzerCnt==40)
				{
					BUZZER = 1;
				}
				else if(errCardBuzzerCnt==50)
				{
					f_BUZZER_ERR_CARD = 0;
					BUZZER = 0;
				}
			}									
			if(f_LED2_ERR_CARD)
			{	  			
			   if(++errCardLedCnt==50)
				{			   
					errCardLedCnt = 0;
					f_LED2_ERR_CARD = 0;
					LED2 = 0;
				}	
		    }
}
