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

/*      �����򾭹�������ȫ����, ���ṩ�绰����֧��, �粻�����, �����в�����ػ���.  */


/*************	��������˵��	**************

	���Է�����

	����������ʱ�ӺͲ������޸�"�û��������"�����ض��壬��������ص�MCU��

	ͨ������������MCU�������ݣ�MCU�յ���ԭ�����ء�
*/
										   

/*************** �û�������� *****************************/

#define MAIN_Fosc		22118400L	//define main clock

#define Baudrate1		9600		//define the baudrate, ���ʹ��BRT�������ʷ�����,�����ʸ�����2һ��
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define Baudrate2		19200		//define the baudrate2,
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define		BUF_LENTH	128		//���崮�ڽ��ջ��峤��

/**********************************************************/

//#include	<reg51.h>

sfr AUXR1 = 0xA2;
sfr	AUXR = 0x8E;
sfr S2CON = 0x9A;	//12C5A60S2˫����ϵ��
sfr S2BUF = 0x9B;	//12C5A60S2˫����ϵ��
sfr IE2   = 0xAF;	//STC12C5A60S2ϵ��
sfr BRT   = 0x9C;

unsigned char 	uart1_wr;		//дָ��
unsigned char 	uart1_rd;		//��ָ��
unsigned char 	xdata RX1_Buffer[BUF_LENTH];
bit		B_TI;

unsigned char 	uart2_wr;		//дָ��
unsigned char 	uart2_rd;		//��ָ��
unsigned char 	xdata RX2_Buffer[BUF_LENTH];
bit		B_TI2;


/****************** �������Զ����ɣ��û������޸� ************************************/

#define T1_TimerReload	(256 - MAIN_Fosc / 192 / Baudrate1)			//Calculate the timer1 reload value	at 12T mode
#define BRT_Reload		(256 - MAIN_Fosc / 12 / 16 / Baudrate2)		//Calculate BRT reload value

#define	TimeOut1		(28800 / (unsigned long)Baudrate1 + 2)
#define	TimeOut2		(28800 / (unsigned long)Baudrate2 + 2)

#define	TI2				(S2CON & 0x02) != 0
#define	RI2				(S2CON & 0x01) != 0
#define	CLR_TI2()		S2CON &= ~0x02
#define	CLR_RI2()		S2CON &= ~0x01

/**********************************************************/

/******************** ���غ������� ***************/
/********************************************************************************
* STC12C5A60S2����PN532����Ѱ������
* STC12C5A60S2ͨ��UART��PN532����
* ͨ���ж�ʵ�ֽ���UART���ݣ���ͨ����ʱ�ж�PN532����Ӧ�Ƿ�ʱ
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

//-----------------------------UART1����ʹ�õ�ȫ�ֱ���---------------------------
xdata unsigned char gPn532Data[100];            //13.56M RF���ݻ���
xdata unsigned char *gpPn532RxBuf, gPn532RxLen; //�����жϺ����������buf
xdata unsigned char gi;     //���ڽ������������ack֮���������
//-------------------------------------------------------------------------------
unsigned char gTimeMult;
unsigned char gTimeOutAckFlag;
unsigned char gTimeOutDataFlag;
unsigned char gUart1RecvFlag;

sbit p25 = P2^5;
sbit LED1 = P3^5;
sbit LED2 = P3^7;
sbit BUZZER = P3^6;

unsigned char icId[4] = {0xB6,0x9B,0x26,0xEE}; 	  	//�׿�

void C51InitIo(void)
{     
 P3M1 &= ~(1<<5),	P3M0 |=  (1<<5);	//P1.5 set as push-pull output mode	 
 P3M1 &= ~(1<<6),	P3M0 |=  (1<<6);	//P1.6 set as push-pull output mode
 P3M1 &= ~(1<<7),	P3M0 |=  (1<<7);	//P1.7 set as push-pull output mode
 LED1 = 0;
 LED2 = 0;
 BUZZER = 0;
 //P2M0 |= 0x20; 
 //P2M1 |= 0x00;//����P2.5Ϊǿ�������
}
//��ʱʱ��=50ms
void C51InitTimer1(void)
{
	AUXR &= 0xBF;		//��ʱ��ʱ��12Tģʽ
	TMOD &= 0x0F;		//���ö�ʱ��ģʽ
	TMOD |= 0x10;		//���ö�ʱ��ģʽ
	TL1 = 0x00;		//���ö�ʱ��ֵ
	TH1 = 0x4C;		//���ö�ʱ��ֵ
	TF1 = 0;		//���TF1��־
	TR1 = 0;		//��ʱ��1��ʼ��ʱ
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
 //AUXR |= 0xC0;//bit7->T0x12:0->12T,1->1T;bit6->T1x12:0->12T,1->1T,Ĭ��������Ǵ�ͳ8051�ٶ�
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
  ʹ�ö��������ʷ�������Ϊ�����ʷ����� 
**************************************************/
void C51InitUart(void)
{
// AUXR |= 0x18; //BRTR=1,�������������ʷ�����,S2SMOD=1,����2������*2��S1BRS=1������1ʹ�ö��������ʷ�������Ϊ�����ʷ�����,S1BRS=0������1ʹ�ö�ʱ��1��Ϊ�����ʷ�����
//     
// SCON |= 0x50; //UART1������ģʽ1(8λuart,������ʼλ��ֹͣλ��10λ)���ɱ�����f=((2^SMOD)/32)*BRT��ren==1->�����н���
// PCON |= 0x80; //smod=1, 
// TMOD |= 0x20; //��ʱ�������뷽ʽ2����TR1���ƶ�ʱ���Ŀ�����ر�
//// TH1   = 0xFF; //22.1184MHz:0xF4->9600, 0xFA->19200, 0xFD->38400, 0xFE->57600, 0xFF->115200
//
//// TH1   = 0xFE; //11.0592MHz:0xF4->9600, 0xFA->19200, 0xFD->38400, 0xFE->57600, 0xFF->115200 
//// TL1   = 0xFF;
// TR1   = 1;  //������ʱ��1   
 ES    = 1;     //������1�ж�
// S2CON |= 0x50; //UART2������ģʽ1(8λuart,������ʼλ��ֹͣλ��10λ)���ɱ�����f=((2^S2SMOD)/32)*BRT��ren==1->�����н���  
// BRT    = 0xF4; //BRTx12=0;0xFF->115200,0xF4->9600,0xA0->1200 
 IE2   |= 0x01; //������2�ж�
	PCON |= 0x80;		//ʹ�ܲ����ʱ���λSMOD
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x40;		//��ʱ��1ʱ��ΪFosc,��1T
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//�����ʱ��1ģʽλ
	TMOD |= 0x20;		//�趨��ʱ��1Ϊ8λ�Զ���װ��ʽ
	TL1 = 0xFA;		//�趨��ʱ��ֵ
	TH1 = 0xFA;		//�趨��ʱ����װֵ
	ET1 = 0;		//��ֹ��ʱ��1�ж�
	TR1 = 1;		//������ʱ��1


	AUXR |= 0x08;		//ʹ�ܲ����ʱ���λS2SMOD
	S2CON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x04;		//���������ʷ�����ʱ��ΪFosc,��1T
	BRT = 0xFA;		//�趨���������ʷ�������װֵ
	AUXR |= 0x10;		//�������������ʷ�����
}
//����1����һ���ֽں���
void C51Uart1SendByte(unsigned char num)
{
 ES = 0;  
 TI = 0;
 SBUF = num;
 while (0==TI);
 TI = 0;
 ES = 1;
}
//---------����2���������һ���ֽں���----------
void C51Uart2SendByte(unsigned char num)
{ 
 IE2   &= 0xFE; //������2�жϣ���Ҫ���ӹرա����ж���䣬��Ȼ����2���������� 
 S2CON = S2CON&0xFD;//��������жϱ�־
 S2BUF = num;
 while (0==(S2CON&0x02));//����1˵���������
 S2CON = S2CON&0xFD;//��������жϱ�־
 IE2   |= 0x01; //������2�ж�
}
//��ʱn*1us����
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
* ��������  PN532SendData
* �������ܣ���������PN532��������
* ���������
 pBuf����Ҫ���͸�PN532��֡���� 
 len�� pBuf�ĳ���
* ���������
        �����Ƿ�ɹ���־
            �ɹ���SUCCESS
            ʧ�ܣ���Ӧ�Ĵ������
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
* ��������  PN532RecvData
* �������ܣ���������PN532��������
* ���������
        �� 
* ���������
        pBuf��
            ����֡���� 
        �����Ƿ�ɹ���־
            �ɹ���SUCCESS
            ʧ�ܣ���Ӧ�Ĵ������
***************************************************/
unsigned char PN532RecvData(unsigned char *pBuf)
{  
 gTimeOutDataFlag = 0;
 gUart1RecvFlag = 0;
// C51InitTimer(4);
 gpPn532RxBuf = pBuf;        //Start of RX buffer
    gPn532RxLen = 6;            // Load RX byte counter
 gi = 0;
 while (!(gTimeOutDataFlag||gUart1RecvFlag));//ֻҪ��һ��Ϊ1���˳�ѭ��
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
* ��������  PN532SendAck
* �������ܣ���������PN532����ACK֡
* ���������
 ��
* ���������
        �����Ƿ�ɹ���־
            �ɹ���SUCCESS
            ʧ�ܣ���Ӧ�Ĵ������
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
* ��������  PN532RecvAck
* �������ܣ���������PN532����ACK֡
* ���������
 �� 
* ���������
        pBuf��
            ����ACK֡����
        �����Ƿ�ɹ���־
            �ɹ���SUCCESS
            ʧ�ܣ���Ӧ�Ĵ������
*************************************************/
unsigned char PN532RecvAck(unsigned char *pBuf)
{  
 gTimeOutAckFlag = 0;
 gUart1RecvFlag = 0;
// C51InitTimer(1);
 gpPn532RxBuf = pBuf;        //Start of RX buffer
    gPn532RxLen = 6;            // Load RX byte counter
 gi = 0;
 while (!(gTimeOutAckFlag||gUart1RecvFlag));//ֻҪ��һ��Ϊ1���˳�ѭ��
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
* ��������  SAMConfiguration
* �������ܣ�����PN532���ڲ�����
* ���������
 mode:
      0x01:normal mode
      0x02:virtual mode
      0x03:wired mode
      0x04:dual mode
 timeout:
      0x00:��ʹ��timeout
      0x01-0xFF����ʱֵ
 irq:
      0x00����ʹ��P70_IRQ
      0x01��ʹ��P70_IRQ
* ���������
 �ɹ���ERR_NO
        ʧ�ܣ�ERR_RECVACK��ERR_RECVDATA
* �������������
        pRfBuf�����룬���ڴ����������Ļ��棬���ⲿ���룬���ڽ���ں����ڲ�����������������
                �������Ŵ�PN532���յĽ��
**********************************************************************************************************/
unsigned char SAMConfiguration(unsigned char *pRfBuf, unsigned char mode, unsigned char timeout, unsigned char irq)
{
    unsigned char temp;
    unsigned char s;
    pRfBuf[0] = 0x00; //ǰ����
    pRfBuf[1] = 0x00; //����ʼ��־2���ֽ�
    pRfBuf[2] = 0xFF;
    pRfBuf[3] = 3 + 1 + 1;//len + 1�ֽڵ�PD0 + 1�ֽڵ�TFI
    pRfBuf[4] = 0x100-pRfBuf[3];
    pRfBuf[5] = 0xD4; //�����ʾ
    temp = pRfBuf[5];
    pRfBuf[6] = 0x14; //������
    temp += pRfBuf[6];
    pRfBuf[7] = mode; 
    temp += pRfBuf[7];
    pRfBuf[8] = timeout;
    temp += pRfBuf[8];
    pRfBuf[9] = irq;
    temp += pRfBuf[9];
    
    pRfBuf[10] = 0x100 - temp; //DCS
    pRfBuf[11] = 0x00;
    
    //�ײ㷢�ͺ���
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
* ��������  InListPassiveTarget
* �������ܣ�PN532��⿨
* ���������
 maxTag:
            PN532����ʼ�����ĸ�����PN532һ��ֻ��ͬʱ����2�ſ�         
 brty:
            0x00 : 106 kbps type A (ISO/IEC14443 Type A),
            0x01 : 212 kbps (FeliCa polling),
            0x02 : 424 kbps (FeliCa polling),
            0x03 : 106 kbps type B (ISO/IEC14443-3B),
            0x04 : 106 kbps Innovision Jewel tag.
        pData:
* ���������
 �ɹ���ERR_NO
        ʧ�ܣ�ERR_RECVACK��ERR_RECVDATA
* �������������
        pRfBuf�����룬���ڴ����������Ļ��棬���ⲿ���룬���ڽ���ں����ڲ�����������������
                �������Ŵ�PN532���յĽ��
*****************************************************************************************/
unsigned char InListPassiveTarget(unsigned char *pRfBuf, unsigned char maxTag, unsigned char brty, 
                                  unsigned char *pData, unsigned char len)
{
    unsigned char i,j,temp;
    unsigned char s;
	
//    unsigned char icId[4] = {0x66,0x76,0x4B,0xE8};  		

    pRfBuf[0] = 0x00; //ǰ����
    pRfBuf[1] = 0x00; //����ʼ��־2���ֽ�
    pRfBuf[2] = 0xFF;
    pRfBuf[3] = len + 1 + 1 + 1 + 1;//len + 1�ֽڵ�maxTag + 1�ֽڵ�brty + 1�ֽڵ�PD0 + 1�ֽڵ�TFI
    pRfBuf[4] = 0x100-pRfBuf[3];
    pRfBuf[5] = 0xD4; //�����ʾ
    temp = pRfBuf[5];
    pRfBuf[6] = 0x4A; //������
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
    
    //�ײ㷢�ͺ���    
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
// WDT_CONTR = 0x3F;//ʹ�ܿ��Ź����忴�Ź���־������ģʽ��������Ƶϵ��(scale:bit2-bit0)�����ʱ��=��12*scale*32768��/Fosc
    
 EA = 1;   //cpu�ж�������
 
// p25 = 0;
// C51Delay1us(60000);
// C51Delay1us(60000);
// C51Delay1us(60000);
// C51Delay1us(60000);
// C51Delay1us(60000);
// p25 = 1;
 gUart1RecvFlag = 0;
//--------------����PN532----------------
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
					    //�����״̬��Ϣ	  
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
					    //����ȷ��״̬��Ϣ					  
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
//�����жϷ������
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
    if (0x00==*gpPn532RxBuf)//����ACK֡����ͨ֡��һ������������Ҫ����Դ�
    {
     gPn532RxLen = *gpPn532RxBuf + 2;  //����ACK
    }
    else
    {
     gPn532RxLen = *gpPn532RxBuf + 3;  //������ + 1�ֽڵİ�����У�� + 1�ֽڵ�����У�� + 1�ֽڵĺ���
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
		if(++ms_25Cnt==1)  //����25ms�¼�
		{
			ms_25Cnt = 0;
			gTimeOutAckFlag = 1;													
		}	
	}	
	if(gTimeOutDataFlag==0)
	{				 
		if(++ms_100Cnt==4)	//����100ms�¼�
		{
			ms_100Cnt = 0;
			gTimeOutDataFlag = 1;
		}
	} 
// if (--gTimeMult)
//  return;
// gTimeOutFlag = 1;
//
			 //��ȷ�Ŀ�
		//	C51Uart2SendByte(0x33);
			//����1s�¼�
			if(f_BUZZER_ERR_NO)
			{
				if(++noErrBuzzerCnt==10)//
				{
					noErrBuzzerCnt = 0;
					f_BUZZER_ERR_NO = 0;
					BUZZER = 0;								
				}
			}						
			//����5s�¼�
			if(f_LED1_ERR_NO)
			{		   				
				if(++noErrLedCnt==50)		//25*200ms
				{	  	
					noErrLedCnt = 0;
					f_LED1_ERR_NO = 0;
					LED1 = 0;
				} 
			} 

			//����Ŀ�
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
