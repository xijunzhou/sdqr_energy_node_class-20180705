/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "stm32l1xx_hal_rtc.h"
#include "sx1276-Hal.h"
#include "sx1276.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
#include "stm32l1xx_lp_modes.h"
#include "dlt645.h"
#include "cj188.h"
#include "main.h"
/*
一、  Neruogrid-Class-LORA 协议：
1、     网络ID   网络ID   设备ID   设备ID   RX窗口   前导码    心跳周期  采集周期 数据帧开始码 数据帧开始码 数据帧长度  数据
symb:	NwkID[0] NwkID[1] DevID[0] DevID[1] DevRxGap  PrebSymb  Heartbeat  Collect  Sync1(0xa5)  Sync2(0x5a)  DATA_LEN    DATA
      	 |        |         |         |         |      |          |          |           |         |           |         |    
      	 |        |         |         |         |      |          |          |           |         |           | 	       | 
byte:    0        1         2         3         4    5 - 6      7 - 8      9 - 10   	  11         12         13       14 - N

2       Neruogrid-Class-LORA支持如下两种模式：
		Ø  Neruogrid-ClassA-LORA   超低功耗模式，模块只在心跳时发送、接收信号，其余时间处于休眠模式
		Ø  Neruogrid-ClassC-LORA   低功耗模式，模块周期性监听无线信号，收到信号自动唤醒  
		Ø 水表、气表、热量表LORA模块工作在 Neruogrid-ClassA-LORA 模式
		Ø 电表LORA模块工作在 Neruogrid-ClassC-LORA 模式

3、LORA模块通讯参数：
        中心频率：  434Mhz
        扩频因子：  1024
        带    宽：  250Khz
        纠 错 码：  4/5
        CRC校验：   有
        报   头：   显式
     
二、功耗设计
1、水表、气表、热量表LORA模块需要与表具安装在一起或外挂方式，采用 Neruogrid-ClassA-LORA 超低功耗协议，休眠周期默认为24小时。
    Neruogrid-ClassA-LORA工作模式如下：
                __                               __   ____	         	             
      _________|  |__|__|_______________________|  |_|    |________________

              呼吸|gap1|gap2     睡眠           呼吸 gap1收到网关命令   
      
2、电表LORA模块安装在表箱或电表内部，模块支持220V供电，采用 Neruogrid-ClassC-LORA 低功耗呼吸监听协议。

    Neruogrid-ClassC-LORA工作模式如下：
		            __ 	                  __ 	         __  	             
      _________|  |__________________|  |_________|  |________    

       睡眠   接收网关命令  睡眠    接收网关命令

      
三、水、电、气、热表具类型以及通讯地址处理

1、LORA模块与水表、气表、热量表一对一连接，通过广播地址进行通讯。

2、LORA模块与电表一对多连接，LORA模块按网关命令所指地址与电表进行通讯。

3、LORA模块通过硬件码序选择通讯协议，编码如下：
	         PIN0 PIN1      协议
     水表   0    0       CJ188
     气表   0    1       CJ188
     热表   1    0       CJ188
     电表   1    1       DLT645

四、 Neruogrid-Class-LORA 网关工作模式

1、 Neruogrid-ClassA-LORA模式：
    网关无发送任务时，进入RXcontinue模式，监听水表、气表、热量表LORA模块的心跳，当收到心跳后，接收LORA模块计量数据，当有主动采集任务时，按照Neruogrid-ClassA-LORA规范，下发采集命令。
    网关工作模式如下：
				                __                 __  __
      _________|_______|  |_______|_______|  ||  |____

              监听   接收到数据         接收到数据\下发采集命令

2、Neruogrid-ClassC-LORA模式： 
      网关判断采集策略，向LORO模块发送采集命令，LORA模块在监听模式下收到网关采集命令，按照网关命令中的电表通讯地址采集电表数据，并将数据返回网关。
    网关工作模式如下：
	              __ 	                  __ 	         __  	       
      _________|  |__________________|  |_________|  |________

            发送采集命令          发送采集命令	



五、使用说明

1、节点工作模式：

LoRaClassType = LoRaClassA  ：节点工作在超低功耗模式A，节点只会在设定的心跳周期内唤醒，采集数据发送给网关后，进行休眠，在休眠期间不能接收任何外部信号

LoRaClassType = LoRaClassC  ：节点工作在低功耗模式C，节点每隔4秒钟，自动苏醒一次，监测外部信号，如果有匹配的信号，则接收数据。

2、节点串口波特率：

节点对外的串口，默认波特率为 2400 偶校验

3、节点类型选择：METER_TYPE

节点类型：水电气热。可以通过软件设置,模块根据对应的节点类型，自动判断抄表协议



LoRaDevice.type = ELECTOR_METER    电表类型、DLT645-2007协议 

LoRaDevice.type = WATER_METER      水表类型、CL188协议

LoRaDevice.type = HEART_METER      热量表类型、CL188协议

LoRaDevice.type = GAS_METER        燃气表类型、CL188协议



六、STM32L151CB 硬件资源

		SPI1  ---  SX1278
		UART1 ---  METER     2400 E 8 1  

*/

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ST_LINK_DEBUG_ENABLE 1
#define SECOND    1
#define MINITE   (60*SECOND)
#define HOURE    (60*MINITE)

#if(JOHN_DEBUG)
#define SYSTEM_SLEEP_TIME       (20*SECOND) //调试时：30秒
#else
#define SYSTEM_SLEEP_TIME       (20*HOURE)  //正常运行默认：24小时  单位秒：65535秒 = 21小时
#endif

#define DEVICE_RESPONSE_TIMEOUT 1000     //设备反馈超时时间

//  协议类型
#define LoRaClassA 1
#define LoRaClassC 3
#define LoRaClassType  LoRaClassA                

//  表计类型
#define  ELECTOR_METER        0x40           //电表
#define  GAS_METER            0x30           //燃气表
#define  WATER_METER          0x10           //水表
#define  HEART_METER          0x20           //热量表
#define METER_TYPE ELECTOR_METER

/***************************************************************************************************************/
typedef struct
{
	uint8_t Sync1;            //配置帧开始标识    0xaa
	uint8_t Sync2;            //配置帧开始标识    0x55
	uint8_t type;             //配置帧类型        0x00:网络地址, 0x01:设备地址, 0x10:心跳周期, 0x11:采集周期 
	uint8_t data0;            //配置帧数据位0
	uint8_t data1;            //配置帧数据位1
	uint8_t cs;               //配置帧校验位      Sync1 + Sync2 + type + data0 + data1 + cs
}s_Config_data;

typedef struct
{
	uint8_t NwkID[2];         //无线网络地址
	uint8_t DevID[2];         //无线节点地址
	uint8_t DevRxGap;         //接收串口间隔时间  1 byte	   单位：秒
	uint16_t PrebSymb;        //前导码            2 bytes    
	uint16_t Heartbeat;       //心跳周期          2 bytes    单位：分钟
	uint16_t Collect;         //采集周期          2 bytes    单位：分钟 ，默认采集周期等于心跳周期
	uint8_t Sync1;            //数据帧开始标识    0xA5
	uint8_t Sync2;            //数据帧开始标识    0x5A
	uint8_t Dlen;             //用户数据长度
	uint8_t Buff[PAYLOAD_DATABUFF_SIZE];
}sPayLoadType;

enum LORA_frame
{
	NwkID_OFFSET = 0,
	DevID_OFFSET = 2,
	RXGAP_OFFSET = 4,
	PREAMB_OFFSET = 5,
	HEART_OFFSET = 7,
	COLLECT_OFFSET = 9,
	SYNC1_OFFSET = 11,
	SYNC2_OFFSET,
	DLEN_OFFSET,
	DATA_OFFSET,
};

uint8_t PayLoadBuff[PAYLOAD_MAX_SIZE];                                           //LORA数据buff

uint16_t PayLoadLen;

sPayLoadType *pRFPayLoad = (sPayLoadType *)PayLoadBuff;      //LORA数据buff指针

/***************************************************************************************************************/

enum t_ClassState
{
	sIDLE = 0,
	TXWindow1,
	TXWindow2,
	RXWindow1Gap,
	RXWindow1,
	RXWindow2Gap,
	RXWindow2
}LoRaProtocolState = sIDLE;

                              
#define TXWindowTimeOut     10000                       // 发送超时时间
#define DEV_ID_LEN  10                                 // 设备通讯地址：ID长度
typedef struct                                         //表具数据帧
{
	uint8_t type;               //设备类型
  uint8_t id[DEV_ID_LEN];
  uint8_t NwkID[2];         //无线网络地址
	uint8_t DevID[2];         //无线节点地址
	uint8_t DevRxGap;         //接收串口间隔时间  1 byte	   单位：秒
	uint16_t PrebSymb;        //前导码            2 bytes    
	uint16_t Heartbeat;       //心跳周期          2 bytes    单位：分钟
	uint16_t Collect;         //采集周期          2 bytes    单位：分钟 ，默认采集周期等于心跳周期
}t_LoRaDevice;



t_LoRaDevice LoRaDevice;                              //表具数据帧

tRFProcessReturnCodes RfResult;

USART_RECEIVETYPE UsartType;

typedef enum 
{
	SX1278_RX = 0,
	SX1278_TX,
	SX1278_IDLE,
	SX1278_SLEEP,
}t_LoraFlag;
t_LoraFlag Lora_flag;          /*  LORA state:	SX1278_RX 	SX1278_TX 	SX1278_IDLE 	SX1278_SLEEP,   */


uint32_t SystemTick;

const uint8_t NetAddr[2] = {'L','O'};

const uint8_t NodeAddr[2] = {'R','A'};

uint8_t	RecvGatewayCmd = 0;                 //节点心跳后收到网关数据

uint32_t RecvGatewayCmdTimeOut;             //收到网关数据并转发给设备，等待设备反馈时间

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_SYSTICK_Callback(void)
{
	SystemTick++;
}

void SysTick_Init()
{
	int i; 
  SystemTick = i;             //获取随机变量值
  Debug("\r\nSysTick_Init SystemTick:%d!",SystemTick);
}

void UartPayLoadInit(void)
{
	memset( &UsartType, 0, sizeof(USART_RECEIVETYPE) );
}

void LoRaPayLoadInit(void)
{
	pRFPayLoad->NwkID[0] = 'L';
	pRFPayLoad->NwkID[1] = 'O';
	pRFPayLoad->DevID[0] = 'R';
	pRFPayLoad->DevID[1] = 'A';
	
	pRFPayLoad->DevRxGap = 5;
	pRFPayLoad->PrebSymb = Preamble_Default;	       //默认为 8 

	if(LoRaClassType == LoRaClassC) //电表采用监控呼吸模块：ClassC
		{
			pRFPayLoad->Heartbeat = LORA_SLEEP_TIME/1000;      //默认12小时发送一次心跳，在心跳中发送采集的数据
			pRFPayLoad->Collect = 0;        //默认12小时采集一次数据，默认采集周期等于心跳周期
		}
	else
		{
			pRFPayLoad->Heartbeat = SYSTEM_SLEEP_TIME;      //默认12小时发送一次心跳，在心跳中发送采集的数据
			pRFPayLoad->Collect = SYSTEM_SLEEP_TIME;        //默认12小时采集一次数据，默认采集周期等于心跳周期
		}
	pRFPayLoad->Sync1 = 0xA5;                        //数据帧开始标识    0xA5
	pRFPayLoad->Sync2 = 0x5A;                        //数据帧开始标识    0x5A
	pRFPayLoad->Dlen = 0;                            //初始化用户数据长度 = 0

}

static void SetIoSleep( void )
{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	HAL_SPI_MspDeInit(&hspi2);	
//	HAL_UART_MspDeInit(&huart3);
//	HAL_UART_MspDeInit(&huart1);	
//	
///* set SPI2\USART3\USART1 IO as pull low input	
//    PB10     ------> USART3_TX
//    PB11     ------> USART3_RX 
//    PB6     ------> USART1_TX
//    PB7     ------> USART1_RX 
//    */
//	  GPIO_InitStruct.Pin = SX1278_SCK_Pin|SX1278_MISO_Pin|SX1278_MOSI_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//		
//	  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		
//#undef ST_LINK_DEBUG_ENABLE
#if( ST_LINK_DEBUG_ENABLE )
				HAL_DBGMCU_EnableDBGSleepMode();
				HAL_DBGMCU_EnableDBGStopMode();
				HAL_DBGMCU_EnableDBGStandbyMode();
#else
				HAL_DBGMCU_DisableDBGSleepMode();
				HAL_DBGMCU_DisableDBGSleepMode();
				HAL_DBGMCU_DisableDBGSleepMode();
#endif
		BSP_LED_Off(LED1);
		BSP_LED_Off(LED2);
}

static void LoRaDeviceInit(void)
{
	int i = 0;
	
	
	memset(&LoRaDevice,0,sizeof(t_LoRaDevice));			
	
	LoRaDevice.type = METER_TYPE;
	for(i=0;i<DEV_ID_LEN;i++)        //默认广播地址0xAA
		LoRaDevice.id[i] = 0xAA;
		

	LoRaDevice.NwkID[0] = 'L';
	LoRaDevice.NwkID[1] = 'O';
	LoRaDevice.DevID[0] = 'R';
	LoRaDevice.DevID[1] = 'A';
	
	LoRaDevice.DevRxGap = 5;
	LoRaDevice.PrebSymb = Preamble_Default;
	if(LoRaClassType == LoRaClassC) //电表采用监控呼吸模块：ClassC
		{
			LoRaDevice.Heartbeat = LORA_SLEEP_TIME/1000;      //默认12小时发送一次心跳，在心跳中发送采集的数据
			LoRaDevice.Collect = 0;        //默认12小时采集一次数据，默认采集周期等于心跳周期
		}
	else
		{
			LoRaDevice.Heartbeat = SYSTEM_SLEEP_TIME;      //默认12小时发送一次心跳，在心跳中发送采集的数据
			LoRaDevice.Collect = SYSTEM_SLEEP_TIME;        //默认12小时采集一次数据，默认采集周期等于心跳周期
		}
}

void SystemReInit(void)
{
	MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
	SPI_Enable(&hspi2);
	Debug("=>SleepMode!\r\nPress WAKE_UP button to wake up ...\r\n");
	UartReceive_DMA();
}



void SetLoRaSleep(void)
{
	SX1276LoRaSetRFState(RFLR_STATE_IDLE);               /*RFState ?????*/
	SX1276LoRaSetOpMode(RFLR_OPMODE_SLEEP);                  /*????*/
	BSP_LED_Off(LED1);
	Lora_flag = SX1278_SLEEP;	
	RFDebug("\r\n SetLoRaSleep:%d\r\n",GET_TICK_COUNT( ));
}

void SetLoRaRx(void)
{	
	SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
	BSP_LED_On(LED1);
	Lora_flag = SX1278_RX;
	RFDebug("\r\n SetLoRaRx:%d\r\n",GET_TICK_COUNT( ));
}

void SetLoRaTx(void)
{
	SX1276LoRaSetRFState( RFLR_STATE_TX_INIT );
	BSP_LED_On(LED1);
	Lora_flag = SX1278_TX;
  Debug("\r\n SetLoRaTx:%d\r\n",GET_TICK_COUNT( ));
}

static void SystemSleep(int second)
{
	SetLoRaSleep();
	SetIoSleep();
  StopMode_Measure(second);
	BSP_LED_On(LED1);
	BSP_LED_On(LED2);
}

int LoRaNodeCollectData(uint8_t *data)
{
  if(LoRaDevice.type == ELECTOR_METER)
    return dlt645_collect_meter(LoRaDevice.id,data,ENERGY_FORWARDA);	 //默认只采集正向有功总电能
		
	else if(LoRaDevice.type == WATER_METER || LoRaDevice.type == GAS_METER || LoRaDevice.type == HEART_METER)
		return cj188_collect_meter(LoRaDevice.id,data,LoRaDevice.type);	
  
	else
		return 0;
}

void SetLoRaCommPram(void)
{
	uint8_t len;
  if(LoRaClassType == LoRaClassC)              //监控呼吸ClassC 下不需要更改通讯参数
	{
			LoRaPayLoadInit();
			return;
	}
	len = 2 + GET_TICK_COUNT( ) % 4;
  Debug("\r\n len = %d",len);
	LoRaDevice.DevRxGap = len;                                              //随机设置接收窗口窗口时间  [0~6]
	LoRaDevice.PrebSymb = GetPreambSymbols(2*LORA_BREATH_TIME + LoRaDevice.DevRxGap*1000); //节点对应的前导码,网关发送前导码必须与此值一致才能与此节点通讯
	LoRaDevice.Heartbeat = LoRaDevice.Heartbeat;
	
	Debug("\r\nNode LoRaDevice.Heartbeat=%d,RxGap=%2ds,PreambleSymbols=%2d",LoRaDevice.Heartbeat,LoRaDevice.DevRxGap,LoRaDevice.PrebSymb);	

	PayLoadBuff[RXGAP_OFFSET] = LoRaDevice.DevRxGap - 1;                  //节点接收窗口,网关在LoRaDevice.DevRxGap提前1s发送前导码
	
  PayLoadBuff[PREAMB_OFFSET] = (LoRaDevice.PrebSymb >> 8) & 0x00FF;  //节点本次接收窗口前导码
	PayLoadBuff[PREAMB_OFFSET + 1] = LoRaDevice.PrebSymb & 0xFF;	

  PayLoadBuff[HEART_OFFSET] = (LoRaDevice.Heartbeat >> 8) & 0x00FF;  //节点心跳周期
	PayLoadBuff[HEART_OFFSET + 1] = LoRaDevice.Heartbeat & 0xFF;

  PayLoadBuff[COLLECT_OFFSET] = (LoRaDevice.Collect >> 8) & 0x00FF;  //节点采集周期
	PayLoadBuff[COLLECT_OFFSET + 1] = LoRaDevice.Collect & 0xFF;	
}

void LoRaNodeHeartBreath(void)
{
	uint8_t data[256] = {'\0'};
  uint8_t nbyte;
  SetLoRaCommPram();                             //LORA通讯参数初始化
  nbyte = LoRaNodeCollectData(data);             //采集数据
  HAL_UART_Transmit(&huart1,data, nbyte, 0xFFFF);
  HAL_Delay(2000);                                //波特率为2400时，传输256个字节时长为1秒
  pRFPayLoad->Dlen = 0;
	if(UsartType.RX_flag == 1)    	//  串口接收
		{ 
        UsartType.RX_flag=0;	// clean flag
				if(UsartType.RX_Size >= PAYLOAD_DATABUFF_SIZE)
					UsartType.RX_Size = PAYLOAD_DATABUFF_SIZE;
				
				pRFPayLoad->Dlen = UsartType.RX_Size;
				memcpy(pRFPayLoad->Buff,UsartType.RX_pData,pRFPayLoad->Dlen);                      //获取串口数据
    }
  PayLoadLen = (DATA_OFFSET + 1 + pRFPayLoad->Dlen);
  SX1276LoRaSetTxPacket( PayLoadBuff, PayLoadLen);     //将串口数据按照LORA帧格式组装，并发送出去!
	SetLoRaTx();
	
	Debug("\r\n=>%d Data to rf:",PayLoadLen);
	for(int i=0;i<PayLoadLen;i++)
		Debug("0x%2x ",PayLoadBuff[i]);
	
	LoRaProtocolState = TXWindow1;             //设置发送状态
}


void gateway_process(void)             //收到网关命令
{

  Debug("\r\n=>Reciver Host %d Data to LoRaDevice:\r\n",PayLoadLen);		
  HAL_UART_Transmit(&huart1,PayLoadBuff, PayLoadLen, 0xFFFF);
  RecvGatewayCmd = 1;
  RecvGatewayCmdTimeOut = GET_TICK_COUNT( );
}

/*******************************************************************************************
1、节点心跳，每隔Breath_time,自动唤醒上传一次数据
2、上行数据发送完成后，紧跟2个接收窗口，完成接收操作后再次进入休眠
3、在第一个接收窗口收到数据，直接进入休眠，忽略后一个接收窗口
4、网关必须一致处于接收监听状态，并在收到节点上行数据后，在确定的时间窗口发送下行数据
*******************************************************************************************/

void lora_protocol_process(void)
{
	static uint32_t delay = 0;
	switch (LoRaProtocolState)
	{
		case TXWindow1:
			LoRaProtocolState = TXWindow2;
			delay = GET_TICK_COUNT( );
			break;	
		case TXWindow2:
			if(RfResult == RF_TX_DONE)
			{
				LoRaProtocolState = RXWindow1Gap;
				Debug("\r\nRF_TX_DONE goto RXWindow1Gap ");
			}
			else
			{
				if((GET_TICK_COUNT( ) - delay) > TXWindowTimeOut)   //发送失败
				{
					SX1276LoRaSetPreambleLength(Preamble_Default);      // 恢复 前导码为默认值：8
					Debug("\r\nRF_RX_DONE goto sIDLE ");
					LoRaProtocolState = sIDLE;
				  SetLoRaSleep();
				}
			}
			break;	
		case RXWindow1Gap:
				Debug("\r\nRXWindow1Gap sleep");
				SystemSleep(LoRaDevice.DevRxGap);		
				Debug("\r\n%ds Wakeup!",LoRaDevice.DevRxGap);				
				SX1276LoRaSetPreambleLength(LoRaDevice.PrebSymb);                //设置接收时的前导码长度
        printf("\r\nSet PreambleLength = %d",LoRaDevice.PrebSymb);
		
				SetLoRaRx();
				LoRaProtocolState = RXWindow1;
			break;		
		case RXWindow1:
			if(RfResult == RF_RX_DONE)
			{
				gateway_process();
				SX1276LoRaSetPreambleLength(Preamble_Default);      //第一个接收窗口收到 数据后：恢复 前导码为默认值：8
        printf("\r\nResove PreambleLength = %d",Preamble_Default);
				LoRaProtocolState = sIDLE;
				SetLoRaSleep();
			}
			if(RfResult == RF_RX_TIMEOUT)
			{
				LoRaProtocolState = RXWindow2Gap;
			}
			break;
		case RXWindow2Gap:
			Debug("\r\nRXWindow2Gap sleep!");
			SystemSleep(LoRaDevice.DevRxGap);
			Debug("\r\n%ds Wakeup!",LoRaDevice.DevRxGap);		
			
			SetLoRaRx();
			LoRaProtocolState = RXWindow2;
			break;	
		case RXWindow2:
			if(RfResult == RF_RX_DONE)
			{
				gateway_process();				
				SX1276LoRaSetPreambleLength(Preamble_Default);      //第二个接收窗口收到数据后  恢复 前导码为默认值：8
        printf("\r\nResove PreambleLength = %d",Preamble_Default);
				SetLoRaSleep();
				LoRaProtocolState = sIDLE;
				Debug("\r\nRF_RX_DONE goto sIDLE ");
			}
			if(RfResult == RF_RX_TIMEOUT)
			{
				SX1276LoRaSetPreambleLength(Preamble_Default);      //第二个接收窗口没有收到数据 恢复 前导码为默认值：8
        printf("\r\nResove PreambleLength = %d",Preamble_Default);
				SetLoRaSleep();
				LoRaProtocolState = sIDLE;
				Debug("\r\nRF_RX_TIMEOUT goto sIDLE ");
			}
			break;
		case sIDLE:
			break;
		default:
			break;
	}
}

//输入流不超过251个字节，超过251字节自动截流，输入流间隔1S自动打包成一个数据流
void uart_Process(void)
{
//	if(LoRaProtocolState == sIDLE)
//	{
		if(UsartType.RX_flag == 1)    	// UART --> RF
			{ 
				if(LoRaProtocolState != sIDLE)
					return;

				SetLoRaCommPram();                             //LORA通讯参数初始化
				UsartType.RX_flag=0;	// clean flag	
				if(UsartType.RX_Size >= PAYLOAD_DATABUFF_SIZE)
					UsartType.RX_Size = PAYLOAD_DATABUFF_SIZE;
				
				pRFPayLoad->Dlen = UsartType.RX_Size;
				memcpy(pRFPayLoad->Buff,UsartType.RX_pData,pRFPayLoad->Dlen);                      //获取串口数据
				

				PayLoadLen = (DATA_OFFSET + 1 + pRFPayLoad->Dlen);
				SX1276LoRaSetTxPacket( PayLoadBuff, PayLoadLen);     //将串口数据按照LORA帧格式组装，并发送出去！
				SetLoRaTx();
				
				Debug("\r\n=>%d Data to rf:",PayLoadLen);
				for(int i=0;i<PayLoadLen;i++)
					Debug("0x%2x ",PayLoadBuff[i]);
									
				LoRaProtocolState = TXWindow1;             //设置发送状态
			} 
		if(UsartType.TX_flag == 1)    	//  RF --> UART  
		{
			UsartType.TX_flag = 0;
			if(PayLoadLen > DATA_OFFSET)
				UsartType.TX_Size = PayLoadLen - DATA_OFFSET;
			else
				UsartType.TX_Size = PayLoadLen;
			memcpy(UsartType.TX_pData,pRFPayLoad->Buff,UsartType.TX_Size);
			Debug("\r\n=>%d Data from rf:",UsartType.TX_Size);
				for(int i=0;i<UsartType.TX_Size;i++)
					Debug("0x%2x ",pRFPayLoad->Buff[i]);
			HAL_UART_Transmit(&huart1, UsartType.TX_pData, UsartType.TX_Size, 0xFFFF);
		}	
//	}	
}

void lora_process(void)
{
		RfResult = SX1276LoRaProcess();
		switch(RfResult)   
			{
				case RF_TX_DONE:
					BSP_LED_Off(LED1);
					SetLoRaSleep();
					break;
				case RF_RX_DONE:
					SX1276LoRaGetRxPacket(PayLoadBuff, &PayLoadLen );
				  BSP_LED_Off(LED1);				
					SetLoRaSleep();
					break;
				case RF_RX_TIMEOUT:
					BSP_LED_Off(LED1);
					SetLoRaSleep();
					break;
				default:
					break;
			}
}



void system_Status_process(void)
{
	if(RecvGatewayCmd == 1)
	{
		if((GET_TICK_COUNT( ) - RecvGatewayCmdTimeOut) > DEVICE_RESPONSE_TIMEOUT)   //设备超时无反馈
		{
			RecvGatewayCmd = 0;
		}
	}	
	else if(RfResult == RF_RX_DONE)
	{
		if(sIDLE == LoRaProtocolState)
		{
			UsartType.TX_flag = 1;	
		}
	}

	if(Lora_flag == SX1278_SLEEP && LoRaProtocolState == sIDLE && UsartType.RX_flag == 0 && UsartType.TX_flag == 0 &&	RecvGatewayCmd == 0)
	{
		Debug("\r\nSystem Goto Sleep,after %ds Wake up",LoRaDevice.Heartbeat);
		SystemSleep(LoRaDevice.Heartbeat);
		Debug("\r\nSystem Wake up!");
		
    LoRaNodeHeartBreath();                            //苏醒后进入心跳
	}

}

void system_Status_ClassA_process(void)
{
	if(RfResult == RF_RX_DONE)
	{
			UsartType.TX_flag = 1;	
	}

	if(Lora_flag == SX1278_SLEEP && LoRaProtocolState == sIDLE && UsartType.RX_flag == 0 && UsartType.TX_flag == 0)
	{
		Debug("\r\nSystem Goto Sleep,after %ds Wake up",LoRaDevice.Heartbeat);
		SystemSleep(LoRaDevice.Heartbeat);
		Debug("\r\nSystem Wake up!");
    SetLoRaRx();
	}
  LoRaProtocolState = sIDLE;
}

void LoRaSystemInit(void)
{
	UartPayLoadInit();
	LoRaPayLoadInit();
	UartReceive_DMA();
  LoRaDeviceInit();
}



void test(void)
{
	
	RTC_TimeTypeDef stime;
	Debug("\r\n testing!!!!");
  HAL_Delay(10000);
	while(1)
	{
    HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
	  Debug("\r\n stime.Hours:%d stime.Minutes:%d stime.Seconds:%d",stime.Hours,stime.Minutes,stime.Seconds);		
		Debug("\r\n---------------- sleep !");		
		BSP_LED_Off(LED1);
		SystemSleep(10);
		Debug("\r\nwake up from RTC !");
		BSP_LED_On(LED1);
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	SX1276Init();
	LoRaSystemInit();	
	Debug("Node Running!\r\n");		
  SetLoRaSleep();	
	HAL_Delay(10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
	//test();
	  if(LoRaClassType == LoRaClassC) //电表采用监控呼吸模块：ClassC
		{
			  lora_process();
				uart_Process();
        system_Status_ClassA_process();
		}
		else                 //其他采用监控呼吸模块：ClassC
		{	
				lora_process();
				lora_protocol_process();
				uart_Process();
				system_Status_process();
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 9, 0);
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: Debug("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
