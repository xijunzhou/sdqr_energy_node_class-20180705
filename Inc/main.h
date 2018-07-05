/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32l1xx_hal.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SX1278_DIO0_Pin GPIO_PIN_0
#define SX1278_DIO0_GPIO_Port GPIOB
#define SX1278_DIO1_Pin GPIO_PIN_1
#define SX1278_DIO1_GPIO_Port GPIOB
#define SX1278_DIO2_Pin GPIO_PIN_2
#define SX1278_DIO2_GPIO_Port GPIOB
#define SX1278_NSS_Pin GPIO_PIN_12
#define SX1278_NSS_GPIO_Port GPIOB
#define SX1278_SCK_Pin GPIO_PIN_13
#define SX1278_SCK_GPIO_Port GPIOB
#define SX1278_MISO_Pin GPIO_PIN_14
#define SX1278_MISO_GPIO_Port GPIOB
#define SX1278_MOSI_Pin GPIO_PIN_15
#define SX1278_MOSI_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOA
#define SX1278_RESET_Pin GPIO_PIN_15
#define SX1278_RESET_GPIO_Port GPIOA
#define SX1278_DIO3_Pin GPIO_PIN_3
#define SX1278_DIO3_GPIO_Port GPIOB
#define SX1278_DIO4_Pin GPIO_PIN_4
#define SX1278_DIO4_GPIO_Port GPIOB
#define SX1278_DIO5_Pin GPIO_PIN_5
#define SX1278_DIO5_GPIO_Port GPIOB
#define METER_TX_2400_Pin GPIO_PIN_6
#define METER_TX_2400_GPIO_Port GPIOB
#define METER_RX_2400_Pin GPIO_PIN_7
#define METER_RX_2400_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SWDAT_PIN GPIO_PIN_13
#define SWCLK_PIN GPIO_PIN_14
/*


GPIOB:  GPIO_PIN_0 ~ GPIO_PIN_5 ?GPIO_PIN_8?GPIO_PIN_12 ~ GPIO_PIN_15 ?GPIO_PIN_6 | GPIO_PIN_7
	DIO0_Pin | DIO1_Pin | DIO2_Pin | DIO3_Pin | DIO4_Pin | DIO5_Pin | RADIO_ANT_SWITCH_LF_Pin | SX1278_NSS_Pin | SX1278_SCK_Pin | SX1278_MISO_Pin | SX1278_MOSI_Pin |

	Unuser IO: GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
	Low  in: DIO0_Pin | DIO1_Pin | DIO2_Pin | DIO3_Pin | DIO4_Pin | DIO5_Pin
	HIGH in :others    ~(DIO0_Pin | DIO1_Pin | DIO2_Pin | DIO3_Pin | DIO4_Pin | DIO5_Pin)
	
	
	GPIO_InitStruct.Pin = DIO0_Pin | DIO1_Pin | DIO2_Pin | DIO3_Pin | DIO4_Pin | DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ~(DIO0_Pin | DIO1_Pin | DIO2_Pin | DIO3_Pin | DIO4_Pin | DIO5_Pin)
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
----------------------------------------------------------------------------------------------------
GPIOA:  GPIO_PIN_9 ? GPIO_PIN_10 ?GPIO_PIN_15
	LED2_Pin | LED1_Pin | SX1278_RESET_Pin

	Unuser IO: ~(LED2_Pin | LED1_Pin | SX1278_RESET_Pin)
	Low  out: GPIO_PIN_9 | GPIO_PIN_10
	HIGH out: GPIO_PIN_15
	HIGH in :others           ~(GPIO_PIN_15 | GPIO_PIN_10 | GPIO_PIN_9)

  GPIO_InitStruct.Pin = LED2_Pin | LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = SX1278_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ~(GPIO_PIN_9 | GPIO_PIN_10 | SX1278_RESET_Pin);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
---------------------------------------------------------------------------------------------	
GPIOC:GPIO_PIN_13
	RADIO_ANT_SWITCH_PWR_Pin

  GPIO_InitStruct.Pin = ~RADIO_ANT_SWITCH_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
  
-----------------------------------
	GPIO_InitStruct.Pin = DIO0_Pin | DIO1_Pin | DIO2_Pin | DIO3_Pin | DIO4_Pin | DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ~(DIO0_Pin | DIO1_Pin | DIO2_Pin | DIO3_Pin | DIO4_Pin | DIO5_Pin)
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = LED2_Pin | LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = SX1278_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ~(GPIO_PIN_9 | GPIO_PIN_10 | SX1278_RESET_Pin);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
  
  GPIO_InitStruct.Pin = ~RADIO_ANT_SWITCH_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
-----------------------------------  
*/


#define VALID_HEADER_DETECTED 1
#define JOHN_DEBUG 1

#define RF_DEBUG 0

#if(JOHN_DEBUG) 
#define Debug(...) printf(__VA_ARGS__)
#else
#define Debug(...)  
#endif

#if(RF_DEBUG) 
#define RFDebug(...) printf(__VA_ARGS__)
#else
#define RFDebug(...)  
#endif

#define PAYLOAD_ADDRESS_LEN  4
#define PAYLOAD_MAX_SIZE 255            /*禺粘一艤蝇诮250俣貣逇*/

#define PAYLOAD_DATABUFF_SIZE 230            //鏈?闀垮厑璁?230涓敤鎴锋暟鎹?

#define RX_LEN   1024
#define TX_LEN   1024 

#define LORA_SLEEP_TIME	 4000   /*休眠时间：*/
#define LORA_BREATH_TIME	10   /*呼吸时间：*/


#define DEVICE_RESPONSE_TIMEOUT  1000   //设备反馈时间，从串口收到数据后反馈，如有必要！

#define Preamble_Default        (uint16_t)(((LORA_SLEEP_TIME + 2 * LORA_BREATH_TIME) * RF_Bw / RF_Sf) - 4) 
//#define Preamble_Default                          8


#define MAX_METER 16
#define METER_DATA_NUM 1               //榛樿鍙噰闆嗘鍚戞湁鍔熸?荤數鑳斤紝METER_DATA_NUM = 1锛?
#define MATER_DATA_ERRO 0xffffffff



typedef struct  
{  
uint8_t  RX_flag;        //IDLE receive flag
uint16_t RX_Size;          //receive length
uint8_t  RX_pData[RX_LEN]; //DMA receive buffer
uint8_t  TX_flag;        //IDLE tx flag
uint16_t TX_Size;          //tx length
uint8_t  TX_pData[TX_LEN]; //DMA tx buffer
}USART_RECEIVETYPE;
extern USART_RECEIVETYPE UsartType; 
extern const uint8_t NetAddr[2];
extern const uint8_t NodeAddr[2];
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
