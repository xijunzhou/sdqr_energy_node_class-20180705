#include <stdlib.h>
#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include "stm32l1xx_hal.h"
#include "usart.h"
#include "cj188.h"
#include "main.h"


/*  示例
const char METER_TEST_ACK[] = 
{0x68,0x10,0x06,0x00,0x04,0x00,0x00,0x00,0x00,
	0x81,
	0x16,
	0x1f,0x90,
	0x00,
	0x12,0x34,0x56,0x78,0x2c,
	0x23,0x45,0x67,0x89,0x2c,
	10,15,12,23,11,17,20,
	0x21,0x84,
	0x9d,
	0x16};
 */


int cj188_collect_meter(unsigned char *meter_id,unsigned char cj188[],int type)
{

	static unsigned char SER = 0;
	int i,len;
	unsigned char cs;
	cs = 0;
	len = 0;
	cj188[CJ188_START]= CJ188_FRAME_START;                         /*帧起始位   */  
	cj188[CJ188_TYPE] = type;
	
	for(i = 0 ;i < 5 ; i++)
		cj188[CJ188_ADDR + i] = meter_id[i];


	cj188[CJ188_CTRL] = CJ188_CTRL0_READ_DATA;                     /*控制码位   */   
	cj188[CJ188_DLEN] = 3;                                       /*数据长度位 */ 
	len = CJ188_DATA;
	
	cj188[len++] = CJ188_DI0_READ_DATA;                           /*数据位     */   
	cj188[len++] = CJ188_DI1_READ_DATA;
	cj188[len++] = SER;
	
  for(i=0;i<len;i++)                              
		{
			cs += cj188[i];
		}
	cj188[len++]= cs;                                                /*校验位     */ 
	cj188[len++]= CJ188_FRAME_END;	                                /*帧结束位   */
	
	return len;	
}
