#include <stdlib.h>
#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include "stm32l1xx_hal.h"
#include "dlt645.h"
#include "usart.h"
#include "main.h"


/*       DLT645 ?????         */

const char DLT645_Data_Mark[][4] =
{
	{0x33,0x33,0x34,0x33}, /*NOW ENERGY_FORWARDA */
	{0x33,0x33,0x35,0x33}, /*NOW ENERGY_BACKWARDA*/
	{0x33,0x34,0x34,0x35}, /*NOW VOLTAGE;        */
	{0x33,0x34,0x35,0x35}, /*NOW CURRENT;	      */
	{0x33,0x33,0x36,0x35}, /*NOW POWER_ACT;      */
	{0x33,0x33,0x37,0x35}, /*NOW POWER_REACT;	  */
	{0x33,0x33,0x38,0x35}, /*NOW FACTOR;         */
	{0x35,0x33,0xb3,0x35}, /*NOW FREQUENCY;     */			
	{0x33,0x33,0x36,0x33}, /*NOW EnergyReactCombination1*/   
	{0x33,0x33,0x37,0x33}, /*NOW EnergyReactCombination2*/	
	{0x33,0x33,0x38,0x33},/*NOW Quadrant1ReactCombination*/
	{0x33,0x33,0x39,0x33},/*NOW Quadrant2ReactCombination*/
	{0x33,0x33,0x3A,0x33},/*NOW Quadrant3ReactCombination*/
	{0x33,0x33,0x3B,0x33},/*NOW Quadrant4ReactCombination*/		
	{0x33,0x33,0x34,0x34}, /*NOW ActiveForwardDemand;     */	
	{0x33,0x33,0x35,0x34}, /*NOW ReactForwardDemand;     */
	{0x35,0x33,0x33,0x39} /*NOW LOADRECORD;     */
};




int dlt645_collect_meter(uint8_t *meter_id,uint8_t dlt645[],uint16_t type)
{

	uint16_t i,len;
	
	uint8_t cs;
	cs = 0;
	len = 0;

	dlt645[DLT645_START]= DLT645_FRAME_START;                         /*֡ǰʼλ   */  

	for(i = 0 ;i < 6 ; i++)
		dlt645[DLT645_ADDR + i] = meter_id[i];
	
	dlt645[DLT645_DSTART]= DLT645_FRAME_START;                        /*˽ߝǰʼλ */ 
	dlt645[DLT645_CTRL] = DLT645_FRAME_READ_DATA;                     /*࠘׆ëλ   */   
	dlt645[DLT645_DLEN] = 4;                                          /*˽ߝӤ׈λ */ 
	len = DLT645_DATA;
	for(i=0;i<dlt645[DLT645_DLEN];i++)
		dlt645[len++] = DLT645_Data_Mark[type][i];                      /*˽ߝλ     */     
  for(i=0;i<len;i++)                              
		{
			cs += dlt645[i];
		}
	dlt645[len++]= cs;                                                /*Уҩλ     */ 
	dlt645[len++]= DLT645_FRAME_END;	                                /*֡ޡ˸λ   */
	
  
	return len;
}
