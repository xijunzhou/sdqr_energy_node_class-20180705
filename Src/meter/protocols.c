#include <stdlib.h>
#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "protocols.h"

#define DLT645_START  0    /*֡��ʼλ*/
#define DLT645_ADDR 1     /*��ַ��*/
#define DLT645_DSTART 7   /*������ʼλ*/
#define DLT645_CTRL 8      /*������λ*/
#define DLT645_DLEN 9	       /*���ݳ���λ*/
#define DLT645_DATA 10         /*����λ*/
#define DLT645_CS            /*У��λ*/
#define DLT645_END           /*֡����λ*/

const uint8_t DLT645_Data_Mark[][4] =          /*���ݱ�ʶ��*/
{
	{0x33,0x33,0x34,0x33}, /*ENERGY_FORWARDA */
	{0x33,0x33,0x35,0x33}, /*ENERGY_BACKWARDA*/
	{0x33,0x34,0x34,0x35}, /*VOLTAGE;        */
	{0x33,0x34,0x35,0x35}, /*CURRENT;	      */
	{0x33,0x33,0x36,0x35}, /*POWER_ACT;      */
	{0x33,0x33,0x37,0x35}, /*POWER_REACT;	  */
	{0x33,0x33,0x38,0x35}, /*FACTOR;         */
	{0x35,0x33,0xb3,0x35}, /*FREQUENCY;     */
	{},
};

const uint8_t DLT645_BroadCast_Addr[6] = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};/*�㲥��ַ����*/
const uint8_t DLT645_CTRL_CODE_ENERGY = 0x11;/*�����룺�����ܱ�����*/
const uint8_t DLT645_CTRL_CODE_TRIP = 0x1C;/*�����룺��բ����բ*/
const uint8_t TRIP_OPEN[] = {0x1a,0xff,0,0,0,0,0,0};/*��բ*/
const uint8_t TRIP_CLOSE[] = {0x1b,0xff,0,0,0,0,0,0};/*��բ*/
uint8_t DLT645_METER_ID[6] = {0x00,0x06,0x00,0x04,0x00,0x00,0x00};

#define DLT645_FRAME_START  0x68
#define DLT645_FRAME_END  0x16
#define DLT645_FRAME_READ_DATA 0x11
#define DLT645_C_OK	0x91
#define DLT645_C_ERRO 0xd1
#define DLT645_DATA_OFFSET 0x33
/*
dlt645_collect_data[0]:						�ɼ������й��ܵ���
dlt645_collect_data[1]:						�ɼ������й��ܵ���
dlt645_collect_data[2]:						�ɼ���ѹ
dlt645_collect_data[3]:						�ɼ����� 
dlt645_collect_data[4]:						�ɼ�˲ʱ���й�����
dlt645_collect_data[5]:						�ɼ�˲ʱ���޹�����
dlt645_collect_data[6]:						�ɼ���������
dlt645_collect_data[7]:						�ɼ�����Ƶ��
*/
uint32_t dlt645_collect_data[DLT645_TYPE_NUM];


/****
  * Function: uint32_t bcds_to_int(uint8_t bcds[],uint16_t len)
  *@Describe:��BCD����ת���ɳ��������ݡ���������ݾ��ȵ�λ���λ 
             DLT645_DATA_OFFSET Ϊ����DLT645�Ķ��ؼ������
             ������bcds[] = 12 34 56 78   ����ֵΪʮ���Ƶģ�78563412
  *@Param_1 : bcds[],��ת����BCD����
  *@Param_2 : len��BCD���鳤��
  *@Param_3 : 
  *@Return  : ���� ת����ĳ�������
  */
uint32_t bcds_to_int(uint8_t bcds[],uint16_t len)
{
	uint16_t i;
	uint32_t rate,res;
	res = 0;
	rate = 1;
	for(i=0;i<len;i++)
	{		
		res += (((bcds[i] - DLT645_DATA_OFFSET) & 0x0f)) * rate;
		rate = rate * 10;
		res += (((bcds[i] - DLT645_DATA_OFFSET) >> 4)) * rate;
		rate = rate * 10;
	}
	return res;
}



/****
  * Function: uint16_t ChangeToBcd(uint8_t iChTemp)
  *@Describe: �ַ�ת����BCD
  *@Param_1 : uint8_t iChTemp Ҫת�����ַ���
  *@Param_2 :
  *@Param_3 : 
  *@Return  : 
  */
uint16_t ChangeToBcd(uint8_t iChTemp)
{
	uint16_t m;
	if( iChTemp>='0' && iChTemp<='9' )
	{
		m=iChTemp-'0';
	}
	else if( iChTemp>='A' && iChTemp<='Z' )
	{
		m=iChTemp-'A'+10;
	}
	else if( iChTemp>='a' && iChTemp<='z' )
	{
		m=iChTemp-'a'+10;
	}
	return m;
}

/****
  * Function: void StringToBcd(uint8_t *Schar,uint8_t bcd[],uint16_t f_rev)
  *@Describe: �ַ���ת��ΪBCD����
  *@Param_1 : uint8_t *Schar Ҫת�����ַ���
  *@Param_2 : uint8_t bcd[] ת�������BCD����
  *@Param_3 : uint16_t f_rev  ����洢��ʽ������0 ����1
  *@Return  : 
  */
void StringToBcd(uint8_t *Schar,uint8_t bcd[],uint16_t f_rev) 
{
	uint16_t i,len;
	len = strlen(Schar);
	for(i = 0; i < len; i+=2)
	{
		if(f_rev ==1)
			{
				bcd[(len-i)/2 - 1] = ChangeToBcd(*Schar++) << 4;
				bcd[(len-i)/2 - 1] += ChangeToBcd(*Schar++);
			}
		else
			{
				bcd[i/2] = ChangeToBcd(*Schar++) << 4;
				bcd[i/2] += ChangeToBcd(*Schar++);
			}
	}

}

/****
  * Function: uint8_t *BcdToString(uint8_t *Pchar,const uint8_t data[],uint16_t nbyte)
  *@Describe: BCD����ת�����ַ���
  *@Param_1 : 
  *@Param_2 : 
  *@Param_3 : 
  *@Return  : �ɹ������� 0��ʧ�ܣ����� -1.
  */
uint8_t *BcdToString(uint8_t *Pchar, uint8_t data[],uint16_t nbyte)
{
	uint16_t i;
	uint8_t *src = Pchar;
	for(i = 0; i < nbyte ; i++,Pchar)
	{
   if((data[i] >> 4) >= 0 &&  (data[i] >> 4) <= 9)
           *Pchar++ = (data[i] >> 4) + '0';
   else if((data[i] >> 4) >= 10 &&  (data[i] >> 4) <= 15)
           *Pchar++ = (data[i] >> 4) + 'a' - 10;
   if((data[i] & 0x0f) >= 0 &&  (data[i] & 0x0f) <= 9)
           *Pchar++ = (data[i] & 0x0f) + '0';
   else if((data[i] & 0x0f) >= 10 &&  (data[i] & 0x0f) <= 15)
           *Pchar++ = (data[i] & 0x0f) + 'a' - 10;	
	}
	*Pchar = '\0';
	return src;
}

int dlt645_decode(uint8_t data[],uint16_t nbyte,uint8_t *meter_id,uint16_t *Rdata)
{
	uint16_t res;
	uint8_t flag,i,j,cs;
	i = 0;
	j = 0;
	cs = 0;
	flag = 0;
	if(nbyte > 256)
		return -1;
	while(i < nbyte)
	{
		switch(flag)
		{
			case 0:
				if(data[i] == 0x68)
					{
						flag = 1;
						j = i;
						meter_id = data[i + 1];
					}
				break;
			case 1:				
				cs += data[i -1];
				if((data[i] == 0x68) && (i - j == 7))
					{
						flag = 2;
						j = i;
					}
				break;
			case 2:
				cs += data[i -1];
				if(i - j == 2)
					{
						flag = 3;
						j = data[i];
					}
				break;	
			case 3:
				cs += data[i -1];
				if(i == j + 10)
					{
						if(cs == data[i])
						{
							BcdToString(meter_id,&data[1],6);
							//printf("cs ok!\n");
							if(data[DLT645_CTRL] == 0xd1)						
								{
							//		printf("��֧�ֵ�������,������:0xd1\n");
									return -1;
								}
							for(i = 0 ; i <= 7 ; i ++)
							{
								if (0 == memcpy(DLT645_Data_Mark[i][0],data[10],4))
									break;							 	
							}
							switch(i)
							{
								case ENERGY_FORWARDA:     /*�����й��ܵ���*/									
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],4);
									break;
								case ENERGY_BACKWARDA:    /*�����й��ܵ���*/								
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],4);
									break;
								case VOLTAGE:            /*��ѹ/(A)��ѹ*/								
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],2);
									break;
								case CURRENT:	            /*����/(A)����*/									
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],3);
									break;
								case POWER_ACT:            /*˲ʱ���й�����*/								
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],3);
									break;
								case POWER_REACT:         /*˲ʱ���޹�����*/								
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],3);
									break;
								case FACTOR:              /*�ܹ�������*/							
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],2);
									break;
								case FREQUENCY:            /*����Ƶ��*/								
									*Rdata = bcds_to_int(&data[DLT645_DATA + 4],2);
									break;	
								default:
									return -1;
									break;
							}
							return res;
						}
						else
							return -1;
					}
				break;	
			default:
				return -1;
				break;		
		}
	i++;	
	}
	return -1;			
}



/****
  * Function: void dlt645_encode(uint8_t dlt645[],uint8_t *meter_id,uint16_t type)
  *@Describe:��ȡ������������dlt645��������֡��װ
  *@Param_1 : uint8_t *meter_id,��ȡ���ݶ�Ӧ�ĵ��id
  *@Param_2 : type����������
  *@Param_3 : 
  *@Return  : ���� long���������� 
  */
uint8_t dlt645_encode(uint8_t dlt645[],uint8_t *meter_id,uint16_t type)
{
	uint16_t i,len;
	uint8_t cs;
	cs = 0;
	len = 0;
	dlt645[DLT645_START]= DLT645_FRAME_START;                         /*֡��ʼλ   */  
	StringToBcd(meter_id,&dlt645[DLT645_ADDR],1);                     /*��ַ��     */ 
	dlt645[DLT645_DSTART]= DLT645_FRAME_START;                        /*������ʼλ */ 
	dlt645[DLT645_CTRL] = DLT645_FRAME_READ_DATA;                     /*������λ   */   
	dlt645[DLT645_DLEN] = 4;                                          /*���ݳ���λ */ 
	len = DLT645_DATA;
	for(i=0;i<dlt645[DLT645_DLEN];i++)
		dlt645[len++] = DLT645_Data_Mark[type][i];                      /*����λ     */     
  for(i=0;i<len;i++)                              
		{
			cs += dlt645[i];
		}
	dlt645[len++]= cs;                                                /*У��λ     */ 
	dlt645[len++]= DLT645_FRAME_END;	                                /*֡����λ   */
	return len;
}
