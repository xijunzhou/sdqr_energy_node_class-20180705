#ifndef _PROTOCOLS_
#define _PROTOCOLS_

#define DLT645_TYPE_NUM  30

#define UN_DEFINE_TYPE   -1
#define ENERGY_FORWARDA  0  /*�����й��ܵ���*/
#define ENERGY_BACKWARDA 1    /*�����й��ܵ���*/
#define VOLTAGE 2             /*��ѹ/(A)��ѹ*/
#define CURRENT 3            /*����/(A)����*/
#define POWER_ACT 4           /*˲ʱ���й�����*/
#define POWER_REACT 5        /*˲ʱ���޹�����*/
#define FACTOR 6               /*�ܹ�������*/
#define FREQUENCY 7           /*����Ƶ��*/


uint32_t bcds_to_int(uint8_t bcds[],uint16_t len);
uint8_t *BcdToString(uint8_t *Pchar,uint8_t data[],uint16_t nbyte);
uint16_t ChangeToBcd(uint8_t iChTemp);
void StringToBcd(uint8_t *Schar,uint8_t bcd[],uint16_t f_rev);

uint8_t dlt645_encode(uint8_t dlt645[],uint8_t *meter_id,uint16_t type);
int dlt645_decode(uint8_t data[],uint16_t nbyte,uint8_t *meter_id,uint16_t *Rdata);

extern uint32_t dlt645_collect_data[DLT645_TYPE_NUM];

#endif