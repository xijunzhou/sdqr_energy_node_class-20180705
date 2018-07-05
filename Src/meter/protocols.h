#ifndef _PROTOCOLS_
#define _PROTOCOLS_

#define DLT645_TYPE_NUM  30

#define UN_DEFINE_TYPE   -1
#define ENERGY_FORWARDA  0  /*正向有功总电能*/
#define ENERGY_BACKWARDA 1    /*反向有功总电能*/
#define VOLTAGE 2             /*电压/(A)电压*/
#define CURRENT 3            /*电流/(A)电流*/
#define POWER_ACT 4           /*瞬时总有功功率*/
#define POWER_REACT 5        /*瞬时总无功功率*/
#define FACTOR 6               /*总功率因素*/
#define FREQUENCY 7           /*电网频率*/


uint32_t bcds_to_int(uint8_t bcds[],uint16_t len);
uint8_t *BcdToString(uint8_t *Pchar,uint8_t data[],uint16_t nbyte);
uint16_t ChangeToBcd(uint8_t iChTemp);
void StringToBcd(uint8_t *Schar,uint8_t bcd[],uint16_t f_rev);

uint8_t dlt645_encode(uint8_t dlt645[],uint8_t *meter_id,uint16_t type);
int dlt645_decode(uint8_t data[],uint16_t nbyte,uint8_t *meter_id,uint16_t *Rdata);

extern uint32_t dlt645_collect_data[DLT645_TYPE_NUM];

#endif