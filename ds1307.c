#include "ds1307.h"

extern I2C_HandleTypeDef hi2c2;


uint8_t bcd2bin(uint8_t data){
 return ((data>>4)*10)+(data&0x0F);
}

uint8_t bin2bcd(uint8_t data){
	return ((data/10)<<4)|(data%10);
}

uint8_t rtc_read(uint8_t address)
{
	uint8_t data;
	
	if(HAL_I2C_Mem_Read(&hi2c2,addr_ds1307,address,I2C_MEMADD_SIZE_8BIT,&data,1,100)!=HAL_OK){
		_Error_Handler(__FILE__,__LINE__);
	}
	
	return data;
}

void rtc_write(uint8_t address,uint8_t data)
{
	if(HAL_I2C_Mem_Write(&hi2c2,addr_ds1307,address,I2C_MEMADD_SIZE_8BIT,&data,1,100)!=HAL_OK){
		_Error_Handler(__FILE__,__LINE__);
	}
}

void rtc_init(uint8_t rs,uint8_t sqwe,uint8_t out)
{
	rs&=3;
	if (sqwe) rs|=0x10;
	if (out) rs|=0x80;
	
	rtc_write(0x07,rs);
}

void rtc_get_time(uint8_t *hour,uint8_t *min,uint8_t *sec)
{
	uint8_t data[3];
	if(HAL_I2C_Mem_Read(&hi2c2,addr_ds1307,0x00,I2C_MEMADD_SIZE_8BIT,data,3,1000)!=HAL_OK){
		_Error_Handler(__FILE__,__LINE__);
	}	

	*sec=bcd2bin(data[0]);
	*min=bcd2bin(data[1]);
	*hour=bcd2bin(data[2]);
}

void rtc_set_time(uint8_t hour,uint8_t min,uint8_t sec)
{
	uint8_t data[3]={bin2bcd(sec),bin2bcd(min),bin2bcd(hour)};
	if(HAL_I2C_Mem_Write(&hi2c2,addr_ds1307,0x00,I2C_MEMADD_SIZE_8BIT,data,3,1000)!=HAL_OK){
		_Error_Handler(__FILE__,__LINE__);
	}
}

void rtc_get_date(uint8_t *week_day,uint8_t *day,uint8_t *month,uint8_t *year)
{
	uint8_t data[4]={0,0,0,0};
	if(HAL_I2C_Mem_Read(&hi2c2,addr_ds1307,0x03,I2C_MEMADD_SIZE_8BIT,data,4,1000)!=HAL_OK){
		_Error_Handler(__FILE__,__LINE__);
	}
	
	*week_day=data[0];
	*day=bcd2bin(data[1]);
	*month=bcd2bin(data[2]);
	*year=bcd2bin(data[3]);
}

void rtc_set_date(uint8_t week_day,uint8_t day,uint8_t month,uint8_t year)
{
	uint8_t data[4]={week_day,bin2bcd(day),bin2bcd(month),bin2bcd(year)};
	if(HAL_I2C_Mem_Write(&hi2c2,addr_ds1307,0x03,I2C_MEMADD_SIZE_8BIT,data,4,1000)!=HAL_OK)
		{
		_Error_Handler(__FILE__,__LINE__);
		}
}
