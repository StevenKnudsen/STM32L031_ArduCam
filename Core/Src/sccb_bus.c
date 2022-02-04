#include "sccb_bus.h"
#include "delay.h"

void sccb_bus_init(void)
{
  // @todo Not sure this is actually needed
  SCCB_DATA_OUT;
  SCCB_SIC_H();
  SCCB_SID_H();
  delay_ms(1);
}

void sccb_bus_start(void)
{
  SCCB_SID_H();
  delay_us(I2C_TIM);
  SCCB_SIC_H();
  delay_us(I2C_TIM);
  SCCB_SID_L();
  delay_us(I2C_TIM);
  SCCB_SIC_L();
  delay_us(I2C_TIM);
}


void sccb_bus_stop(void)
{
  SCCB_SID_L();
  delay_us(I2C_TIM);
  SCCB_SIC_H();
  delay_us(I2C_TIM);
  SCCB_SID_H();
  delay_us(I2C_TIM);
}


void sccb_bus_send_noack(void)
{	
  SCCB_SID_H();
  delay_us(I2C_TIM);
  SCCB_SIC_H();
  delay_us(I2C_TIM);
  SCCB_SIC_L();
  delay_us(I2C_TIM);
  SCCB_SID_L();
  delay_us(I2C_TIM);
}

void sccb_bus_send_ack(void)
{	
  SCCB_SID_L();
  delay_us(I2C_TIM);
  SCCB_SIC_L();
  delay_us(I2C_TIM);
  SCCB_SIC_H();
  delay_us(I2C_TIM);
  SCCB_SIC_L();
  delay_us(I2C_TIM);
  SCCB_SID_L();
  delay_us(I2C_TIM);
}

uint8_t sccb_bus_write_byte(uint8_t data)
{
  uint32_t i;
  uint8_t tem;

  for(i = 0; i < 8; i++)
  {
    if((data<<i) & 0x80)
    {
      SCCB_SID_H();
    }
    else
    {
      SCCB_SID_L();
    }
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    SCCB_SIC_L();

  }
  SCCB_DATA_IN;
  delay_us(I2C_TIM);
  SCCB_SIC_H();
  delay_us(I2C_TIM);
  if(SCCB_SID_STATE)
  {
    tem = 0;
  }
  else
  {
    tem = 1;
  }

  SCCB_SIC_L();
  delay_us(I2C_TIM);
  SCCB_DATA_OUT;
  return tem;
}

uint8_t sccb_bus_read_byte(void)
{	
  uint32_t i;
  uint8_t read = 0;

  SCCB_DATA_IN;
  delay_us(I2C_TIM);
  for(i = 8; i > 0; i--)
  {
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    read = read << 1;
    if(SCCB_SID_STATE)
    {
      read += 1;
    }
    SCCB_SIC_L();
    delay_us(I2C_TIM);
  }
  SCCB_DATA_OUT;
  return read;
}

