
#ifndef _SCCB_BUS_H_
#define _SCCB_BUS_H_


#include "main.h"

#define I2C_TIM 5

#define SCCB_SIC_H()      HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET)   //SCL H
#define SCCB_SIC_L()      HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET) //SCL L
#define SCCB_SID_H()      HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET)   //SDA  H
#define SCCB_SID_L()      HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET) //SDA  L

#define SCCB_DATA_IN     {SDA_GPIO_Port->MODER &= ~(GPIO_MODER_MODE6);}
#define SCCB_DATA_OUT    {SDA_GPIO_Port->MODER |= GPIO_MODER_MODE6_0;}

#define SCCB_SID_STATE   HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin)

void sccb_bus_init(void);
void sccb_bus_start(void);
void sccb_bus_stop(void);
void sccb_bus_send_noack(void);
void sccb_bus_send_ack(void);
uint8_t sccb_bus_write_byte(uint8_t data);
uint8_t sccb_bus_read_byte(void);

#endif /* _SCCB_BUS_H_ */
