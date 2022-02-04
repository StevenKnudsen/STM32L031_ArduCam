/*
 * spi.h
 *
 *  Created on: Jan 27, 2022
 *      Author: knud
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "main.h"

#include "stdbool.h"

#define  BUFFER_MAX_SIZE 4096
extern uint32_t sendlen ;
extern uint32_t haveRev ;
extern uint32_t noRev;
extern uint8_t *picbuf  ;
extern bool receive_OK;
extern bool send_OK ;
extern uint8_t  Buf1[BUFFER_MAX_SIZE];
extern uint8_t  Buf2[BUFFER_MAX_SIZE];
extern uint8_t  EP2_SendFinish;


//#define  PB_SPI_RX_DMA_Channel       DMA1_Channel4
//#define  PB_SPI_DMA_IRQ          DMA1_Channel4_IRQn
//#define  PB_SPI_DMA_IRQHandler     DMA1_Channel4_IRQHandler
#define  CS_PORT  SPI1_CS_GPIO_Port
#define  CS_PIN   SPI1_CS_Pin

void SPI_WriteByte(uint8_t data);

uint8_t SPI_ReadByte(uint8_t data);

uint8_t SPI_ReadWriteByte(uint8_t txData);

//uint8_t DMA1_Init(void);
//void SPI2_Init(void);
//void SPI2_SetSpeed(uint8_t SpeedSet);
//uint8_t SPI2_ReadWriteByte(uint8_t TxData);
//void DMA1_RX(uint8_t *p , uint32_t len);
//void DMA1_SendtoUsart(uint8_t *p , uint32_t len);
//void SendbyUSART1( void);
void SingleCapTransfer(void);
//void StartBMPcapture(void);
void CS_HIGH(void);
void CS_LOW(void);


#endif /* INC_SPI_H_ */
