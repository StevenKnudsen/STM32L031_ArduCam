#include "ArduCAM.h"
#include "ov5642_regs.h"
#include "sccb_bus.h"
#include "spi.h"

//byte sensor_model = 0;
//byte sensor_addr = 0;
byte m_fmt = JPEG;
uint32_t length = 0;
uint8_t is_header= false ;

void ArduCAM_Init(byte address)
{
  wrSensorReg16_8(address, 0x3008, 0x80);
  wrSensorRegs16_8(address, OV5642_QVGA_Preview);

  if (m_fmt == JPEG)
  {
    wrSensorRegs16_8(address, OV5642_JPEG_Capture_QSXGA);
    wrSensorRegs16_8(address, ov5642_320x240);
    wrSensorReg16_8(address, 0x3818, 0xa8);
    wrSensorReg16_8(address, 0x3621, 0x10);
    wrSensorReg16_8(address, 0x3801, 0xb0);
    wrSensorReg16_8(address, 0x4407, 0x04);
  }
  else
  {
    byte reg_val;
    wrSensorReg16_8(address, 0x4740, 0x21);
    wrSensorReg16_8(address, 0x501e, 0x2a);
    wrSensorReg16_8(address, 0x5002, 0xf8);
    wrSensorReg16_8(address, 0x501f, 0x01);
    wrSensorReg16_8(address, 0x4300, 0x61);
    rdSensorReg16_8(address, 0x3818, &reg_val);
    wrSensorReg16_8(address, 0x3818, (reg_val | 0x60) & 0xff);
    rdSensorReg16_8(address, 0x3621, &reg_val);
    wrSensorReg16_8(address, 0x3621, reg_val & 0xdf);
  }
  write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH

  uint8_t _x3503 = 0; // @todo KSK forced this to be initialized as 0, was originally uninitialized
  wrSensorReg16_8(address, 0x5001,_x3503|0x01);	 // Close auto exposure mode
  //Manually set the exposure value
  wrSensorReg16_8(address, 0x3500,0x00);
  wrSensorReg16_8(address, 0x3501,0x79);
  wrSensorReg16_8(address, 0x3502,0xe0);

}
////CS init
//void ArduCAM_CS_init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Pin =  CS_PIN;		
//	GPIO_Init(CS_PORT, &GPIO_InitStructure);
//	CS_HIGH();	
//}

////Ö¸Ê¾µÆ³õÊ¼»¯
//void ArduCAM_LED_init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Pin =  LED_PIN;		
//	GPIO_Init(LED_PORT, &GPIO_InitStructure);
////***********************************************
//	//   debug pin 
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;		
//	GPIO_Init(GPIOE, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOE,GPIO_Pin_2);
////************************************************/
//}


void set_format(byte fmt)
{
  if (fmt == BMP)
    m_fmt = BMP;
  else
    m_fmt = JPEG;
}

uint8_t bus_read(int address)
{
  uint8_t value;
  CS_LOW();
  SPI_WriteByte(address);
  value = SPI_ReadByte(0x00);
  CS_HIGH();
  return value;
}
//uint8_t my_bus_read(int address)
//{
//  uint8_t value;
//  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
//
////  SPI1_ReadWriteByte(address);
////  value = SPI1_ReadWriteByte(0xAA); // dummy value
//  SPI1_WriteByte(address);
//  value = SPI1_ReadByte(0xAA);
//  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//  return value;
//}

uint8_t bus_write(int address,int value)
{	
  CS_LOW();
  delay_us(10);
  SPI_WriteByte(address);
  SPI_WriteByte(value);
  delay_us(10);
  CS_HIGH();
  return 1;
}

uint8_t read_reg(uint8_t addr)
{
  uint8_t data;
  data = bus_read(addr & 0x7F);
  return data;
}

void write_reg(uint8_t addr, uint8_t data)
{
  bus_write(addr | 0x80, data);
}

uint8_t read_fifo(void)
{
  uint8_t data;
  data = bus_read(SINGLE_FIFO_READ);
  return data;
}
void set_fifo_burst()
{
  SPI_ReadWriteByte(BURST_FIFO_READ);
}


void flush_fifo(void)
{
  write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void start_capture(void)
{
  write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void clear_fifo_flag(void )
{
  write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t read_fifo_length(void)
{
  uint32_t len1,len2,len3,len=0;
  len1 = read_reg(FIFO_SIZE1);
  len2 = read_reg(FIFO_SIZE2);
  len3 = read_reg(FIFO_SIZE3) & 0x7f;
  len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
  return len;
}

//Set corresponding bit  
void set_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = read_reg(addr);
  write_reg(addr, temp | bit);
}
//Clear corresponding bit 
void clear_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = read_reg(addr);
  write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void set_mode(uint8_t mode)
{
  switch (mode)
  {
  case MCU2LCD_MODE:
    write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
    break;
  case CAM2LCD_MODE:
    write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
    break;
  case LCD2MCU_MODE:
    write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
    break;
  default:
    write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
    break;
  }
}

void OV5642_set_JPEG_size(byte address, uint8_t size)
{ 
  switch (size)
  {
  case OV5642_320x240:
    wrSensorRegs16_8(address, ov5642_320x240);
    break;
  case OV5642_640x480:
    wrSensorRegs16_8(address, ov5642_640x480);
    break;
  case OV5642_1024x768:
    wrSensorRegs16_8(address, ov5642_1024x768);
    break;
  case OV5642_1280x960:
    wrSensorRegs16_8(address, ov5642_1280x960);
    break;
  case OV5642_1600x1200:
    wrSensorRegs16_8(address, ov5642_1600x1200);
    break;
  case OV5642_2048x1536:
    wrSensorRegs16_8(address, ov5642_2048x1536);
    break;
  case OV5642_2592x1944:
    wrSensorRegs16_8(address, ov5642_2592x1944);
    break;
  default:
    wrSensorRegs16_8(address, ov5642_320x240);
    break;
  }
}


byte wrSensorReg8_8(byte address, int regID, int regDat)
{
  delay_us(5);
  sccb_bus_start();
  if(sccb_bus_write_byte(address) == 0)
  {
    sccb_bus_stop();
    return 1;
  }
  delay_us(5);
  if(sccb_bus_write_byte(regID) == 0)
  {
    sccb_bus_stop();
    return 2;
  }
  delay_us(5);
  if(sccb_bus_write_byte(regDat)==0)
  {
    sccb_bus_stop();
    return 3;
  }
  sccb_bus_stop();
  return 0;
}


byte rdSensorReg8_8(byte address, uint8_t regID, uint8_t* regDat)
{
  delay_us(10);
  sccb_bus_start();
  if(sccb_bus_write_byte(address) == 0)
  {
    sccb_bus_stop();
    //goto start;
    return 1;
  }
  delay_us(10);
  if(sccb_bus_write_byte(regID)==0)//ID
  {
    sccb_bus_stop();
    //goto start;
    return 2;
  }
  sccb_bus_stop();
  delay_us(10);
  sccb_bus_start();
  if(sccb_bus_write_byte(address|0x01)==0)
  {
    sccb_bus_stop();
    //goto start;
    return 3;
  }
  delay_us(10);
  *regDat = sccb_bus_read_byte();
  sccb_bus_send_noack();
  sccb_bus_stop();
  return 0;
}

//I2C Array Write 8bit address, 8bit data
int wrSensorRegs8_8(byte address, const struct sensor_reg reglist[])
{
  int err = 0;
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = next->reg;
    reg_val = next->val;
    err = wrSensorReg8_8(address, reg_addr, reg_val);
    //   delay_us(400);
    next++;
  }

  return err;
}

byte wrSensorReg16_8(byte address, int regID, int regDat)
{
  sccb_bus_start();
  if(0==sccb_bus_write_byte(address))
  {
    sccb_bus_stop();
    return(0);
  }
  delay_us(5);
  if(0==sccb_bus_write_byte(regID>>8))
  {
    sccb_bus_stop();
    return(0);
  }
  delay_us(5);
  if(0==sccb_bus_write_byte(regID))
  {
    sccb_bus_stop();
    return(0);
  }
  delay_us(5);
  if(0==sccb_bus_write_byte(regDat))
  {
    sccb_bus_stop();
    return(0);
  }
  sccb_bus_stop();

  return(1);
}

int wrSensorRegs16_8(byte address, const struct sensor_reg reglist[])
{
  int err = 0;

  unsigned int reg_addr;
  unsigned char reg_val;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xffff) | (reg_val != 0xff))
  {
    reg_addr =next->reg;
    reg_val = next->val;
    err = wrSensorReg16_8(address, reg_addr, reg_val);
    delay_us(600);
    next++;
  }
  return err;
}


byte rdSensorReg16_8(byte address, uint16_t regID, uint8_t* regDat)
{
  sccb_bus_start();
  if(0==sccb_bus_write_byte(address))
  {
    sccb_bus_stop();
    return(0);
  }
  delay_us(20);
  delay_us(20);
  if(0==sccb_bus_write_byte(regID>>8))
  {
    sccb_bus_stop();
    return(0);
  }
  delay_us(20);
  if(0==sccb_bus_write_byte(regID))
  {
    sccb_bus_stop();
    return(0);
  }
  delay_us(20);
  sccb_bus_stop();

  delay_us(20);


  sccb_bus_start();
  if(0==sccb_bus_write_byte(address+1))
  {
    sccb_bus_stop();
    return(0);
  }
  delay_us(20);
  *regDat=sccb_bus_read_byte();
  sccb_bus_send_noack();
  sccb_bus_stop();
  return(1);
}



