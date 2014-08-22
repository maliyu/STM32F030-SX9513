/******************************************************************************
 * Project        : STM32F030+SX9513
 * File           : sx9513.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0.0 - Aug 2014
    > Initial revision

******************************************************************************/
#include "stm32f0xx.h"
#include "stm32f0308_discovery.h"

#define SX9500_ADDR          ((uint16_t)(0x2B))

static uint8_t IrqSrc;
static uint8_t PreTouchStatus;
static uint8_t TouchStatus;
static uint8_t CapSenseUsefulDataMsb;
static uint8_t CapSenseUsefulDataLsb;
static uint8_t CapSenseAverageDataMsb;
static uint8_t CapSenseAverageDataLsb;
static uint8_t CapSenseDiffDataMsb;
static uint8_t CapSenseDiffDataLsb;
static uint8_t CapSenseCompMsb;
static uint8_t CapSenseCompLsb;

/**
  * @brief  This function reads data from a register of the SX9500 EVK.
  * @param  reg: register to be read
  * @retval 8-bit register contents
  */
uint8_t SX9500_RegRead(uint8_t reg)
{
  uint8_t tmp = 0xFF;
  
//I2C1->CR2 = (uint32_t)(LIS302DL_ADDR) | (uint32_t)(1 << 16) | I2C_CR2_START ;
  I2C_TransferHandling(I2C1, SX9500_ADDR, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
		
//I2C1->TXDR = (uint8_t)(addr);
  I2C_SendData(I2C1, reg);
  while(!(I2C1->ISR & I2C_ISR_TC)){;}
    
//I2C1->CR2 = (uint32_t)(LIS302DL_ADDR) | (uint32_t)(1 << 16) | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START;
  I2C_TransferHandling(I2C1, SX9500_ADDR, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  while(!(I2C1->ISR & I2C_ISR_RXNE)){;}
    
//tmp = (uint8_t)I2C1->RXDR;
  tmp = I2C_ReceiveData(I2C1);
    
  while(!(I2C1->ISR & I2C_ISR_STOPF)){;}
  I2C1->ICR = I2C_ICR_STOPCF;
  
  return(tmp);
}

/**
  * @brief  This function writes data to a register of the SX9500 EVK.
  * @param  reg: register to be written
  *         data: data to be written to the selected register
  * @retval None
  */
void SX9500_RegWrite(uint8_t reg, uint8_t data)
{
//I2C1->CR2 |= (uint32_t)(LIS302DL_ADDR) | (uint32_t)(1 << 16) | I2C_CR2_RELOAD | I2C_CR2_START ;
  I2C_TransferHandling(I2C1, SX9500_ADDR, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
  while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
  
//I2C1->TXDR = (uint8_t)(addr);
  I2C_SendData(I2C1, reg);
  while(!(I2C1->ISR & I2C_ISR_TCR)){;}
    
//I2C1->CR2 |= I2C_CR2_AUTOEND;
  I2C_AutoEndCmd(I2C1, ENABLE);
  while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
    
//I2C1->TXDR = data;
  I2C_SendData(I2C1, data);
  
//I2C1->CR2 &= ~I2C_CR2_RELOAD;
  I2C_ReloadCmd(I2C1, DISABLE);
    
  while(!(I2C1->ISR & I2C_ISR_STOPF)){;}
  I2C1->ICR = I2C_ICR_STOPCF;
}

void SX9513_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
	uint8_t tmp[2] = {0xDE, 0x00};
  
  RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

  //(#) Enable peripheral clock using RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2Cx, ENABLE)
  //    function for I2C1 or I2C2.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  
  //(#) Enable SDA, SCL  and SMBA (when used) GPIO clocks using 
  //    RCC_AHBPeriphClockCmd() function. 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
	// Enable PA0 and make it as interrupt input pin
  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	/* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  // Enable PB8 and make it active high
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIOB->BSRR = GPIO_BSRR_BS_8;
  
  //(#) Peripherals alternate function: 
  //    (++) Connect the pin to the desired peripherals' Alternate 
  //         Function (AF) using GPIO_PinAFConfig() function.
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);

  //    (++) Configure the desired pin in alternate function by:
  //         GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

  //    (++) Select the type, OpenDrain and speed via  
  //         GPIO_PuPd, GPIO_OType and GPIO_Speed members
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  //    (++) Call GPIO_Init() function.
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //(#) Program the Mode, Timing , Own address, Ack and Acknowledged Address 
  //    using the I2C_Init() function.
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Timing = 0x10420F13;
  I2C_Init(I2C1, &I2C_InitStructure);
  
  //(#) Optionally you can enable/configure the following parameters without
  //    re-initialization (i.e there is no need to call again I2C_Init() function):
  //    (++) Enable the acknowledge feature using I2C_AcknowledgeConfig() function.
  //    (++) Enable the dual addressing mode using I2C_DualAddressCmd() function.
  //    (++) Enable the general call using the I2C_GeneralCallCmd() function.
  //    (++) Enable the clock stretching using I2C_StretchClockCmd() function.
  //    (++) Enable the PEC Calculation using I2C_CalculatePEC() function.
  //    (++) For SMBus Mode:
  //         (+++) Enable the SMBusAlert pin using I2C_SMBusAlertCmd() function.

  //(#) Enable the NVIC and the corresponding interrupt using the function
  //    I2C_ITConfig() if you need to use interrupt mode.
	//I2C_ITConfig(I2C1, (I2C_IT_RXI | I2C_IT_TXI | I2C_IT_TCI | I2C_IT_ADDRI | I2C_IT_NACKI | I2C_IT_STOPI), ENABLE);
  
  //(#) When using the DMA mode 
  //   (++) Configure the DMA using DMA_Init() function.
  //   (++) Active the needed channel Request using I2C_DMACmd() function.
  
  //(#) Enable the I2C using the I2C_Cmd() function.
  I2C_Cmd(I2C1, ENABLE);
	
	//GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	
	SX9500_RegRead(0x00);
	SX9500_RegWrite(0x07,0x00);
	SX9500_RegWrite(0x09, 0xE4);
	SX9500_RegWrite(0x1F, 0x83);
	SX9500_RegWrite(0x20, 0x83);
	SX9500_RegWrite(0x21, 0x83);
	SX9500_RegWrite(0x22, 0x83);
	SX9500_RegWrite(0x23, 0x83);
	SX9500_RegWrite(0x24, 0x83);
	SX9500_RegWrite(0x25, 0x83);
	SX9500_RegWrite(0x26, 0x83);
	SX9500_RegWrite(0x27, 0x43);
	SX9500_RegWrite(0x28, 0x60);
	SX9500_RegWrite(0x29, 0x60);
	SX9500_RegWrite(0x2A, 0x60);
	SX9500_RegWrite(0x2B, 0x60);
	SX9500_RegWrite(0x2C, 0x60);
	SX9500_RegWrite(0x2D, 0x60);
	SX9500_RegWrite(0x2E, 0x60);
	SX9500_RegWrite(0x2F, 0x60);
	SX9500_RegWrite(0x30, 0x10);
	SX9500_RegWrite(0x31, 0x14);
	SX9500_RegWrite(0x33, 0x0F);
	SX9500_RegWrite(0x34, 0x40);
	SX9500_RegWrite(0x35, 0x40);
	SX9500_RegWrite(0x36, 0x1D);
	SX9500_RegWrite(0x37, 0x1A);
	SX9500_RegWrite(0x38, 0x00);
	SX9500_RegWrite(0x3B, 0x00);
	SX9500_RegWrite(0x3E, 0xFF);
	
	SX9500_RegWrite(0x1E, 0x0F);
	
	//GPIO_SetBits(GPIOB, GPIO_Pin_8);
}

void SX9513_IrqSrc(void)
{
	IrqSrc = SX9500_RegRead(0x00);
}

void SX9513_ReadBL0(void)
{
	// Set channel as BL0
	SX9500_RegWrite(0x63, 0x00);
	CapSenseUsefulDataMsb = SX9500_RegRead(0x64);
	CapSenseUsefulDataLsb = SX9500_RegRead(0x65);
	CapSenseAverageDataMsb = SX9500_RegRead(0x66);
	CapSenseAverageDataLsb = SX9500_RegRead(0x67);
	CapSenseDiffDataMsb = SX9500_RegRead(0x68);
	CapSenseDiffDataLsb = SX9500_RegRead(0x69);
	CapSenseCompMsb = SX9500_RegRead(0x6A);
	CapSenseCompLsb = SX9500_RegRead(0x6B);
}

void SX9513_HandleBL0(void)
{
	IrqSrc = SX9500_RegRead(0x00);
	TouchStatus = SX9500_RegRead(0x01);
	
	if((IrqSrc & 0x40) == 0x40)
	{
		// detect touch event
		if((TouchStatus & 0x01) == 0x01)
		{
			// BL0
			STM_EVAL_LEDToggle(LED3);
		}
	}
	else if((IrqSrc & 0x20) == 0x20)
	{
		// detect release event
		if(((TouchStatus & 0x01) == 0x00) && ((PreTouchStatus & 0x01) == 0x01) )
		{
			// BL0
			STM_EVAL_LEDToggle(LED3);
		}
	}
	
	PreTouchStatus = TouchStatus;
}
