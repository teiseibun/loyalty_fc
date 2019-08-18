#include "stm32f4xx_conf.h"
#include "i2c.h"

/* I2C Timeout exception */
int i2c_timeout;
I2C_Status i2c_status;
#define I2C_TIMED(x) i2c_timeout = 0xFFFF; i2c_status = I2C_SUCCESS; \
while(x) { if(i2c_timeout-- == 0) { i2c_status = I2C_TIMEOUT; goto i2c_restart;} }

void i2c1_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* Reset I2C1 to avoid the busy flag issue */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_OD,
		.GPIO_PuPd = GPIO_PuPd_UP	
	};
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	I2C_InitTypeDef I2C_InitStruct = {
		.I2C_ClockSpeed = 400000,
		.I2C_Mode = I2C_Mode_I2C,
		.I2C_DutyCycle = I2C_DutyCycle_2,
		.I2C_OwnAddress1 = 0x68 << 1,
		.I2C_Ack = I2C_Ack_Disable,
		.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit	
	};
	I2C_Init(I2C1, &I2C_InitStruct);

	I2C_Cmd(I2C1, ENABLE);
}

I2C_Status i2c_read(I2C_TypeDef* i2c_channel, uint8_t device_address, uint8_t register_address, uint8_t *data,
	int data_count)
{
	while(I2C_GetFlagStatus(i2c_channel, I2C_FLAG_BUSY));
    
	/* Send the I2C start condition */
	I2C_GenerateSTART(i2c_channel, ENABLE);
  
	/* Test on I2C EV5 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send the device address */
	I2C_Send7bitAddress(i2c_channel, device_address, I2C_Direction_Transmitter);

	/* Test on I2C EV6 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
	/* Clear the I2C EV6 by setting again the PE bit */
	I2C_Cmd(i2c_channel, ENABLE);

	/* Send the register_address */
	I2C_SendData(i2c_channel, register_address);  

	/* Test on I2C EV8 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
	/* Send the start condition a second time */  
	I2C_GenerateSTART(i2c_channel, ENABLE);
  
	/* Test on I2C EV5 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_MODE_SELECT));
  
	/* Send the device address */
	I2C_Send7bitAddress(i2c_channel, device_address, I2C_Direction_Receiver);
  
	/* Test on I2C EV6 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
	while(data_count) {
		if(data_count == 1) {
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(i2c_channel, DISABLE);
 
			/* Send STOP Condition */
			I2C_GenerateSTOP(i2c_channel, ENABLE);
		}

		/* Test on EV7 and clear it */
		I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_BYTE_RECEIVED));
		
		/* Read a byte from the register */
		*data = I2C_ReceiveData(i2c_channel);

		/* Point to the next location where the byte read will be saved */
		data++;

		/* Decrement the read bytes counter */
		data_count--;

		/* Wait to make sure that STOP control bit has been cleared */
		I2C_TIMED(i2c_channel->CR1 & I2C_CR1_STOP);
	}

	/* Restart the I2C */
	i2c_restart:
	I2C_AcknowledgeConfig(i2c_channel, ENABLE);

	return i2c_status;
}

I2C_Status i2c_write(I2C_TypeDef* i2c_channel, uint8_t device_address, uint8_t register_address, 
	uint8_t data)
{
	while(I2C_GetFlagStatus(i2c_channel, I2C_FLAG_BUSY));

	/* Send the I2C start condition */
	I2C_GenerateSTART(i2c_channel, ENABLE);
 	 
	/* Test on I2C EV5 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_MODE_SELECT)); 
  
	/* Send device address for write */
	I2C_Send7bitAddress(i2c_channel, device_address, I2C_Direction_Transmitter);
  
	/* Test on I2C EV6 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  

	/* Send the register address */    
	I2C_SendData(i2c_channel, register_address);  

	/* Test on I2C EV8 and clear it */
	I2C_TIMED(! I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the current byte */
	I2C_SendData(i2c_channel, data); 

	/* Test on I2C EV8 and clear it */
	I2C_TIMED(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the I2C stop condition */
	I2C_GenerateSTOP(i2c_channel, ENABLE);

	/* Wait to make sure that STOP control bit has been cleared */
	I2C_TIMED(i2c_channel->CR1 & I2C_CR1_STOP);

	/* Restart the I2C */
	i2c_restart:
	I2C_AcknowledgeConfig(i2c_channel, ENABLE);

	return i2c_status;
}
