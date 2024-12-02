/*
 * i2c.c
 *
 *  Created on: Nov 29, 2024
 *      Author: SANG HUYNH
 */


#include "i2c.h"
#include "stm32f4xx_ll_utils.h"

/**
 * @brief  Sends data via I2C using the LL library.
 * @param  I2Cx: Pointer to the I2C instance (e.g., I2C1, I2C2, ...)
 * @param  addr: Address of the slave device (7-bit, no left shift required)
 * @param  reg: Address of the register of slave device
 * @param  pData: Pointer to the data to be sent
 * @param  size: Number of bytes to send
 * @param  timeout: Timeout duration (ms)
 * @retval 0: Success, -1: Error
 */
uint8_t I2C_Write(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t *pData, uint8_t size, uint32_t timeout)
{
  // 1. Check busy status
  while ((LL_I2C_IsActiveFlag_BUSY(I2Cx)))
  {
//    if (++count > timeout)
//      return -1;
  }

  // 1.1 Set I2C normal mode
  LL_I2C_DisableBitPOS(I2Cx);

  // 2. Send START condition
  LL_I2C_GenerateStartCondition(I2Cx);

  // 3. Waiting START sent successfully
  while (!(LL_I2C_IsActiveFlag_SB(I2Cx)))
  {
//    if (++count > timeout)
//      return -1;
  }

  // 4. Send slave address with Write bit(0)
  LL_I2C_TransmitData8(I2Cx, addr << 1);

  // 5. Waiting ACK from slave
  while (!(LL_I2C_IsActiveFlag_ADDR(I2Cx)))
  {
//    if (++count > timeout)
//      return -1;
  }

  // 6. Clear flag address
  LL_I2C_ClearFlag_ADDR(I2Cx);

  // 7. Send register address
  LL_I2C_TransmitData8(I2Cx, reg);

  // 8. Send data to register
  for (uint8_t i = 0; i < size; ++i)
  {
	// 8.1. Waiting TX buffer empty
	while (!(LL_I2C_IsActiveFlag_TXE(I2Cx)))
	{
//		if (++count > timeout)
//			return -1;
	}
	// 8.2. Send data
	LL_I2C_TransmitData8(I2Cx, pData[i]);
	// 8.3. Waiting BTF (Byte Transfer Finished)
	while (!LL_I2C_IsActiveFlag_BTF(I2Cx))
	{
//		if (++count > timeout)
//			return -1;
	}
  }
  // 9. Send STOP condition
  LL_I2C_GenerateStopCondition(I2Cx);
  return 0;
}

/**
 * @brief  Sends data via I2C using the LL library.
 * @param  I2Cx: Pointer to the I2C instance (e.g., I2C1, I2C2, ...)
 * @param  addr: Address of the slave device (7-bit, no left shift required)
 * @param  reg: Address of the register of slave device
 * @param  pData: Pointer to the data to receive
 * @param  size: Number of bytes to send
 * @param  timeout: Timeout duration (ms)
 * @retval 0: Success, -1: Error
 */
uint8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t *pData, uint8_t size, uint32_t timeout)
{
	// 1. Check busy status
	while ((LL_I2C_IsActiveFlag_BUSY(I2Cx)))
	{
//		if (++count > timeout)
//			return -1;
	}

	// 2. Send START condition
	LL_I2C_GenerateStartCondition(I2Cx);

	// 3. Waiting START sent successfully

	while (!(LL_I2C_IsActiveFlag_SB(I2Cx)))
	{
//		if (++count > timeout)
//			return -1;
	}

	// 4. Send slave address with Write bit(0)
	LL_I2C_TransmitData8(I2Cx, addr << 1);

	// 5. Waiting ACK from slave
	while (!(LL_I2C_IsActiveFlag_ADDR(I2Cx)))
	{
//		if (++count > timeout)
//			return -1;
	}

	// 6. Clear flag address
	LL_I2C_ClearFlag_ADDR(I2Cx);

	// 7. Send register address
	LL_I2C_TransmitData8(I2Cx, reg);

	// 8. Send START condition
	LL_I2C_GenerateStartCondition(I2Cx);

	// 9. Send slave address with Read bit(1)
	LL_I2C_TransmitData8(I2Cx, (addr << 1)|1);

	// 10. Waiting ACK from slave
	while (!(LL_I2C_IsActiveFlag_ADDR(I2Cx)))
	{
//		if (++count > timeout)
//			return -1;
	}

	// 11. Clear flag address
	LL_I2C_ClearFlag_ADDR(I2Cx);

	// 12. Read data
	if (size == 1)
		LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
	for (uint8_t i = 0; i < size; ++i)
	{
		// 12.1. Checking end bytes
		if (i == size-1)
			LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
		// 12.2. Waiting RX buffer not empty
		while (!LL_I2C_IsActiveFlag_RXNE(I2Cx))
		{
//			if ((GetTick() - tickStart) >= Timeout) return 1;
//			return -1;
		}
		// 12.3. Read data from DR
		*pData[i] = LL_I2C_ReceiveData8(I2Cx);
	}

	// 13. Send STOP condition
	LL_I2C_GenerateStopCondition(I2Cx);
}
