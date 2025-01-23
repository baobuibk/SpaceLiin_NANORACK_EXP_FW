/*
 * register.c
 *
 *  Created on: Oct 21, 2024
 *      Author: CAO HIEU
 */

#include "register.h"
#include "stdio.h"


volatile I2C_Slave_StatusTypeDef I2C_Slave_Status = I2C_Status_OK;

reg_t g_registers[PASSIVE_REGISTERS + 1];

void register_init(void)
{
    g_registers[0].reg_addr = REG_VERSION_ADDR;
    g_registers[0].access = READ_ONLY;
    g_registers[0].value = 0x12;

    for (uint8_t i = 1; i <= PASSIVE_REGISTERS; i++)
    {
        g_registers[i].reg_addr = i;
        g_registers[i].access = FULL_ACCESS;
        g_registers[i].value = i;
    }
}

int reg_get_index(uint8_t address)
{
    if (address == REG_VERSION_ADDR)
    {
        return 0;
    }
    if (address >= 0x01 && address <= PASSIVE_REGISTERS)
    {
        return address;
    }
    if (address >= 0x11 && address <= 0xFF)
    {
        return -2;
    }
    return -1;
}

Peripheral_StatusTypeDef reg_write(uint8_t reg_address, uint8_t value)
{
//    if (I2C_Slave_Status == I2C_Status_BUSY) {
//        return Status_BUSY;
//    }

    int idx = reg_get_index(reg_address);
    if (idx >= 0)
    {
        if (g_registers[idx].access == FULL_ACCESS || g_registers[idx].access == WRITE_ONLY)
        {
            g_registers[idx].value = value;
            return Status_OK;
        }
    }
    return Status_ERROR;
}

Peripheral_StatusTypeDef reg_read(uint8_t reg_address, uint8_t *value)
{
    if (I2C_Slave_Status == I2C_Status_BUSY) {
        return Status_BUSY;
    }

    int idx = reg_get_index(reg_address);
    if (idx >= 0)
    {
    	if (g_registers[idx].access != RESERVED)
    	{
			*value = g_registers[idx].value;
			return Status_OK;
    	}
    }
    return Status_ERROR;
}



