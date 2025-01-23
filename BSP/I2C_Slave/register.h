/*
 * register.h
 *
 *  Created on: Oct 21, 2024
 *      Author: CAO HIEU
 */

#ifndef SRC_REGISTER_H_
#define SRC_REGISTER_H_
#include <stdint.h>

typedef enum
{
  Status_OK       = 0x00U,
  Status_ERROR    = 0x01U,
  Status_BUSY     = 0x02U,
  Status_TIMEOUT  = 0x03U
} Peripheral_StatusTypeDef;

typedef enum {
    I2C_Status_OK       = 0x00U,
    I2C_Status_BUSY     = 0x01U,
    I2C_Status_ERROR    = 0x02U
} I2C_Slave_StatusTypeDef;

extern volatile I2C_Slave_StatusTypeDef I2C_Slave_Status;

//Register Addresses
#define REG_VERSION_ADDR		0x00

#define PASSIVE_REGISTERS      16

typedef enum reg_mode_t {
    RESERVED = 0,
    WRITE_ONLY,
    READ_ONLY,
    FULL_ACCESS
} reg_access_t;


typedef struct {
    uint8_t        reg_addr;
    reg_access_t   access;
    uint8_t        value;
} reg_t;

extern reg_t g_registers[PASSIVE_REGISTERS + 1];

void register_init(void);

int reg_get_index(uint8_t address);

void reg_factory(void);

Peripheral_StatusTypeDef reg_read(uint8_t reg_address, uint8_t *value);

Peripheral_StatusTypeDef reg_write(uint8_t reg_address, uint8_t value);

#endif /* SRC_REGISTER_H_ */
