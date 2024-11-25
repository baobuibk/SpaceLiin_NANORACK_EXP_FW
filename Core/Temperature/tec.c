/*
 * tec.c
 *
 *  Created on: Nov 21, 2024
 *      Author: SANG HUYNH
 */

#include "tec.h"
#include "scheduler.h"
#include "command.h"
#include "main.h"
#include "stm32f4xx_ll_spi.h"
#include "delay.h"

#define SPI_TIMEOUT 5000

/* Private typedef -----------------------------------------------------------*/
typedef struct _Tec_TaskContextTypedef_
{
	SCH_TASK_HANDLE               taskHandle;
	SCH_TaskPropertyTypedef       taskProperty;
} Tec_TaskContextTypedef;

/* Private function -----------------------------------------------------------*/
static void tec_task_update(void);

static inline void csLOW(void){
	LL_GPIO_ResetOutputPin(TEC_1_CS_GPIO_Port, TEC_1_CS_Pin);
}

static inline void csHIGH(void){
	LL_GPIO_SetOutputPin(TEC_1_CS_GPIO_Port, TEC_1_CS_Pin);
}

static uint8_t SPI_LL_Transmit(uint8_t data){
//	uint32_t timeout = SPI_TIMEOUT;
	LL_SPI_TransmitData8(SPI1, data);

	while(LL_SPI_IsActiveFlag_BSY(SPI1));
//    while(LL_SPI_IsActiveFlag_BSY(SPI1) && timeout--);
//    if (timeout == 0) return 0;

	return LL_SPI_ReceiveData8(SPI1);
}

uint8_t SPI_write_and_read_buffer(uint8_t *buffer, uint8_t byte_number) {
    uint8_t received_data = 0;
    csLOW();
    for (uint8_t i = 0; i < byte_number; i++) {
        received_data = SPI_LL_Transmit(buffer[i]); // Gửi từng byte từ buffer
        buffer[i] = received_data;
    }
    csHIGH();
    return received_data; // Trả về dữ liệu nhận được từ byte cuối cùng
}


uint8_t Calculate_CRC8(uint8_t *data, uint8_t length);
void put_unaligned_be32(uint32_t val, uint8_t *buf);
void put_unaligned_be16(uint16_t val, uint8_t *buf);
uint32_t get_unaligned_be32(uint8_t *buf);
uint32_t get_unaligned_be16(uint8_t *buf);
uint32_t find_first_set_bit(uint32_t word);
uint32_t field_prep(uint32_t mask, uint32_t val);
uint32_t field_get(uint32_t mask, uint32_t word);
int lt8722_set_dac(uint32_t value);

/* Private variable -----------------------------------------------------------*/
static Tec_TaskContextTypedef           	s_TecTaskContext =
{
	SCH_INVALID_TASK_HANDLE,                // Will be updated by Schedular
	{
		SCH_TASK_SYNC,                      // taskType;
		SCH_TASK_PRIO_0,                    // taskPriority;
		10,                                 // taskPeriodInMS;
		tec_task_update,                // taskFunction;
		9
	}
};

struct lt8722_reg lt8722_regs[LT8722_NUM_REGISTERS] = {
	{
		LT8722_SPIS_COMMAND, LT8722_SPIS_COMMAND_DEFAULT_VALUE,
		LT8722_SPIS_COMMAND_SIZE
	},
	{
		LT8722_SPIS_STATUS, LT8722_SPIS_STATUS_DEFAULT_VALUE,
		LT8722_SPIS_STATUS_SIZE
	},
	{
		LT8722_SPIS_DAC_ILIMN, LT8722_SPIS_DAC_ILIMN_DEFAULT_VALUE,
		LT8722_SPIS_DAC_ILIMN_SIZE
	},
	{
		LT8722_SPIS_DAC_ILIMP, LT8722_SPIS_DAC_ILIMP_DEFAULT_VALUE,
		LT8722_SPIS_DAC_ILIMP_SIZE
	},
	{
		LT8722_SPIS_DAC, LT8722_SPIS_DAC_DEFAULT_VALUE,
		LT8722_SPIS_DAC_SIZE
	},
	{
		LT8722_SPIS_OV_CLAMP, LT8722_SPIS_OV_CLAMP_DEFAULT_VALUE,
		LT8722_SPIS_OV_CLAMP_SIZE
	},
	{
		LT8722_SPIS_UV_CLAMP, LT8722_SPIS_UV_CLAMP_DEFAULT_VALUE,
		LT8722_SPIS_UV_CLAMP_SIZE
	},
	{
		LT8722_SPIS_AMUX, LT8722_SPIS_AMUX_DEFAULT_VALUE,
		LT8722_SPIS_AMUX_SIZE
	},
};

int8_t tec_init(void)
{
	int8_t ret;
	uint32_t i;
	int32_t dac;
	int64_t voltage;
	int64_t start_voltage;
	int64_t end_voltage;

	LL_GPIO_ResetOutputPin(TEC_1_EN_GPIO_Port, TEC_1_EN_Pin);
	LL_GPIO_ResetOutputPin(TEC_1_SWEN_GPIO_Port, TEC_1_SWEN_Pin);
	// Reset LT8722
	lt8722_reset();
	/*
	 * Start-up sequence
	 * 1. Apply proper VIN and VDDIO voltages
	 *
	 * 2. Enable VCC LDO and other LT8722 circuitry
	 */
	ret = lt8722_clear_faults();
	LL_GPIO_SetOutputPin(TEC_1_EN_GPIO_Port, TEC_1_EN_Pin);
	ret = lt8722_set_enable_req(LT8722_ENABLE_REQ_ENABLED);
	// 3. Configure output voltage control DAC to 0xFF000000
	ret = lt8722_set_dac(0xFF000000);
	// 4. Write all SPIS_STATUS registers to 0
	ret = lt8722_reg_write(LT8722_SPIS_STATUS, 0);
	LL_mDelay(1);
	// 5. Ramp the output voltage control DAC from 0xFF000000 to 0x00000000
	start_voltage = lt8722_dac_to_voltage(0xFF000000);
	end_voltage = lt8722_dac_to_voltage(0x00000000);
	for (i = 0;  i < 5; i++)
	{
		voltage = (start_voltage + (end_voltage - start_voltage) * i / 4);
		dac = lt8722_voltage_to_dac(voltage);
		ret = lt8722_set_dac(dac);

		LL_mDelay(1);
	}

	// 6. Enable the PWM switching behavior
	LL_GPIO_SetOutputPin(TEC_1_SWEN_GPIO_Port, TEC_1_SWEN_Pin);
	ret = lt8722_set_swen_req(LT8722_SWEN_REQ_ENABLED);
	delay_us(160);


	// 7. Set the desired output voltage

	return ret;

}

void tec_read(void)
{
	return;
}

void tec_create_task(void)
{
	SCH_TASK_CreateTask(&s_TecTaskContext.taskHandle, &s_TecTaskContext.taskProperty);
}

static void tec_task_update(void)
{
	return;
}

/**
 * @brief Convert voltage to DAC code.
 * @param voltage - Voltage value in nanovolts.
 * @return DAC code.
 */
int32_t lt8722_voltage_to_dac(int64_t voltage)
{
	return (LT8722_DAC_OFFSET - voltage) * (1 << LT8722_DAC_RESOLUTION) /
	       LT8722_DAC_VREF;
}

/**
 * @brief Convert DAC code to nanovolts.
 * @param dac - DAC code.
 * @return Voltage value in nanovolts.
 */
int64_t lt8722_dac_to_voltage(int32_t dac)
{
	return LT8722_DAC_OFFSET - dac * LT8722_DAC_VREF /
	       (1 << LT8722_DAC_RESOLUTION);
}

int lt8722_transaction(struct lt8722_packet *packet)
{
	uint8_t buffer[8] = {0};

	buffer[0] = packet->command.byte;
	buffer[1] = packet->reg.address << 1;

	if (packet->command.byte == LT8722_DATA_WRITE_COMMAND)
	{
		put_unaligned_be32(packet->data, &buffer[2]);
		buffer[6] = Calculate_CRC8(buffer, 6);
	} else
		buffer[2] = Calculate_CRC8(buffer, 2);

	SPI_write_and_read_buffer(buffer, packet->command.size);

	packet->status = (get_unaligned_be16(&buffer[0]) & GENMASK(10, 0));

	if (packet->command.byte == LT8722_DATA_WRITE_COMMAND)
	{
		packet->crc = buffer[2];
		packet->ack = buffer[7];
	} else if (packet->command.byte == LT8722_DATA_READ_COMMAND) {
		packet->data = get_unaligned_be32(&buffer[2]);
		packet->crc = buffer[6];
		packet->ack = buffer[7];
	} else {
		packet->crc = buffer[2];
		packet->ack = buffer[3];
	}

	if (packet->ack != LT8722_ACK_ACKNOWLEDGE)
		return -1;

	return 0;
}

int lt8722_reg_read(uint8_t address, uint32_t *data)
{
	struct lt8722_packet packet;
	struct lt8722_command command = {
		LT8722_DATA_READ_COMMAND,
		LT8722_DATA_READ_COMMAND_SIZE
	};

	packet.command = command;
	packet.reg = lt8722_regs[address];

	lt8722_transaction(&packet);

	*data = packet.data;

	return 0;
}


int lt8722_reg_write(uint8_t address, uint32_t data)
{
	struct lt8722_packet packet;
	struct lt8722_command command = {
		LT8722_DATA_WRITE_COMMAND,
		LT8722_DATA_WRITE_COMMAND_SIZE
	};

	packet.command = command;
	packet.reg = lt8722_regs[address];
	packet.data = data;

	return lt8722_transaction(&packet);
}

int lt8722_reg_write_mask(uint8_t address, uint32_t mask, uint32_t data)
{
	uint32_t reg_data;

	lt8722_reg_read(address, &reg_data);

	reg_data &= ~mask;
	reg_data |= field_prep(mask, data);

	return lt8722_reg_write(address, reg_data);
}

int lt8722_get_status(uint16_t *status)
{
	int8_t ret;
	struct lt8722_packet packet;
	struct lt8722_command command = {
		LT8722_STATUS_ACQUISITION_COMMAND,
		LT8722_STATUS_ACQUISITION_COMMAND_SIZE
	};

	packet.command = command;
	packet.reg = lt8722_regs[LT8722_SPIS_STATUS];

	ret = lt8722_transaction(&packet);
	if (ret)
		return ret;

	*status = packet.status;

	return 0;
}

int lt8722_reset(void)
{
	return lt8722_reg_write_mask(LT8722_SPIS_COMMAND,
			LT8722_SPI_RST_MASK, LT8722_SPI_RST_RESET);
}

int lt8722_clear_faults(void)
{
	int ret;
	uint16_t status;

	ret = lt8722_get_status(&status);

	return lt8722_reg_write_mask(LT8722_SPIS_STATUS,
				     LT8722_FAULTS_MASK, 0);
}

int lt8722_set_enable_req(bool value)
{
	return lt8722_reg_write_mask(LT8722_SPIS_COMMAND,
				     LT8722_ENABLE_REQ_MASK, value);
}

int lt8722_get_enable_req(bool *value)
{
	int ret;
	uint32_t data;

	ret = lt8722_reg_read(LT8722_SPIS_COMMAND, &data);
	if (ret)
		return ret;

	*value = field_get(LT8722_ENABLE_REQ_MASK, data);

	return 0;
}

int lt8722_set_swen_req(bool value)
{
	return lt8722_reg_write_mask(LT8722_SPIS_COMMAND,
				     LT8722_SWEN_REQ_MASK, value);
}

uint8_t Calculate_CRC8(uint8_t *data, uint8_t length) {
    uint8_t crc = 0x00;
    uint8_t poly = 0x07;
    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void put_unaligned_be32(uint32_t val, uint8_t *buf)
{
	buf[3] = val & 0xFF;
	buf[2] = (val >> 8) & 0xFF;
	buf[1] = (val >> 16) & 0xFF;
	buf[0] = val >> 24;
}

void put_unaligned_be16(uint16_t val, uint8_t *buf)
{
	buf[1] = val & 0xFF;
	buf[0] = val >> 8;
}

uint32_t get_unaligned_be32(uint8_t *buf)
{
	return buf[3] | ((uint16_t)buf[2] << 8) | ((uint32_t)buf[1] << 16)
	       | ((uint32_t)buf[0] << 24);
}

uint32_t get_unaligned_be16(uint8_t *buf)
{
	return buf[1] | ((uint16_t)buf[0] << 8);
}

uint32_t find_first_set_bit(uint32_t word)
{
	uint32_t first_set_bit = 0;

	while (word) {
		if (word & 0x1)
			return first_set_bit;
		word >>= 1;
		first_set_bit ++;
	}

	return 32;
}

uint32_t field_prep(uint32_t mask, uint32_t val)
{
	return (val << find_first_set_bit(mask)) & mask;
}

uint32_t field_get(uint32_t mask, uint32_t word)
{
	return (word & mask) >> find_first_set_bit(mask);
}

int lt8722_set_dac(uint32_t value)
{
	return lt8722_reg_write_mask(LT8722_SPIS_DAC,
				     LT8722_SPIS_DAC_MASK, value);
}
