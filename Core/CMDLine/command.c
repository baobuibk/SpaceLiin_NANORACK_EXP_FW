/*
 * command.c
 *
 *  Created on: Nov 21, 2024
 *      Author: SANG HUYNH
 */

#include "command.h"
#include "cmdline.h"
#include "scheduler.h"
#include "uart.h"
#include "main.h"
#include "board.h"
#include "lt8722.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#include <stdlib.h>
#include <stdio.h>
#include "temperature.h"
#include "bmp390.h"
#include "ntc.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct _Command_TaskContextTypedef_
{
	SCH_TASK_HANDLE               taskHandle;
	SCH_TaskPropertyTypedef       taskProperty;
} Command_TaskContextTypedef;

/* Private function ----------------------------------------------------------*/
static	void	command_task_update(void);

/* Private variable -----------------------------------------------------------*/
uart_stdio_typedef  EXP_UART;
char                g_EXP_UART_TX_buffer[64];
char                g_EXP_UART_RX_buffer[64];
cmd_line_typedef    CMD_line;
char                g_CMD_line_buffer[64];

const char * ErrorCode[6] = {"OK\r\n", "CMDLINE_BAD_CMD\r\n", "CMDLINE_TOO_MANY_ARGS\r\n",
"CMDLINE_TOO_FEW_ARGS\r\n", "CMDLINE_INVALID_ARG\r\n", "CMD_OK_BUT_PENDING...\r\n" };

tCmdLineEntry g_psCmdTable[] =  {
					/* Command support */
								{"help", Cmd_help,": Display list of help commands | format: help"},
					/* Command for power supply */
								{"tec_pw_on", Cmd_tec_pw_on,": test | format: tec_pw_on 0/1"},
								{"led_pw_on", Cmd_led_pw_on,": test | format: led_pw_on 0/1"},
								{"heater_pw_on", Cmd_heater_pw_on,": test | format: heater_pw_on 0/1"},
					/* Command for temperature */
								{"set_temp", Cmd_set_temp,": set_temp | format: set_temp 20 30 40 50"},
								{"get_temp_setpoint", Cmd_get_temp_setpoint,": get_temp_setpoint | format: get_temp_setpoint"},
								{"get_temp", Cmd_get_temp,": get_temp | format: get_temp"},

								{"set_tec_vol", Cmd_set_tec_vol,": set_tec_voltage | format: set_tec_voltage vol0 vol1 vol2 vol3"},
								{"get_tec_vol", Cmd_get_tec_vol,": get_tec_voltage | format: get_tec_vol"},
								{"set_heater_duty", Cmd_set_heater_duty,": set_heater_duty | format: set_heater_duty 0% 1% 2% 3%"},
								{"get_heater_duty", Cmd_get_heater_duty,": get_heater_duty | format: get_heater_duty"},

								{"temp_ctrl", Cmd_temp_ctrl,": tec_ctrl | format: tec_ctrl 1 1 0 0"},
								{"temp_auto_ctrl", Cmd_temp_auto_ctrl,": tec_ctrl_auto | format: temp_auto_ctrl 1 1 0 0"},
					/* Command for ir led */
								{"set_ir_duty", Cmd_set_ir_duty, " | format: set_ir_duty <duty>"},
								{"get_ir_duty", Cmd_get_ir_duty, " | format: get_ir_duty"},
					/* Command for i2c sensor */
								{"get_accel_gyro", Cmd_get_acceleration_gyroscope, " | format: get_accel_gyro"},
								{"get_press", Cmd_get_pressure, " | format: get_press"},
					/* Command for system */
								{"get_all", Cmd_get_all, ":Display all | format: get_all"},



								{"reset", Cmd_reset,": test | format: "},
								{"set_en_req", Cmd_set_en_req,": test | format: "},
								{"reset_en_req", Cmd_reset_en_req,": test | format: "},
								{"clear_status_reg", Cmd_clear_status_reg,": test | format: "},
								{"read", Cmd_read,": test | format: "},
								{"on_tec", Cmd_on_tec,": test | format: "},
								{"tec_set_vol", Cmd_tec_set_vol,": test | format: "},
								{"get_status", Cmd_get_status,": test | format: "},
								{"set_ov_clamp", Cmd_set_ov_clamp,": test | format: "},
								{"set_uv_clamp", Cmd_set_uv_clamp,": test | format: "},
								{0,0,0}
								};

static Command_TaskContextTypedef           s_CommandTaskContext =
{
	SCH_INVALID_TASK_HANDLE,                // Will be updated by Schedular
	{
		SCH_TASK_SYNC,                      // taskType;
		SCH_TASK_PRIO_0,                    // taskPriority;
		10,                                 // taskPeriodInMS;
		command_task_update,                // taskFunction;
		9
	}
};

void	command_init(void)
{
    UART_Init( &CONSOLE_UART, EXP_UART_HANDLE, EXP_UART_IRQ,
				g_EXP_UART_TX_buffer, g_EXP_UART_RX_buffer,
				sizeof(g_EXP_UART_TX_buffer), sizeof(g_EXP_UART_RX_buffer));
    CMD_line.p_buffer       = g_CMD_line_buffer;
	CMD_line.buffer_size    = 64;
	CMD_line.write_index 	= 0;
	if(CMD_line.buffer_size!= 0)
	{
		memset((void *)CMD_line.p_buffer, 0, sizeof(CMD_line.p_buffer));
	}
//	command_send_splash();
	UART_Send_String(&CONSOLE_UART, "EXP FIRMWARE V1.0.0\n");
	UART_Send_String(&CONSOLE_UART, "> ");
}

void process_command(USART_TypeDef* USARTx, char rxData);

#define MAX_HISTORY 5
#define MAX_CMD_LENGTH COMMAND_MAX_LENGTH

static void command_task_update(void)
{
	uint8_t cmd_return, time_out;
	for(time_out = 50; (!RX_BUFFER_EMPTY(&CONSOLE_UART)) && (time_out != 0); time_out--)
	{
		CMD_line.RX_char = UART_Get_Char(&CONSOLE_UART);

		if(((CMD_line.RX_char == 8) || (CMD_line.RX_char == 127)))
		{
			if (CMD_line.write_index == 0)
				break;
			CMD_line.write_index--;
			UART_Send_Char(&CONSOLE_UART, CMD_line.RX_char);
			break;
		}

		UART_Send_Char(&CONSOLE_UART, CMD_line.RX_char);

		if((CMD_line.RX_char == '\r') || (CMD_line.RX_char == '\n'))
		{
			if(CMD_line.write_index > 0)
			{
				// Add a NUL char at the end of the CMD
				CMD_line.p_buffer[CMD_line.write_index] = 0;
				CMD_line.write_index++;

				cmd_return = CmdLineProcess(CMD_line.p_buffer, EXP_UART_HANDLE);
				CMD_line.write_index    = 0;

				UART_Send_String(&CONSOLE_UART, "> ");
				UART_Printf(&CONSOLE_UART, ErrorCode[cmd_return]);
				UART_Send_String(&CONSOLE_UART, "> ");
			}
			else
			{
				UART_Send_String(&CONSOLE_UART, "> ");
			}
		}
		else
		{
			CMD_line.p_buffer[CMD_line.write_index] = CMD_line.RX_char;
			CMD_line.write_index++;

			if (CMD_line.write_index > CMD_line.buffer_size)
			{
				UART_Send_String(&CONSOLE_UART, "\n> CMD too long!\n> ");
				CMD_line.write_index    = 0;
			}
		}
	}
}


/* Command support */
int Cmd_help(int argc, char *argv[])
{
	tCmdLineEntry * pEntry;
	// USART_TypeDef * USARTx = (USART_TypeDef*)argv[argc-1];

	pEntry = &g_psCmdTable[0];
	// Enter a loop to read each entry from the command table.  The
	// end of the table has been reached when the command name is NULL.
	while (pEntry->pcCmd)
	{
		// Print the command name and the brief description.
		UART_Send_String(&CONSOLE_UART, pEntry->pcCmd);
		UART_Send_String(&CONSOLE_UART, pEntry->pcHelp);
		UART_Send_String(&CONSOLE_UART, "\n");
		// Advance to the next entry in the table.
		pEntry++;
	}
	return (CMDLINE_OK);
}

/* Command for power supply */
int Cmd_tec_pw_on(int argc, char *argv[])
{
	if (argc < 2) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 2) return CMDLINE_TOO_MANY_ARGS;
	if (atoi(argv[1]))
		LL_GPIO_SetOutputPin(EF_TEC_EN_GPIO_Port, EF_TEC_EN_Pin);
	else
		LL_GPIO_ResetOutputPin(EF_TEC_EN_GPIO_Port, EF_TEC_EN_Pin);
	return (CMDLINE_OK);
}
int Cmd_led_pw_on(int argc, char *argv[])
{
	if (argc < 2) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 2) return CMDLINE_TOO_MANY_ARGS;
	if (atoi(argv[1]))
		LL_GPIO_SetOutputPin(EF_LED_EN_GPIO_Port, EF_LED_EN_Pin);
	else
		LL_GPIO_ResetOutputPin(EF_LED_EN_GPIO_Port, EF_LED_EN_Pin);
	return (CMDLINE_OK);
}
int Cmd_heater_pw_on(int argc, char *argv[])
{
	if (argc < 2) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 2) return CMDLINE_TOO_MANY_ARGS;
	if (atoi(argv[1]))
		LL_GPIO_SetOutputPin(EF_HEATER_EN_GPIO_Port, EF_HEATER_EN_Pin);
	else
		LL_GPIO_ResetOutputPin(EF_HEATER_EN_GPIO_Port, EF_HEATER_EN_Pin);
	return (CMDLINE_OK);
}
/* Command for temperature */
int Cmd_set_temp(int argc, char *argv[])
{
	if (argc < 5) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 5) return CMDLINE_TOO_MANY_ARGS;
	int16_t setpoint_0 = atoi(argv[1]);
	int16_t setpoint_1 = atoi(argv[2]);
	int16_t setpoint_2 = atoi(argv[3]);
	int16_t setpoint_3 = atoi(argv[4]);
	temperature_set_setpoint(0, setpoint_0);
	temperature_set_setpoint(1, setpoint_1);
	temperature_set_setpoint(2, setpoint_2);
	temperature_set_setpoint(3, setpoint_3);
	UART_Printf(&CONSOLE_UART, "Setpoint[%d]:%i \n", 0, setpoint_0);
	UART_Printf(&CONSOLE_UART, "Setpoint[%d]:%i \n", 1, setpoint_1);
	UART_Printf(&CONSOLE_UART, "Setpoint[%d]:%i \n", 2, setpoint_2);
	UART_Printf(&CONSOLE_UART, "Setpoint[%d]:%i \n", 3, setpoint_3);
	return (CMDLINE_OK);
}
int Cmd_get_temp(int argc, char *argv[])
{
	if (argc > 1) return CMDLINE_TOO_MANY_ARGS;
	int16_t temp;

	/* Temperature from BMP390 */
	temp = bmp390_get_temperature();
	if (temp == 0x7FFF) UART_Send_String(&CONSOLE_UART, "BMP390 is fail \n");
	else UART_Printf(&CONSOLE_UART, "BMP390 temp: %i \n", temp);

	/* Temperature from NTC */
	for (uint8_t channel = 0; channel < 4; channel++)
	{
		temp = NTC_Temperature[channel];
		if (temp == 0x7FFF) UART_Printf(&CONSOLE_UART, "NTC[%d] is fail \n", channel);
		else UART_Printf(&CONSOLE_UART, "NTC[%d]: %i \n", channel, temp);
	}
	return (CMDLINE_OK);
}

int Cmd_get_temp_setpoint(int argc, char *argv[])
{
	if (argc > 1) return CMDLINE_TOO_MANY_ARGS;
	int16_t setpoint = 0;
	for (uint8_t channel = 0; channel < 4; channel++)
	{
		setpoint = temperature_get_setpoint(channel);
		UART_Printf(&CONSOLE_UART, "Setpoint[%d]:%i \r\n", channel, setpoint);
	}
	return (CMDLINE_OK);
}
int Cmd_set_tec_vol(int argc, char *argv[])
{
	if (argc < 5) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 5) return CMDLINE_TOO_MANY_ARGS;
	uint16_t vol[4];
	for (uint8_t i = 0; i < 4; i++)
	{
		vol[i] = atoi(argv[i+1]);
		if (vol[i] > 3000) vol[i] = 3000;
	}
	temperature_set_tec_vol(0, vol[0]);
	temperature_set_tec_vol(1, vol[1]);
	temperature_set_tec_vol(2, vol[2]);
	temperature_set_tec_vol(3, vol[3]);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 0, vol[0]);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 1, vol[1]);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 2, vol[2]);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 3, vol[3]);
	return (CMDLINE_OK);
}
int Cmd_get_tec_vol(int argc, char *argv[])
{
	if (argc > 1) return CMDLINE_TOO_MANY_ARGS;
	uint16_t vol_0 = temperature_get_tec_vol(0);
	uint16_t vol_1 = temperature_get_tec_vol(1);
	uint16_t vol_2 = temperature_get_tec_vol(2);
	uint16_t vol_3 = temperature_get_tec_vol(3);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 0, vol_0);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 1, vol_1);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 2, vol_2);
	UART_Printf(&CONSOLE_UART, "Tec voltage[%d]:%i mV\n", 3, vol_3);
	return (CMDLINE_OK);
}
int Cmd_set_heater_duty(int argc, char *argv[])
{
	if (argc < 5) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 5) return CMDLINE_TOO_MANY_ARGS;
	uint8_t duty[4];
	for (uint8_t i = 0; i < 4; i++)
	{
		duty[i] = atoi(argv[i+1]);
		if (duty[i] > 100) duty[i] = 100;
	}
	temperature_set_heater_duty(0, duty[0]);
	temperature_set_heater_duty(1, duty[1]);
	temperature_set_heater_duty(2, duty[2]);
	temperature_set_heater_duty(3, duty[3]);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 0, duty[0]);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 1, duty[1]);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 2, duty[2]);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 3, duty[3]);
	return (CMDLINE_OK);
}
int Cmd_get_heater_duty(int argc, char *argv[])
{
	if (argc > 1) return CMDLINE_TOO_MANY_ARGS;
	int16_t duty_0 = temperature_get_heater_duty(0);
	int16_t duty_1 = temperature_get_heater_duty(1);
	int16_t duty_2 = temperature_get_heater_duty(2);
	int16_t duty_3 = temperature_get_heater_duty(3);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 0, duty_0);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 1, duty_1);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 2, duty_2);
	UART_Printf(&CONSOLE_UART, "Heater duty[%d]:%i %\n", 3, duty_3);
	return (CMDLINE_OK);
}
int Cmd_temp_ctrl(int argc, char *argv[])
{
	if (argc < 5) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 5) return CMDLINE_TOO_MANY_ARGS;
	mode_ctrl_temp_t mode_0 = OFF;
	mode_ctrl_temp_t mode_1 = OFF;
	mode_ctrl_temp_t mode_2 = OFF;
	mode_ctrl_temp_t mode_3 = OFF;

	if (!strcmp(argv[1], "C")) mode_0 = COOL;
	else if (!strcmp(argv[1], "H")) mode_0 = HEAT;
	else mode_0 = OFF;
	if (!strcmp(argv[2], "C")) mode_1 = COOL;
	else if (!strcmp(argv[2], "H")) mode_1 = HEAT;
	else mode_1 = OFF;
	if (!strcmp(argv[3], "C")) mode_1 = COOL;
	else if (!strcmp(argv[3], "H")) mode_2 = HEAT;
	else mode_2 = OFF;
	if (!strcmp(argv[4], "C")) mode_3 = COOL;
	else if (!strcmp(argv[4], "H")) mode_3 = HEAT;
	else mode_3 = OFF;

	temperature_set_ctrl(mode_0, mode_1, mode_2, mode_3);
	return (CMDLINE_OK);
}
int Cmd_temp_auto_ctrl(int argc, char *argv[])
{
	if (argc < 5) return CMDLINE_TOO_FEW_ARGS;
	if (argc > 5) return CMDLINE_TOO_MANY_ARGS;
	uint8_t auto_0 = atoi(argv[1]) ? 1 : 0;
	uint8_t auto_1 = atoi(argv[2]) ? 1 : 0;
	uint8_t auto_2 = atoi(argv[3]) ? 1 : 0;
	uint8_t auto_3 = atoi(argv[4]) ? 1 : 0;
	temperature_set_auto_ctrl(auto_0, auto_1, auto_2, auto_3);
	return (CMDLINE_OK);
}
/* Command for ir led */
int Cmd_set_ir_duty(int argc, char *argv[])
{
	return (CMDLINE_OK);
}
int Cmd_get_ir_duty(int argc, char *argv[])
{
	return (CMDLINE_OK);
}
/* Command for i2c sensor */
int Cmd_get_acceleration_gyroscope(int argc, char *argv[])
{
	return (CMDLINE_OK);
}
int Cmd_get_pressure(int argc, char *argv[])
{
	return (CMDLINE_OK);
}
/* Command for system */
int Cmd_get_all(int argc, char *argv[])
{
	UART_Send_String(&CONSOLE_UART, "get all \n");
	return (CMDLINE_OK);
}















///////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Cmd_reset(int argc, char *argv[])
{
	lt8722_reset();
	return (CMDLINE_OK);
}

int Cmd_set_en_req(int argc, char *argv[])
{
	LL_GPIO_SetOutputPin(TEC_1_EN_GPIO_Port, TEC_1_EN_Pin);
	lt8722_reg_write(LT8722_SPIS_COMMAND, 0x00004000);
//	lt8722_set_enable_req(LT8722_ENABLE_REQ_ENABLED);
	return (CMDLINE_OK);
}

int Cmd_reset_en_req(int argc, char *argv[])
{
	lt8722_set_enable_req(LT8722_ENABLE_REQ_DISABLED);
	return (CMDLINE_OK);
}

int Cmd_clear_status_reg(int argc, char *argv[])
{
	lt8722_reg_write(LT8722_SPIS_STATUS, 0);
	return (CMDLINE_OK);
}

int Cmd_read(int argc, char *argv[])
{
	uint32_t data;
	lt8722_reg_read(LT8722_SPIS_COMMAND, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_COMMAND: 0x%X-%X \n", data>>16, data);

	lt8722_reg_read(LT8722_SPIS_STATUS, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_STATUS: 0x%X-%X \n", data>>16, data);

	lt8722_reg_read(LT8722_SPIS_DAC_ILIMN, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_DAC_ILIMN: 0x%X-%X \n", data>>16, data);

	lt8722_reg_read(LT8722_SPIS_DAC_ILIMP, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_DAC_ILIMP: 0x%X-%X \n", data>>16, data);

	lt8722_reg_read(LT8722_SPIS_DAC, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_DAC: 0x%X-%X \n", data>>16, data);

	lt8722_reg_read(LT8722_SPIS_OV_CLAMP, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_OV_CLAMP: 0x%X \n", data);

	lt8722_reg_read(LT8722_SPIS_UV_CLAMP, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_UV_CLAMP: 0x%X \n", data);

	lt8722_reg_read(LT8722_SPIS_AMUX, &data);
	UART_Printf(&CONSOLE_UART, "SPIS_AMUX: 0x%X \n", data);

	return (CMDLINE_OK);
}


int Cmd_on_tec(int argc, char *argv[])
{
	lt8722_init();
	return (CMDLINE_OK);
}

int Cmd_tec_set_vol(int argc, char *argv[])
{
//	if (argc < 2) return CMDLINE_TOO_FEW_ARGS;
//	if (argc > 2) return CMDLINE_TOO_MANY_ARGS;

	int64_t vol = atoi(argv[1]);
	UART_Printf(&CONSOLE_UART, "Tec set: %d mV \n", vol);
	vol *= 1000000;
	lt8722_set_output_voltage(vol);
	return (CMDLINE_OK);
}

int Cmd_get_status(int argc, char *argv[])
{
	uint16_t status;
	lt8722_get_status(&status);
	UART_Printf(&CONSOLE_UART, "status: 0x%X \n", status);
	return (CMDLINE_OK);
}

int Cmd_set_ov_clamp(int argc, char *argv[])
{
	uint8_t over_vol = atoi(argv[1]);
	UART_Printf(&CONSOLE_UART, "OV_CLAMP: %X \n", over_vol);
	lt8722_set_spis_ov_clamp(over_vol);
	return CMDLINE_OK;
}
int Cmd_set_uv_clamp(int argc, char *argv[])
{
	uint8_t uper_vol = atoi(argv[1]);
	UART_Printf(&CONSOLE_UART, "UV_CLAMP: %X \n", uper_vol);
	lt8722_set_spis_uv_clamp(uper_vol);
	return CMDLINE_OK;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////





void command_create_task(void)
{
	SCH_TASK_CreateTask(&s_CommandTaskContext.taskHandle, &s_CommandTaskContext.taskProperty);
}

void command_send_splash(void)
{
	UART_Send_String(&CONSOLE_UART, "-------------------------------------\n");
	UART_Send_String(&CONSOLE_UART, "-------- _______  ______  --------\n");
	UART_Send_String(&CONSOLE_UART, "--------| ____\\ \\/ /  _ \\ --------\n");
	UART_Send_String(&CONSOLE_UART, "--------|  _|  \\  /| |_) |--------\n");
	UART_Send_String(&CONSOLE_UART, "--------| |___ /  \\|  __/ --------\n");
	UART_Send_String(&CONSOLE_UART, "--------|_____/_/\\_\\_|    --------\n");
}


/* :::::::::: CONSOLE_UART_IRQHandler ::::::::::::: */
void CONSOLE_UART_IRQHandler(void)
{
    if(LL_USART_IsActiveFlag_TXE(CONSOLE_UART.handle) == true)
    {
        if(TX_BUFFER_EMPTY(&CONSOLE_UART))
        {
            // Buffer empty, so disable interrupts
            LL_USART_DisableIT_TXE(CONSOLE_UART.handle);
        }
        else
        {
            // There is more data in the output buffer. Send the next byte
            UART_Prime_Transmit(&CONSOLE_UART);
        }
    }

    if(LL_USART_IsActiveFlag_RXNE(CONSOLE_UART.handle) == true)
    {
    	CONSOLE_UART.RX_irq_char = LL_USART_ReceiveData8(CONSOLE_UART.handle);

        // NOTE: On win 10, default PUTTY when hit enter only send back '\r',
        // while on default HERCULES when hit enter send '\r\n' in that order.
        // The code bellow is modified so that it can work on PUTTY and HERCULES.
        if((!RX_BUFFER_FULL(&CONSOLE_UART)) && (CONSOLE_UART.RX_irq_char != '\n'))
        {
            if (CONSOLE_UART.RX_irq_char == '\r')
            {
            	CONSOLE_UART.p_RX_buffer[CONSOLE_UART.RX_write_index] = '\n';
                ADVANCE_RX_WRITE_INDEX(&CONSOLE_UART);
            }
            else
            {
            	CONSOLE_UART.p_RX_buffer[CONSOLE_UART.RX_write_index] = CONSOLE_UART.RX_irq_char;
                ADVANCE_RX_WRITE_INDEX(&CONSOLE_UART);
            }
        }
    }
}
