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
								{"help", Cmd_help,": Display list of help commands | format: help"},
								{"on_tec_pw", Cmd_on_tec_pw,": test | format: on_tec_pw"},
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
								{"get_all", Cmd_get_all, ":Display all | format: get_all"},
								{0,0,0}
								};

// static	char s_commandBuffer[COMMAND_MAX_LENGTH];
// static uint8_t	s_commandBufferIndex = 0;

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
	if(CMD_line.buffer_size != 0)
	{
		memset((void *)CMD_line.p_buffer, 0, sizeof(CMD_line.p_buffer));
	}
	//command_send_splash();
	UART_Send_String(&CONSOLE_UART, "EXP FIRMWARE V1.0.0\n");
	UART_Send_String(&CONSOLE_UART, "> ");
}

void process_command(USART_TypeDef* USARTx, char rxData);

#define MAX_HISTORY 5
#define MAX_CMD_LENGTH COMMAND_MAX_LENGTH

//static char s_commandHistory[MAX_HISTORY][MAX_CMD_LENGTH];
//static int s_historyCount = 0;
//static int s_historyIndex = 0;

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

				cmd_return = CmdLineProcess(CMD_line.p_buffer, CONSOLE_UART_HANDLE);
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
				// SDKLFJSDFKS
				// > CMD too long!
				// >
				UART_Send_String(&CONSOLE_UART, "\n> CMD too long!\n> ");
				CMD_line.write_index    = 0;
			}
		}
	}
//    char rxData;
//    while (IsDataAvailable(UART4) || IsDataAvailable(USART2))
//    {
//        if (IsDataAvailable(UART4))
//        {
//            rxData = Uart_read(UART4);
//            Uart_write(UART4, rxData);
//            process_command(UART4, rxData);
//        }
//
//        if (IsDataAvailable(USART2))
//        {
//            rxData = Uart_read(USART2);
//            Uart_write(USART2, rxData);
//            process_command(USART2, rxData);
//        }
//    }

}

//void process_command(USART_TypeDef * USARTx, char rxData)
//{
//    int8_t ret_val;
//    if (rxData == 27)  // ASCII code for ESC key
//    {
////    	volatile MuxConfig *Mux_state = &MuxBehavior;
////    	auto_report_enabled = 0;
////        rs422_report_enable = 0;
////        rf_report_enable = 0;
////        gps_start_position_ena = 0;
////        iou_status_auto_report = 0;
////        uart_choose_usart2 = 0;
////        char data_temp[1] = {0};
////        Write_ram(data_temp, 4, 1);
////
////        set_receive_flag(CAM_INDEX, 1);
////        set_receive_flag(PDU_INDEX, 1);
////        set_receive_flag(PMU_INDEX, 1);
////        set_receive_flag(IOU_INDEX, 1);
////        Mux_state->status = BOARD_IDLE;
////        Uart_sendstring(USARTx, "\r\n> ");
////        Uart_sendstring(USARTx, "Stopped!");
////        Uart_sendstring(USARTx, "> ");
//        return;
//    }
//
//    if ((rxData == '\r') || (rxData == '\n'))
//    {
//        if (s_commandBufferIndex > 0)
//        {
//            s_commandBuffer[s_commandBufferIndex] = 0;
//            s_commandBufferIndex++;
//            ret_val = CmdLineProcess(s_commandBuffer,USARTx);
//            s_commandBufferIndex = 0;
//
//
//            if (s_historyCount < MAX_HISTORY) {
//                strcpy(s_commandHistory[s_historyCount], s_commandBuffer);
//                s_historyCount++;
//            } else {
//                for (int i = 0; i < MAX_HISTORY - 1; i++) {
//                    strcpy(s_commandHistory[i], s_commandHistory[i + 1]);
//                }
//                strcpy(s_commandHistory[MAX_HISTORY - 1], s_commandBuffer);
//            }
//            s_historyIndex = s_historyCount;
//
//
//
////            Uart_sendstring(USARTx, "\r\n> ");
////            Uart_sendstring(USARTx, ErrorCode[ret_val]);
////            Uart_sendstring(USARTx, "> ");
//        }
//        else
//        {
////            Uart_sendstring(USARTx, "\r\n> ");
//        }
//    }
//    else if ((rxData == 8) || (rxData == 127))
//    {
//        if (s_commandBufferIndex > 0)
//        {
//            s_commandBufferIndex--;
//        }
//    }
//    else if (rxData == '-') {  // '-' key for up arrow
//        if (s_historyIndex > 0) {
//            s_historyIndex--;
//            strcpy(s_commandBuffer, s_commandHistory[s_historyIndex]);
//            s_commandBufferIndex = strlen(s_commandBuffer);
//            Uart_sendstring(USARTx, "\r                                      \r> ");
//            Uart_sendstring(USARTx, s_commandBuffer);
//        }
//    }
//    else if (rxData == '=') {  // '=' key for down arrow
//        if (s_historyIndex < s_historyCount) {
//            s_historyIndex++;
//            if (s_historyIndex == s_historyCount) {
//                s_commandBuffer[0] = '\0';
//                s_commandBufferIndex = 0;
//            } else {
//                strcpy(s_commandBuffer, s_commandHistory[s_historyIndex]);
//                s_commandBufferIndex = strlen(s_commandBuffer);
//            }
//            Uart_sendstring(USARTx, "\r                                      \r> ");
//            Uart_sendstring(USARTx, s_commandBuffer);
//        }
//    }
//    else
//    {
//        s_commandBuffer[s_commandBufferIndex] = rxData;
//        s_commandBufferIndex++;
//        if (s_commandBufferIndex >= COMMAND_MAX_LENGTH)
//        {
//            s_commandBufferIndex = 0;
//        }
//    }
//}

int Cmd_help(int argc, char *argv[]) {
    tCmdLineEntry * pEntry;
//    USART_TypeDef * USARTx = (USART_TypeDef*)argv[argc-1];

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

int Cmd_on_tec_pw(int argc, char *argv[])
{
	LL_GPIO_SetOutputPin(EF_TEC_EN_GPIO_Port, EF_TEC_EN_Pin);
	return (CMDLINE_OK);
}

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

#include "i2c.h"

int Cmd_get_all(int argc, char *argv[])
{
	uint8_t data = 0x32;
	UART_Send_String(&CONSOLE_UART, "get all \n");
//	I2C_SendData(I2C2, 0x68, 0x01, &data, 1, 50000);
	return (CMDLINE_OK);
}

void	command_create_task(void)
{
	SCH_TASK_CreateTask(&s_CommandTaskContext.taskHandle, &s_CommandTaskContext.taskProperty);
}

void	command_send_splash(void)
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
            	CONSOLE_UART.p_RX_buffer[EXP_UART.RX_write_index] = CONSOLE_UART.RX_irq_char;
                ADVANCE_RX_WRITE_INDEX(&CONSOLE_UART);
            }
        }
    }
}
