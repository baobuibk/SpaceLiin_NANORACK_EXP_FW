/*
 * command.h
 *
 *  Created on: Nov 21, 2024
 *      Author: SANG HUYNH
 */

#ifndef CMDLINE_COMMAND_H_
#define CMDLINE_COMMAND_H_

#include "cmdline.h"
#include "uart.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define	COMMAND_MAX_LENGTH		255
#define CONSOLE_UART			EXP_UART
#define CONSOLE_UART_HANDLE		EXP_UART_HANDLE
typedef struct _cmd_line_typedef
{
                uint16_t    buffer_size;
                char*       p_buffer;
    volatile    uint16_t    write_index;
    volatile    char        RX_char;
}cmd_line_typedef;

extern uart_stdio_typedef  EXP_UART;

void command_init(void);
void command_create_task(void);
void command_send_splash(void);

int Cmd_help(int argc, char *argv[]);
int Cmd_on_tec_pw(int argc, char *argv[]);
int Cmd_reset(int argc, char *argv[]);
int Cmd_set_en_req(int argc, char *argv[]);
int Cmd_reset_en_req(int argc, char *argv[]);
int Cmd_clear_status_reg(int argc, char *argv[]);
int Cmd_read(int argc, char *argv[]);
int Cmd_on_tec(int argc, char *argv[]);
int Cmd_tec_set_vol(int argc, char *argv[]);
int Cmd_get_status(int argc, char *argv[]);
int Cmd_set_ov_clamp(int argc, char *argv[]);
int Cmd_set_uv_clamp(int argc, char *argv[]);
int Cmd_get_all(int argc, char *argv[]);

extern void CONSOLE_UART_IRQHandler(void);
#endif /* CMDLINE_COMMAND_H_ */
