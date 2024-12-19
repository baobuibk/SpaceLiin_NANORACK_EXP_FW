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

/* Command support */
int Cmd_help(int argc, char *argv[]);
/* Command for power supply */
int Cmd_tec_pw_on(int argc, char *argv[]);
int Cmd_led_pw_on(int argc, char *argv[]);
int Cmd_heater_pw_on(int argc, char *argv[]);
/* Command for temperature */
int Cmd_set_temp(int argc, char *argv[]);
int Cmd_get_temp(int argc, char *argv[]);
int Cmd_get_temp_setpoint(int argc, char *argv[]);
int Cmd_set_tec_vol(int argc, char *argv[]);
int Cmd_get_tec_vol(int argc, char *argv[]);
int Cmd_set_heater_duty(int argc, char *argv[]);
int Cmd_get_heater_duty(int argc, char *argv[]);
int Cmd_temp_ctrl(int argc, char *argv[]);
int Cmd_temp_auto_ctrl(int argc, char *argv[]);
/* Command for ir led */
int Cmd_set_ir_duty(int argc, char *argv[]);
int Cmd_get_ir_duty(int argc, char *argv[]);
/* Command for i2c sensor */
int Cmd_get_acceleration_gyroscope(int argc, char *argv[]);
int Cmd_get_pressure(int argc, char *argv[]);
/* Command for system */
int Cmd_get_all(int argc, char *argv[]);



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

void CONSOLE_UART_IRQHandler(void);
#endif /* CMDLINE_COMMAND_H_ */
