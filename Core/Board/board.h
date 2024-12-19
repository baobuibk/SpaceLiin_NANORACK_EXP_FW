/*
 * board.h
 *
 *  Created on: Nov 21, 2024
 *      Author: SANG HUYNH
 */

#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_
#include <stm32f407xx.h>

//*****************************************************************************
// UART_CONSOLE
//*****************************************************************************
#define EXP_UART_HANDLE       USART3
#define EXP_UART_IRQ          USART3_IRQn


//*****************************************************************************
// UART_COPC
//*****************************************************************************
#define EXP_RS485_HANDLE       USART2
#define EXP_RS485_IRQ          USART2_IRQn




#endif /* BOARD_BOARD_H_ */
