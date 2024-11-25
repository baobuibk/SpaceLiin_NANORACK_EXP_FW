/*
 * status_led.c
 *
 *  Created on: Nov 20, 2024
 *      Author: SANG HUYNH
 */

#include "status_led.h"
#include "scheduler.h"
#include "main.h"


/* Private define ------------------------------------------------------------*/
#define	POWERUP_PERIOD	500
#define	POWER_NORMAL_OFF_PERIOD	3000
#define	POWER_NORMAL_ON_PERIOD	500
#define	ERROR_PERIOD	50

/* Private function ----------------------------------------------------------*/
	   void status_led_update(void);
static void status_led_powerup(void);
static void status_led_normal(void);
static void status_led_error(void);
static void status_led_led_green_on(void);
static void status_led_led_green_off(void);
static void status_led_led_blue_on(void);
static void status_led_led_blue_off(void);

/* Private typedef -----------------------------------------------------------*/
typedef struct Led_TaskContextTypedef
{
	SCH_TASK_HANDLE               	taskHandle;
	SCH_TaskPropertyTypedef       	taskProperty;
	uint32_t                      	taskTick;
} Led_TaskContextTypedef;

typedef	struct StatusLed_CurrentStateTypedef
{
	uint8_t							led_green:1;
	uint8_t							led_blue:1;
	Status_Led_StateTypedef			state:6;
}StatusLed_CurrentStateTypedef;

/* Private variables ---------------------------------------------------------*/
static	StatusLed_CurrentStateTypedef	s_led_display_status = {
	.led_green = 0,
	.led_blue = 0,
	.state = EXP_POWERUP
	};

static Led_TaskContextTypedef           s_task_context =
{
	SCH_INVALID_TASK_HANDLE,                // Will be updated by Schedular
	{
		SCH_TASK_SYNC,                      // taskType;
		SCH_TASK_PRIO_0,                    // taskPriority;
		50,                                // taskPeriodInMS;
		status_led_update,                	// taskFunction;
		48									// taskTick
	},
};

void status_led_init(void)
{
    s_led_display_status.led_green = 0;
    s_led_display_status.led_blue = 0;
    s_led_display_status.state = EXP_POWERUP;
	status_led_led_green_off();
	status_led_led_blue_off();
}

void	status_led_update(void)
{
	switch (s_led_display_status.state) {
	case EXP_POWERUP:
		status_led_powerup();
		break;
	case EXP_NORMAL:
		status_led_normal();
		break;
	case EXP_ERROR:
		status_led_error();
		break;
	default:
		break;
	}
}

static void status_led_powerup(void)
{
	if (s_led_display_status.led_green && s_led_display_status.led_blue) {
		if (SCH_TIM_HasCompleted(SCH_TIM_LED)) {
			s_led_display_status.led_green = 0;
			s_led_display_status.led_blue = 0;
			status_led_led_green_off();
			status_led_led_blue_off();
			SCH_TIM_Start(SCH_TIM_LED, POWERUP_PERIOD);
		}
	} else {
		if (SCH_TIM_HasCompleted(SCH_TIM_LED)) {
			s_led_display_status.led_green = 1;
			s_led_display_status.led_blue = 1;
			status_led_led_green_on();
			status_led_led_blue_on();
			SCH_TIM_Start(SCH_TIM_LED, POWERUP_PERIOD);
		}
	}
}

static void status_led_normal(void)
{
	// viết tạm
	LL_GPIO_ResetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
	LL_GPIO_SetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
	LL_mDelay(500);
	LL_GPIO_ResetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
	LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
	LL_mDelay(500);
}

static void status_led_error(void)
{
	if (s_led_display_status.led_green && s_led_display_status.led_blue) {
		if (SCH_TIM_HasCompleted(SCH_TIM_LED)) {
			s_led_display_status.led_green = 0;
			s_led_display_status.led_blue = 0;
			status_led_led_green_off();
			status_led_led_blue_off();
			SCH_TIM_Start(SCH_TIM_LED, ERROR_PERIOD);
		}
	} else {
		if (SCH_TIM_HasCompleted(SCH_TIM_LED)) {
			s_led_display_status.led_green = 1;
			s_led_display_status.led_blue = 1;
			status_led_led_green_on();
			status_led_led_blue_on();
			SCH_TIM_Start(SCH_TIM_LED, ERROR_PERIOD);
		}
	}
}

static void status_led_led_green_on(void)
{
	LL_GPIO_SetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
}
static void status_led_led_green_off(void)
{
	LL_GPIO_ResetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
}
static void status_led_led_blue_on(void)
{
	LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
}
static void status_led_led_blue_off(void)
{
	LL_GPIO_ResetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
}

void	status_led_create_task(void)
{
	SCH_TASK_CreateTask(&s_task_context.taskHandle, &s_task_context.taskProperty);
}
