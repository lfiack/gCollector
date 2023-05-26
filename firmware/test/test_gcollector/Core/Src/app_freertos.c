/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "shell.h"
#include "tim.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for xl320 */
osThreadId_t xl320Handle;
const osThreadAttr_t xl320_attributes = {
		.name = "xl320",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 128 * 4
};
/* Definitions for shell */
osThreadId_t shellHandle;
const osThreadAttr_t shell_attributes = {
		.name = "shell",
		.priority = (osPriority_t) osPriorityLow,
		.stack_size = 512 * 4
};
/* Definitions for motors */
osThreadId_t motorsHandle;
const osThreadAttr_t motors_attributes = {
		.name = "motors",
		.priority = (osPriority_t) osPriorityLow,
		.stack_size = 128 * 4
};
/* Definitions for qei */
osThreadId_t qeiHandle;
const osThreadAttr_t qei_attributes = {
		.name = "qei",
		.priority = (osPriority_t) osPriorityLow,
		.stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int sh_motors(h_shell_t * h_shell, int argc, char ** argv);
int sh_qei(h_shell_t * h_shell, int argc, char ** argv);
/* USER CODE END FunctionPrototypes */

void start_xl320_task(void *argument);
void start_shell_task(void *argument);
void start_motor_task(void *argument);
void start_qei_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of xl320 */
	xl320Handle = osThreadNew(start_xl320_task, NULL, &xl320_attributes);

	/* creation of shell */
	shellHandle = osThreadNew(start_shell_task, NULL, &shell_attributes);

	/* creation of motors */
	motorsHandle = osThreadNew(start_motor_task, NULL, &motors_attributes);

	/* creation of qei */
	qeiHandle = osThreadNew(start_qei_task, NULL, &qei_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_start_xl320_task */
/**
 * @brief  Function implementing the xl320 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_xl320_task */
void start_xl320_task(void *argument)
{
	/* USER CODE BEGIN start_xl320_task */
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		osDelay(100);
	}
	/* USER CODE END start_xl320_task */
}

/* USER CODE BEGIN Header_start_shell_task */
/**
 * @brief Function implementing the shell thread.
 * @param argument: Not used
 * @retval None
 */
static h_shell_t h_shell;

uint8_t drv_uart2_receive(char * pData, uint16_t size)
{
	HAL_UART_Receive(&huart2, (uint8_t*)(pData), size, HAL_MAX_DELAY);

	return 0;	// Life's too short for error management
}

uint8_t drv_uart2_transmit(const char * pData, uint16_t size)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)pData, size, HAL_MAX_DELAY);

	return 0;	// Srsly, don't do that kids
}

/* USER CODE END Header_start_shell_task */
void start_shell_task(void *argument)
{
	/* USER CODE BEGIN start_shell_task */
	h_shell.drv.receive = drv_uart2_receive;
	h_shell.drv.transmit = drv_uart2_transmit;

	shell_init(&h_shell);

	shell_add(&h_shell, 'p', sh_motors, "Test the motors");
	shell_add(&h_shell, 'q', sh_qei, "Test the encoders");

	/* Infinite loop */
	for(;;)
	{
		shell_run(&h_shell);
	}
	/* USER CODE END start_shell_task */
}

/* USER CODE BEGIN Header_start_motor_task */
/**
 * @brief Function implementing the motors thread.
 * @param argument: Not used
 * @retval None
 */
#define TIM_CHANNEL_M1_REV TIM_CHANNEL_2
#define TIM_CHANNEL_M1_FWD TIM_CHANNEL_1
#define TIM_CHANNEL_M2_FWD TIM_CHANNEL_3
#define TIM_CHANNEL_M2_REV TIM_CHANNEL_4

int sh_motors(h_shell_t * h_shell, int argc, char ** argv) {
	if (argc == 4)
	{
		uint32_t duty = atoi(argv[3]);

		if ((duty >= 0) && (duty <= 1023))
		{
			if (argv[1][0] == '1')
			{
				if (argv[2][0] == 'f')
				{
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_M1_FWD);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_M1_REV);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_M1_FWD, duty);

					return 0;
				}
				else if (argv[2][0] == 'r')
				{
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_M1_REV);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_M1_FWD);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_M1_REV, duty);

					return 0;
				}
			}
			else if (argv[1][0] == '2')
			{
				if (argv[2][0] == 'f')
				{
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_M2_FWD);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_M2_REV);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_M2_FWD, duty);

					return 0;
				}
				else if (argv[2][0] == 'r')
				{
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_M2_REV);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_M2_FWD);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_M2_REV, duty);

					return 0;
				}
			}
		}
	}

	uint16_t size = snprintf (h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE, "Error: Usage %c <1|2> <f|r> <0-1023>\r\n", argv[0][0]);
	h_shell->drv.transmit(h_shell->print_buffer, size);

	return -1;
}
/* USER CODE END Header_start_motor_task */
void start_motor_task(void *argument)
{
	/* USER CODE BEGIN start_motor_task */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END start_motor_task */
}

/* USER CODE BEGIN Header_start_qei_task */
/**
 * @brief Function implementing the qei thread.
 * @param argument: Not used
 * @retval None
 */

int sh_qei(h_shell_t * h_shell, int argc, char ** argv) {
	int16_t qei1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
	int16_t qei2 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
	uint16_t size = snprintf (h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE, "QEI1 = %hd ; QEI2 = %hd\r\n", qei1, qei2);
	h_shell->drv.transmit(h_shell->print_buffer, size);

	return 0;
}
/* USER CODE END Header_start_qei_task */


void start_qei_task(void *argument)
{
	/* USER CODE BEGIN start_qei_task */

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);

	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END start_qei_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

