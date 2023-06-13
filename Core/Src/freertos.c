/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define ADC_TIMER_MILISEC 1000
//uint32_t ADC_THRESHOLD_VALUE=755;
uint32_t ADC_THRESHOLD_VALUE=625;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

//ADC
uint32_t ADC_raw;

// messages
uint8_t new_message=0;
char ADC_messsage_recieved[ADC_MESSAGE_BYTE_LENGTH]="";
StreamBufferHandle_t ADC_messsage_buffer_handle;
size_t xTriggerLevelBytes=1;

// I2C
HAL_StatusTypeDef i2c_ret;
uint8_t i2c_buff[16];

//TMP117 sensor variables
//uint32_t sen1_val=0;
static const float temp_bin_to_float=0.0078125; //[C]
static const uint8_t TMP117_addr=0x48<<1;
static const uint8_t REG_TEMP=0x00;
int16_t tmp117_val;
float tmp117_temp_c=0;

// VL53L0X sensor variables
uint32_t sen2_val=0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId monitorTaskHandle;
osThreadId read_i2c_sen1Handle;
osThreadId read_i2c_sen2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartMonitorTask(void const * argument);
void StartRead_i2c_sen1(void const * argument);
void startRead_i2c_sen2(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 400);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of monitorTask */
  osThreadDef(monitorTask, StartMonitorTask, osPriorityNormal, 0, 512);
  monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

  /* definition and creation of read_i2c_sen1 */
  osThreadDef(read_i2c_sen1, StartRead_i2c_sen1, osPriorityNormal, 0, 400);
  read_i2c_sen1Handle = osThreadCreate(osThread(read_i2c_sen1), NULL);

  /* definition and creation of read_i2c_sen2 */
  osThreadDef(read_i2c_sen2, startRead_i2c_sen2, osPriorityNormal, 0, 400);
  read_i2c_sen2Handle = osThreadCreate(osThread(read_i2c_sen2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //create stream buffer
  ADC_messsage_buffer_handle=xStreamBufferCreate(ADC_MESSAGE_BYTE_LENGTH,xTriggerLevelBytes);


  //start hardware timer
  HAL_TIM_Base_Start(&htim2);
  //start ADC interrupt
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	printf("Program alive\r\n");
    osDelay(20000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the monitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void const * argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  /* Infinite loop */
  for(;;)
  {
	  //printf("Monitor Task\r\n");
	  //xStreamBufferReceive(xStreamBuffer, pvRxData, xBufferLengthBytes, xTicksToWait);
	  xStreamBufferReceive(ADC_messsage_buffer_handle, ADC_messsage_recieved, ADC_MESSAGE_BYTE_LENGTH, 100);
	  //if(!strcmp(ADC_messsage_recieved,""))
	  if(new_message)
	  {
		  printf("MESS received from streamBuffer: %s\r\n",ADC_messsage_recieved);
		  new_message=0;
		  //strcpy(ADC_messsage_recieved,"");
	  }
	  osDelay(1000);
  }
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartRead_i2c_sen1 */
/**
* @brief Function implementing the read_i2c_sen1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRead_i2c_sen1 */
void StartRead_i2c_sen1(void const * argument)
{
  /* USER CODE BEGIN StartRead_i2c_sen1 */
	// sensor TMP117
  /* Infinite loop */
  for(;;)
  {
	//I2C COMUNICATION
	i2c_buff[0]=REG_TEMP;
	i2c_ret=HAL_I2C_Master_Transmit(&hi2c1, TMP117_addr, i2c_buff, 1, HAL_MAX_DELAY);
	if(i2c_ret!=HAL_OK)
	{
		printf("i2c TX error\r\n");
	}
	else
	{
		//printf("i2c TX status HAL_OK\r\n");
		i2c_ret=HAL_I2C_Master_Receive(&hi2c1, TMP117_addr, i2c_buff, 2,HAL_MAX_DELAY);
		if(i2c_ret!=HAL_OK)
		{
			printf("i2c Rx error\r\n");
		}
		else
		{
			//printf("i2c RX status HAL_OK\r\n");
			// connect 2 bytes into one int16
			tmp117_val=((int16_t)i2c_buff[0]<<8 )|((i2c_buff[1]));

			//Convert binary value to float temp. in Celcius degrees
			tmp117_temp_c=tmp117_val*temp_bin_to_float;
			printf("TMP117 temp= %f C\r\n",tmp117_temp_c);
		}
	}

    osDelay(1000);
  }
  /* USER CODE END StartRead_i2c_sen1 */
}

/* USER CODE BEGIN Header_startRead_i2c_sen2 */
/**
* @brief Function implementing the read_i2c_sen2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startRead_i2c_sen2 */
void startRead_i2c_sen2(void const * argument)
{
  /* USER CODE BEGIN startRead_i2c_sen2 */
	// sensor VL53L0X
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END startRead_i2c_sen2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
