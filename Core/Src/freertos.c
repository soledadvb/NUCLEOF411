/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "lis2dw12_reg.h"
#include "i2c.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "DataScope_DP.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_LEN 3
#define QUEUE_SIZE sizeof(uint16_t)
#define DATA_LENGTH 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t LIS2DW12_Data_Get_Task_Handle = NULL;
// static TaskHandle_t LIS2DW12_Data_Save_Task_Handle = NULL;
static TaskHandle_t LIS2DW12_Data_Send_Task_Handle = NULL;

QueueHandle_t Acceleration_Data_Queue_Handle = NULL;

SemaphoreHandle_t Data_Get_BinarySem_Handle = NULL;
SemaphoreHandle_t Data_Save_BinarySem_Handle = NULL;
SemaphoreHandle_t Data_Send_BinarySem_Handle = NULL;

uint8_t tx_buffer[1000];

int16_t data_raw_acceleration[3];
float acceleration_mg[3];

float data_acceleration_buff_x[512];
float data_acceleration_buff_y[512];
float data_acceleration_buff_z[512];

uint8_t data_full_flag;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void AppTaskCreate(void *parameter);

static void LIS2DW12_Data_Get_Task(void *parameter);
// static void LIS2DW12_Data_Save_Task(void *parameter);
static void LIS2DW12_Data_Send_Task(void *parameter);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  BaseType_t xReturn = pdPASS;

  xReturn = xTaskCreate(AppTaskCreate, "AppTaskCreate", 512, NULL, 1, &AppTaskCreate_Handle);
  if (pdPASS == xReturn)
  {
    printf("AppTaskCreate任务创建成功!/r/n");
  }

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

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void AppTaskCreate(void *parameter)
{
  BaseType_t xReturn = pdPASS;
  taskENTER_CRITICAL();
  Acceleration_Data_Queue_Handle = xQueueCreate((UBaseType_t)QUEUE_LEN, (UBaseType_t)QUEUE_SIZE);
  Data_Get_BinarySem_Handle = xSemaphoreCreateBinary();
  Data_Save_BinarySem_Handle = xSemaphoreCreateBinary();
  Data_Send_BinarySem_Handle = xSemaphoreCreateBinary();
  xReturn = xTaskCreate((TaskFunction_t)LIS2DW12_Data_Get_Task, (const char *)"LIS2DW12_Data_Get_Task", (uint16_t)512, (void *)NULL, (UBaseType_t)2, (TaskHandle_t *)&LIS2DW12_Data_Get_Task_Handle);
  if (pdPASS == xReturn)
    printf("Data_Get任务创建成功!/r/n");
  // xReturn = xTaskCreate((TaskFunction_t)LIS2DW12_Data_Save_Task, (const char *)"LIS2DW12_Data_Save_Task", (uint16_t)512, (void *)NULL, (UBaseType_t)2, (TaskHandle_t *)&LIS2DW12_Data_Save_Task_Handle);
  // if (pdPASS == xReturn)
  //   printf("Data_Save任务创建成功!/r/n");

  xReturn = xTaskCreate((TaskFunction_t)LIS2DW12_Data_Send_Task, (const char *)"LIS2DW12_Data_Send_Task", (uint16_t)512, (void *)NULL, (UBaseType_t)1, (TaskHandle_t *)&LIS2DW12_Data_Send_Task_Handle);
  if (pdPASS == xReturn)
    printf("Data_Send任务创建成功!/r/n");
  vTaskDelete(AppTaskCreate_Handle);
  taskEXIT_CRITICAL();
}

static void LIS2DW12_Data_Get_Task(void *parameter)
{
  uint16_t data_cnt;
  for (;;)
  {
    uint8_t reg;
    /* Read output only if new value is available */
    if (data_full_flag == 0)
    {
      lis2dw12_flag_data_ready_get(&dev_ctx, &reg);

      if (reg)
      {
        /* Read acceleration data */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        // acceleration_mg[0] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[0]);
        // acceleration_mg[1] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[1]);
        // acceleration_mg[2] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[2]);

        /* Start convert data */
        data_acceleration_buff_x[data_cnt] = lis2dw12_from_fs2_to_mg(
            data_raw_acceleration[0]);
        data_acceleration_buff_y[data_cnt] = lis2dw12_from_fs2_to_mg(
            data_raw_acceleration[1]);
        data_acceleration_buff_z[data_cnt] = lis2dw12_from_fs2_to_mg(
            data_raw_acceleration[2]);
        /* convert end */
        data_cnt++;

        if (data_cnt == DATA_LENGTH)
        {
          data_full_flag = 1;
          data_cnt = 0;
          osDelay(DATA_LENGTH * 20);
        }
        // sprintf((char *)tx_buffer,
        //         "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\t第%d个数据\r\n",
        //         acceleration_mg[0], acceleration_mg[1], acceleration_mg[2], ++cnt);
        // tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }
    }
  }
}

// static void LIS2DW12_Data_Save_Task(void *parameter)
// {
//   BaseType_t xReturn = pdPASS;
//   uint16_t data_cnt = 0;
//   for (;;)
//   {
//     xReturn = xSemaphoreTake(Data_Get_BinarySem_Handle, portMAX_DELAY);
//     if (pdTRUE == xReturn)
//     {
//       printf("获取数据成功!\r\n");
//       data_acceleration_buff_x[data_cnt] = acceleration_mg[0];

//       data_acceleration_buff_y[data_cnt] = acceleration_mg[1];

//       data_acceleration_buff_z[data_cnt] = acceleration_mg[2];

//       data_cnt++;

//       if (data_cnt == DATA_LENGTH)
//         data_cnt = 0;
//     }
//     osDelay(800);
//   }
// }

static void LIS2DW12_Data_Send_Task(void *parameter)
{
  uint16_t index;
  for (;;)
  {
    if (data_full_flag == 1)
    {
      DataScope_Send_3axis_Data(data_acceleration_buff_x[index++], data_acceleration_buff_y[index++], data_acceleration_buff_z[index++]);
      if (index == DATA_LENGTH)
      {
        index = 0;
        data_full_flag = 0;
      }
    }
    osDelay(20);
  }
}

/* USER CODE END Application */
