/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include "tim.h"
#include "adc.h"
#include "stdio.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int upEdge=0;
int downEdge=0;
int distance=0;
int value=0;
int light=0;
volatile int capture_valid=0;  /* 本轮是否成功捕获到有效距离 */
volatile int Flag=0;
uint8_t rx_byte;
/* 命令接收缓冲区 */
char cmd_buf[64];
volatile int cmd_len = 0 ;
/* 阈值变量（CmdTask写，AlarmTask读，需要互斥锁保护） */
int dist_threshold = 30 ;
int light_threshold = 40 ;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CmdTask */
osThreadId_t CmdTaskHandle;
const osThreadAttr_t CmdTask_attributes = {
  .name = "CmdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for AlarmTask */
osThreadId_t AlarmTaskHandle;
const osThreadAttr_t AlarmTask_attributes = {
  .name = "AlarmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* Definitions for RecBinarySem */
osSemaphoreId_t RecBinarySemHandle;
const osSemaphoreAttr_t RecBinarySem_attributes = {
  .name = "RecBinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StarSensorTask(void *argument);
void StartDisplayTask(void *argument);
void StartCmdTask(void *argument);
void StartAlarmTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RecBinarySem */
  RecBinarySemHandle = osSemaphoreNew(1, 0, &RecBinarySem_attributes);

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
  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StarSensorTask, NULL, &SensorTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of CmdTask */
  CmdTaskHandle = osThreadNew(StartCmdTask, NULL, &CmdTask_attributes);

  /* creation of AlarmTask */
  AlarmTaskHandle = osThreadNew(StartAlarmTask, NULL, &AlarmTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StarSensorTask */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StarSensorTask */
void StarSensorTask(void *argument)
{
  /* USER CODE BEGIN StarSensorTask */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

  /* 关闭输入滤波：filter=15 太严格，会把有效的 Echo 边沿滤掉 */
  htim1.Instance->CCMR2 &= ~(TIM_CCMR2_IC3F | TIM_CCMR2_IC4F);

  /* ADC 校准（F1 系列必须做，否则读数不准） */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Infinite loop */
  for(;;)
  {
    /* 每轮发 Trig 前：清捕获标志 + 复位有效标志，确保边沿配对 */
    capture_valid = 0;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC3 | TIM_FLAG_CC4);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
    osDelay(100);

    /* 单次 ADC 转换：启动→等待→读数，超时 10ms 防止卡死 */
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      value = (int)HAL_ADC_GetValue(&hadc1);
      light = value * 100 / 4095;   /* 先乘后除，避免整数除法得 0 */
    }
  }
  /* USER CODE END StarSensorTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  osDelay(20);
  OLED_Init();

  char msg[20];
  /* Infinite loop */
  for(;;)
  {
    OLED_NewFrame();
    sprintf(msg,"distance:%dcm",distance);
    OLED_PrintString(0,0,msg,&font16x16,OLED_COLOR_NORMAL);
    sprintf(msg,"light:%d%%",light);
    OLED_PrintString(0,20,msg,&font16x16,OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    osDelay(500);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartCmdTask */
/**
* @brief Function implementing the CmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdTask */
void StartCmdTask(void *argument)
{
  /* USER CODE BEGIN StartCmdTask */
  HAL_UART_Receive_IT(&huart2,&rx_byte,1);
  HAL_UART_Transmit(&huart2, ( uint8_t *) "CMD ready. Type 'help'\r\n" , 24 , 100 );
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(RecBinarySemHandle,osWaitForever);
    if (strncmp(cmd_buf, "help", 4) == 0)
    {
      HAL_UART_Transmit(&huart2, (uint8_t *)
        "Commands:\r\n  get          - show distance & light\r\n"
        "  set dist N   - set distance threshold (cm)\r\n"
        "  set light N  - set light threshold (%)\r\n", 130, 100);
    } else if (strncmp(cmd_buf, "get", 3) == 0)
    {
      char reply[64];
      sprintf(reply, "distance=%dcm, light=%d%%\r\n", distance, light);
      HAL_UART_Transmit(&huart2, (uint8_t *)reply, strlen(reply), 100);
    }else if (strncmp(cmd_buf,"set dist",8)==0)
    {
      int val=atoi(cmd_buf+9);
      osMutexAcquire(myMutex01Handle,osWaitForever);
      dist_threshold=val;
      osMutexRelease(myMutex01Handle);
      char reply[30];
      sprintf(reply,"ok,dist_threshold=%d ", dist_threshold);
      HAL_UART_Transmit(&huart2, (uint8_t *)reply, strlen(reply), 100);
    }else if (strncmp(cmd_buf,"set light",9)==0)
    {
      int val=atoi(cmd_buf+10);
      osMutexAcquire(myMutex01Handle,osWaitForever);
      light_threshold=val;
      osMutexRelease(myMutex01Handle);
      char reply[30];
      sprintf(reply,"ok,light_threshold=%d ", light_threshold);
      HAL_UART_Transmit(&huart2, (uint8_t *)reply, strlen(reply), 100);
    }else
    {
      HAL_UART_Transmit(&huart2, (uint8_t *)"Unknown command, type 'help'\r\n", 31, 100);
    }
    osDelay(1);
  }
  /* USER CODE END StartCmdTask */
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
* @brief Function implementing the AlarmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void *argument)
{
  osDelay(1000);
  for(;;)
  {
    /* 读阈值时加锁，防止 CmdTask 改到一半 */
    osMutexAcquire(myMutex01Handle, osWaitForever);
    int d_th = dist_threshold;
    int l_th = light_threshold;
    osMutexRelease(myMutex01Handle);

    if (distance > 0 && distance < d_th && light < l_th) {
      Flag = 1;
    }
    if (Flag == 1) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      osDelay(500);
    } else {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
    osDelay(1);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 输入捕获回调：Echo下降沿触发，计算距离 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    upEdge   = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    downEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    int duration = downEdge - upEdge;
    /* 有效性检查：HC-SR04 量程 2~400cm，对应时长 118~23529us */
    if (duration > 118 && duration < 23529)
    {
      distance = duration * 17 / 1000;
      capture_valid = 1;
    }
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart==&huart2) {
    if (rx_byte=='\n') {
      cmd_buf[cmd_len]='\0';
      osSemaphoreRelease(RecBinarySemHandle);
      cmd_len=0;
    }else if (rx_byte!='\r') {
      if (cmd_len<63) {
        cmd_buf[cmd_len]=rx_byte;
        cmd_len++;
      }

    }
    HAL_UART_Receive_IT(&huart2,&rx_byte,1);
  }
}

/* USER CODE END Application */

