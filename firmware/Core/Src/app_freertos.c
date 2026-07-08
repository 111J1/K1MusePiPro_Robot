/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include "task_arm.h"
#include "task_demo.h"
#include "task_lift.h"

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
/* Definitions for CommandTask */
osThreadId_t CommandTaskHandle;
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for ArmTask */
osThreadId_t ArmTaskHandle;
const osThreadAttr_t ArmTask_attributes = {
  .name = "ArmTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for LiftTask */
osThreadId_t LiftTaskHandle;
const osThreadAttr_t LiftTask_attributes = {
  .name = "LiftTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for PeripheralTask */
osThreadId_t PeripheralTaskHandle;
const osThreadAttr_t PeripheralTask_attributes = {
  .name = "PeripheralTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for DemoTask */
osThreadId_t DemoTaskHandle;
const osThreadAttr_t DemoTask_attributes = {
  .name = "DemoTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for ChassisCmdQueue */
osMessageQueueId_t ChassisCmdQueueHandle;
const osMessageQueueAttr_t ChassisCmdQueue_attributes = {
  .name = "ChassisCmdQueue"
};
/* Definitions for ArmCmdQueue */
osMessageQueueId_t ArmCmdQueueHandle;
const osMessageQueueAttr_t ArmCmdQueue_attributes = {
  .name = "ArmCmdQueue"
};
/* Definitions for LiftCmdQueue */
osMessageQueueId_t LiftCmdQueueHandle;
const osMessageQueueAttr_t LiftCmdQueue_attributes = {
  .name = "LiftCmdQueue"
};
/* Definitions for TelemetryTxQueue */
osMessageQueueId_t TelemetryTxQueueHandle;
const osMessageQueueAttr_t TelemetryTxQueue_attributes = {
  .name = "TelemetryTxQueue"
};
/* Definitions for DemoCmdQueue */
osMessageQueueId_t DemoCmdQueueHandle;
const osMessageQueueAttr_t DemoCmdQueue_attributes = {
  .name = "DemoCmdQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCommandTask(void *argument);
void StartChassisTask(void *argument);
void StartArmTask(void *argument);
void StartLiftTask(void *argument);
void StartOledTask(void *argument);
void StartTelemetryTask(void *argument);
void StartPeripheralTask(void *argument);
void StartDemoTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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

  /* Create the queue(s) */
  /* creation of ChassisCmdQueue */
  ChassisCmdQueueHandle = osMessageQueueNew (8, 20, &ChassisCmdQueue_attributes);

  /* creation of ArmCmdQueue */
  ArmCmdQueueHandle = osMessageQueueNew (8, 32, &ArmCmdQueue_attributes);

  /* creation of LiftCmdQueue */
  LiftCmdQueueHandle = osMessageQueueNew (8, 12, &LiftCmdQueue_attributes);

  /* creation of TelemetryTxQueue */
  TelemetryTxQueueHandle = osMessageQueueNew (12, 72, &TelemetryTxQueue_attributes);

  /* creation of DemoCmdQueue */
  DemoCmdQueueHandle = osMessageQueueNew (8, 12, &DemoCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CommandTask */
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of ArmTask */
  ArmTaskHandle = osThreadNew(StartArmTask, NULL, &ArmTask_attributes);

  /* creation of LiftTask */
  LiftTaskHandle = osThreadNew(StartLiftTask, NULL, &LiftTask_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* creation of PeripheralTask */
  PeripheralTaskHandle = osThreadNew(StartPeripheralTask, NULL, &PeripheralTask_attributes);

  /* creation of DemoTask */
  DemoTaskHandle = osThreadNew(StartDemoTask, NULL, &DemoTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCommandTask */
/**
  * @brief  Function implementing the CommandTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommandTask */
__weak void StartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartCommandTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCommandTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
__weak void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartArmTask */
/**
* @brief Function implementing the ArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartArmTask */
__weak void StartArmTask(void *argument)
{
  /* USER CODE BEGIN StartArmTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartArmTask */
}

/* USER CODE BEGIN Header_StartLiftTask */
/**
* @brief Function implementing the LiftTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLiftTask */
__weak void StartLiftTask(void *argument)
{
  /* USER CODE BEGIN StartLiftTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLiftTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
__weak void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
__weak void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartPeripheralTask */
/**
* @brief Function implementing the PeripheralTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPeripheralTask */
__weak void StartPeripheralTask(void *argument)
{
  /* USER CODE BEGIN StartPeripheralTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPeripheralTask */
}

/* USER CODE BEGIN Header_StartDemoTask */
/**
* @brief Function implementing the DemoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDemoTask */
__weak void StartDemoTask(void *argument)
{
  /* USER CODE BEGIN StartDemoTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDemoTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

