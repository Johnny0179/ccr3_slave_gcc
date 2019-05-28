// main.c
// ccr3 project

// Created by Song Junlin on 4/28/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
/*-------------------------------Includes-------------------------------*/
/*STM32*/
#include "delay.h"
#include "led.h"
#include "sys.h"
#include "can.h"

/*FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"

/*FreeModbus*/
#include "freemodbus.h"

/*--------------------------------TASK---------------------------------*/

#define START_TASK_PRIO 1
#define START_STK_SIZE 128
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define Modbus_TASK_PRIO 5
#define Modbus_STK_SIZE 256
TaskHandle_t ModbusTask_Handler;
void Modbus_task(void *pvParameters);

#define CanTest_TASK_PRIO 4
#define CanTest_STK_SIZE 256
TaskHandle_t CanTestTask_Handler;
void CanTest_task(void *pvParameters);

/*----------------------------Start Implemention-------------------------*/

int main()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  LED_Init();

  // init the system delay
  delay_init(168);

  xTaskCreate((TaskFunction_t)start_task,
              (const char *)"start_task",
              (uint16_t)START_STK_SIZE,
              (void *)NULL,
              (UBaseType_t)START_TASK_PRIO,
              (TaskHandle_t *)&StartTask_Handler);
  vTaskStartScheduler();
  return 0;
}

/*------------------------------TASK Functions----------------------------*/
void start_task(void *pvParameters)
{

  taskENTER_CRITICAL();

  // creat modbus task
      xTaskCreate((TaskFunction_t)Modbus_task, (const char *)"Modbus_task",
                  (uint16_t)Modbus_STK_SIZE, (void *)NULL,
                  (UBaseType_t)Modbus_TASK_PRIO,
                  (TaskHandle_t *)&ModbusTask_Handler);
  // CAN test
      xTaskCreate((TaskFunction_t)CanTest_task, (const char *)"CanTest_task",
                  (uint16_t)CanTest_STK_SIZE, (void *)NULL,
                  (UBaseType_t)CanTest_TASK_PRIO,
                  (TaskHandle_t *)&CanTestTask_Handler);

  vTaskDelete(StartTask_Handler);
  taskEXIT_CRITICAL();
}

// Modbus task
void Modbus_task(void *pvParameters)
{
  // Modbus Init
  eMBInit(MB_RTU, 11, 0x01, 19200, MB_PAR_NONE);
  eMBEnable();
  while (1)
  {
    eMBPoll();
    vTaskDelay(50);
  }
}

// CAN test task
void CanTest_task(void *pvParameters)
{
  // CAN Init
  CanInit();

  // wait EPSO4 to init
  delay_ms(5000);

  // Start Node
  NMT_Start(0);

  // delay to wait for the command to be executed
  delay_ms(2);
  StartMotor(1);

  while (1)
  {
    vTaskDelay(5000);
  }
}
