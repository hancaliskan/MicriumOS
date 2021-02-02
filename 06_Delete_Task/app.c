//OS_CFG_TASK_DEL_EN must be enabled in os_cfg.h

#include "os.h"                         // Micrium.Micrium::RTOS:uC/OS Kernel
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include <stdint.h>

OS_TCB 	redLedControllerTaskTCB;
CPU_STK redLedControllerTaskSTK[128u];

OS_TCB  blueLedControllerTaskTCB;
CPU_STK blueLedControllerTaskSTK[128u];

OS_TCB 	orangeLedControllerTaskTCB;
CPU_STK orangeLedControllerTaskSTK[128u];

void SystemClock_Config(void);

void GPIO_Init(void);

OS_ERR os_err;

void orangeLedControllerTask(void *p_arg)
{
	 //OSTaskDel(&orangeLedControllerTaskTCB,&os_err);
	
	 while(DEF_TRUE)
	 {
		     OSTimeDlyHMSM(0,
                      0,
                      0,
                      500,
                      OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
		   HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	 }
}


void redLedControllerTask(void *p_arg)
{
	 while(DEF_TRUE)
	 {
		     OSTimeDlyHMSM(0,
                      0,
                      0,
                      1000,
                      OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
		   HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
	 }
}

void blueLedControllerTask(void *p_arg)
{
	 	OSTaskCreate((OS_TCB     *)&redLedControllerTaskTCB,                /* Create the red task                                    */
                 (CPU_CHAR   *)"redLed controller",
                 (OS_TASK_PTR ) redLedControllerTask,
                 (void       *) 0,
                 (OS_PRIO     ) 1,
                 (CPU_STK    *)&redLedControllerTaskSTK[0],
                 (CPU_STK     ) 0u,
                 (CPU_STK_SIZE) 128u,
                 (OS_MSG_QTY  ) 0,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&os_err);
								 
		OSTaskCreate((OS_TCB     *)&orangeLedControllerTaskTCB,                /* Create the orange task                                    */
                 (CPU_CHAR   *)"orangeLed controller",
                 (OS_TASK_PTR ) orangeLedControllerTask,
                 (void       *) 0,
                 (OS_PRIO     ) 1,
                 (CPU_STK    *)&orangeLedControllerTaskSTK[0],
                 (CPU_STK     ) 0u,
                 (CPU_STK_SIZE) 128u,
                 (OS_MSG_QTY  ) 0,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&os_err);
								 
	 OSTaskDel(&redLedControllerTaskTCB,&os_err);
								 
	 while(DEF_TRUE)
	 {
		     OSTimeDlyHMSM(0,
                      0,
                      0,
                      50,
                      OS_OPT_TIME_HMSM_STRICT,
                      &os_err);
		   HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
	 }
}


int main()
{
	SystemClock_Config();                     /* Configure the System Clock     */
	GPIO_Init();

	OSInit(&os_err);
	  
	
		OSTaskCreate((OS_TCB     *)&blueLedControllerTaskTCB,                /* Create the blue task                                    */
                 (CPU_CHAR   *)"blueLed controller",
                 (OS_TASK_PTR ) blueLedControllerTask,
                 (void       *) 0,
                 (OS_PRIO     ) 1,
                 (CPU_STK    *)&blueLedControllerTaskSTK[0],
                 (CPU_STK     ) 0u,
                 (CPU_STK_SIZE) 128u,
                 (OS_MSG_QTY  ) 0,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&os_err);
								 
    OSStart(&os_err);

}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
 

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

void GPIO_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);	
}