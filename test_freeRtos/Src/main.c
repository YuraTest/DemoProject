/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "portmacro.h"
#include "uart.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;
SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

osThreadId defaultTaskHandle;

/* Двоичный семафор – глобальная переменная */
xSemaphoreHandle xBinarySemaphore;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDIO_SD_Init(void);
void StartDefaultTask(void const * argument);

void TaskSdio(void const * argument);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void vTaskRedLed(void *pvParameters);
void vTaskBlueLed(void *pvParameters);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void vTaskRedLed(void *pvParameters) {
	//const char *pcTaskName = "Task 1 is running\r\n";
	//uint32_t uDelay;// = (uint32_t *) pvParameters;
	//static portBASE_TYPE xHigherPriorityTaskWoken;
	for (;;) {
		//osDelay(250);
		vTaskDelay(500);
		HAL_GPIO_TogglePin(GPIOD, RedLed_Pin);
		//xSemaphoreGive(xBinarySemaphore);
	}
}

void vTaskBlueLed(void *pvParameters) {

	for (;;) {
		/* Реализовано ожидание события с помощью двоичного
		 семафора. Семафор после создания становится
		 доступен (так, как будто его кто-то отдал).
		 Поэтому сразу после запуска планировщика задача
		 захватит его. Второй раз сделать это ей не удастся,
		 и она будет ожидать, находясь в блокированном
		 состоянии, пока семафор не отдаст обработчик
		 прерывания. Время ожидания задано равным
		 бесконечности, поэтому нет необходимости проверять
		 возвращаемое функцией xSemaphoreTake() значение. */
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		/* Если программа “дошла” до этого места, значит,
		 семафор был успешно захвачен.
		 Обработка события, связанного с семафором.
		 В нашем случае – мигание светодиодом. */
		//HAL_GPIO_TogglePin(GPIOD, BlueLed_Pin);

	}

}

static portBASE_TYPE xHigherPriorityTaskWoken;
static portBASE_TYPE xCount;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	HAL_GPIO_TogglePin(GPIOD, OrangeLed_Pin);
	xCount++;
	if (xCount == 2) {
		xCount = 0;
		xHigherPriorityTaskWoken = pdFALSE;
		/* Отдать семафор задаче-обработчику */
		xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
		//xSemaphoreGive(xBinarySemaphore);
		//if (xHigherPriorityTaskWoken == pdTRUE) {
		/* Это разблокирует задачу-обработчик. При этом
		 приоритет задачи-обработчика выше приоритета
		 выполняющейся в данный момент периодической
		 задачи. Поэтому переключаем контекст
		 принудительно – так мы добьемся того, что после
		 выполнения обработчика прерывания управление
		 получит задача-обработчик.*/
		/* Макрос, выполняющий переключение контекста.
		 * На других платформах имя макроса может быть другое! */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		//portYIELD();
	}
}
/* USER CODE END 0 */

uart_t uart1;

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM6_Init();
	MX_USART1_UART_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	if(BSP_SD_Init() != pdPASS){
		Error_Handler();
	} else {
		HAL_GPIO_WritePin(GPIOD, GreenLed_Pin, 1);
	}
	//HAL_SD_ErrorTypedef HAL_SD_Get_CardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *pCardInfo)
	//HAL_SD_Get_CardInfo(&hsd, SDCardInfo);
	/*FATFS fs;
	FRESULT res;
	FILINFO fno;
	DIR dir;
	f_readdir(NULL, *fno);*/

	if (sizeof(portBASE_TYPE) > 4) {
		HAL_GPIO_TogglePin(GPIOD, RedLed_Pin);
		for (portBASE_TYPE n = 0; n < 2000000000; n++)
			;
		HAL_GPIO_TogglePin(GPIOD, RedLed_Pin);
	} else {
		HAL_GPIO_TogglePin(GPIOD, BlueLed_Pin);
		for (portBASE_TYPE n = 0; n < 2000000000; n++)
			;
		HAL_GPIO_TogglePin(GPIOD, BlueLed_Pin);
	}

	vSemaphoreCreateBinary(xBinarySemaphore);
	if (xBinarySemaphore != NULL) {

		/* USER CODE BEGIN 2 */

		/* USER CODE END 2 */

		/* USER CODE BEGIN RTOS_MUTEX */
		/* add mutexes, ... */
		/* USER CODE END RTOS_MUTEX */

		/* USER CODE BEGIN RTOS_SEMAPHORES */
		/* add semaphores, ... */
		/* USER CODE END RTOS_SEMAPHORES */

		/* USER CODE BEGIN RTOS_TIMERS */
		/* start timers, add new ones, ... */
		/* USER CODE END RTOS_TIMERS */

		/* Create the thread(s) */
		/* definition and creation of defaultTask */
		/* Перед использованием семафор необходимо создать. */

		//MX_FREERTOS_UART_Init();
		if(uart_init(&uart1, &huart1) != pdPASS){
			Error_Handler();
		}

		if (initUartTask(&uart1) != pdPASS)
			Error_Handler();
		osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
		defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

		xTaskCreate(vTaskRedLed, "Task 2", 1000, NULL, 1, NULL);
		xTaskCreate(vTaskBlueLed, "Task 3", 1000, NULL, 3, NULL);
		/* USER CODE BEGIN RTOS_THREADS */
		/* add threads, ... */
		/* USER CODE END RTOS_THREADS */

		/* USER CODE BEGIN RTOS_QUEUES */
		/* add queues, ... */
		/* USER CODE END RTOS_QUEUES */

		HAL_TIM_Base_Start_IT(&htim6);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		/* Start scheduler */
		osKernelStart();
	}

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 7;


}

/* TIM6 init function */
static void MX_TIM6_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 41999;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1000;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	GreenLed_Pin | OrangeLed_Pin | RedLed_Pin | BlueLed_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : GreenLed_Pin OrangeLed_Pin RedLed_Pin BlueLed_Pin */
	GPIO_InitStruct.Pin = GreenLed_Pin | OrangeLed_Pin | RedLed_Pin
			| BlueLed_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(500);
		HAL_GPIO_TogglePin(GPIOD, GreenLed_Pin);
	}
	/* USER CODE END 5 */
}

void TaskSdio(void const * argument) {
	HAL_SD_MspInit(&hsd);

	for (;;) {

	}
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		 HAL_GPIO_WritePin(GPIOD, BlueLed_Pin, 1);
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
