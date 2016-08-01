#include "main.h"
#include "periph_init.h"
#include "tim6.h"
#include "uart.h"
#include "uart1Task.h"

RCC_ClocksTypeDef RCC_Clocks;
uart_t uart1;

void vTaskGreenLed(void *pvParameters);

uint32_t timetLed;
void TIM6_DAC_IRQHandler() {
	/* Так как этот обработчик вызывается и для ЦАП, нужно проверять,
	 * произошло ли прерывание по переполнению счётчика таймера TIM6.
	 */
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
		/* Очищаем бит обрабатываемого прерывания */
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		timetLed++;
		if (timetLed == 250) {
			timetLed = 0;
			GPIO_ToggleBits(OrangeLed_GPIO_Port, OrangeLed_Pin);

		}
	}
}

void vTaskGreenLed(void *pvParameters) {
	//const char *pcTaskName = "Task 1 is running\r\n";
	//uint32_t uDelay;// = (uint32_t *) pvParameters;
	//static portBASE_TYPE xHigherPriorityTaskWoken;
	for (;;) {
		//osDelay(250);
		vTaskDelay(1000);
		GPIO_ToggleBits(GreenLed_GPIO_Port, GreenLed_Pin);
		//xSemaphoreGive(xBinarySemaphore);
	}
}

int main(void) {
	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	SysTick_Config((RCC_Clocks.HCLK_Frequency / 1000));/*/168000);*/

	initGPIO();

	NVIC_SetPriority(SysTick_IRQn, 15);
	NVIC_EnableIRQ(SysTick_IRQn);
	timer6_init();

	if (uart_init(&uart1, USART1, 115200) != pdPASS) {
		GPIO_SetBits(BlueLed_GPIO_Port, BlueLed_Pin);
	}
	if (initUartTask(&uart1) != pdPASS) {
		GPIO_SetBits(BlueLed_GPIO_Port, BlueLed_Pin);
	}
	xTaskCreate(vTaskGreenLed, "GreenLed", 1000, NULL, 1, NULL);

	vTaskStartScheduler();

	while (1) {

	}
	return 0;
}

static uint32_t sysCount;
//void SysTickHandler(void){
void vApplicationTickHook(void) {
	sysCount++;
	if (!(sysCount % 500)) {
		//sysCount = 0;
		GPIO_ToggleBits(GPIOD, RedLed_Pin);
	}
}
