#include "FreeRTOS.h"
#include "main.h"
#include "periph_init.h"
#include "tim6.h"
#include "uart.h"
#include "uart1Task.h"

#include "stm32_ub_fatfs.h"
#include "ff.h"
#include "sd.h"

RCC_ClocksTypeDef RCC_Clocks;
uart_t uart1;
char textBuf[64];

void vTaskGreenLed(void *pvParameters);
void CONSOLE_printFiles(void);

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
	uint32_t n = 0;

	for (;;) {
		if (n == 0) {
			n = 1;
			if (SDIO_isSDCardAccessible() == true) {
				if (SDIO_mountSDCard() == true) {
					sprintf(textBuf, "SDCard is mounted!\n");
					PutStringUart(&uart1, textBuf);
					SDIO_scanFiles();
					CONSOLE_printFiles();

				} else {
					sprintf(textBuf, "Mount SD Card Failed!\n\r");
					PutStringUart(&uart1, textBuf);
				}

			} else {
				sprintf(textBuf, "No SD Card!\n\r");
				PutStringUart(&uart1, textBuf);
			}
		}
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
	UB_Fatfs_Init();

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

extern char SDIO_currentPath[SDIO_CURRENT_PATH_MAX_LENGTH];
extern uint16_t SDIO_filesNum;
extern struct SDIO_fileStruct SDIO_files[SDIO_FILES_TO_VIEW_MAX];

void CONSOLE_printFiles(void) {
	static char i;
	static char _consoleChar;
	static char _selectedFileIndex = 0;

	sprintf(textBuf, "1 - up, 2 - down, 3 - select, 4 - back, 5 - stop\n");
	PutStringUart(&uart1, textBuf);

	sprintf(textBuf, "%s\n", SDIO_currentPath);
	PutStringUart(&uart1, textBuf);

	for (i = 0; i < SDIO_filesNum; i++) {
		//if (i == _selectedFileIndex)
		//	printf(CONSOLE_TEXT_COLOR_LIGHT_CYAN);
		//else
		//	printf(CONSOLE_TEXT_COLOR_DEFAUL);

		if (SDIO_files[i].isDirectory) {
			sprintf(textBuf, "dir: %s", SDIO_files[i].fileName);
			PutStringUart(&uart1, textBuf);
			//printf("dir: %s", SDIO_files[i].fileName);
		} else {
			sprintf(textBuf, "fil: %s", SDIO_files[i].fileName);
			PutStringUart(&uart1, textBuf);
			//printf("fil: %s", SDIO_files[i].fileName);
		}
		sprintf(textBuf, "\n");
		PutStringUart(&uart1, textBuf);
		//printf("\n\r");
	}

	//printf(CONSOLE_HIDE_CURSOR);
}
