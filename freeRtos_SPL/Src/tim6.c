#include "tim6.h"


// Настройка таймера
void timer6_init(void)
{
	/* Не забываем затактировать таймер */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    /* Таймер работает от частоты APB1 = 84 MHz
     * Инициализируем базовый таймер: делитель 42000, частота 2 кГц, период 500 мс, регистр сравнения 1000.
     * Другие параметры структуры TIM_TimeBaseInitTypeDef
     * не имеют смысла для базовых таймеров.
     */
    TIM_TimeBaseInitTypeDef base_timer;
    TIM_TimeBaseStructInit(&base_timer);
    /* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
    base_timer.TIM_Prescaler = 83;
    base_timer.TIM_Period = 999; // 1 мсек
    TIM_TimeBaseInit(TIM6, &base_timer);

    /* Разрешаем прерывание по обновлению (в данном случае -
     * по переполнению) счётчика таймера TIM6.
     */
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    /* Включаем таймер */
    TIM_Cmd(TIM6, ENABLE);

    /* Разрешаем обработку прерывания по переполнению счётчика
     * таймера TIM6. Так получилось, что это же прерывание
     * отвечает и за опустошение ЦАП.
     */
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

}
