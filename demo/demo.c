#include <stdbool.h>
#include <stdint.h>

#include "demo.h"
#include "main.h"
#include "stm32g4xx_hal.h"

extern FDCAN_HandleTypeDef fdcan;
extern TIM_HandleTypeDef htim6;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t cnt;
	if(htim == &htim6)
	{
		if(cnt > 100)
		{
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			cnt = 0;
		}
		cnt++;
	}
}

void Timer6UpdateCallback(void)
{

}

__NO_RETURN void app_main(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
    do
    {
        HAL_Delay(20);                              /* busy wait delay */
    } while (true);
}
