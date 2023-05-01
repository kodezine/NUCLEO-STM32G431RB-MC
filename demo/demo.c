#include <stdbool.h>
#include <stdint.h>

#include "demo.h"
#include "main.h"
#include "stm32g4xx_hal.h"

extern FDCAN_HandleTypeDef fdcan;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

static volatile uint8_t RxBuffer[64];
static const uint8_t dot = '.';

void Timer6UpdateCallback(void)
{

}

__NO_RETURN void app_main(void)
{
    //HAL_TIM_Base_Start_IT(&htim6);
    HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ);
    do
    {
        //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); /* Toggle pin after some delay */
        HAL_Delay(20);                              /* busy wait delay */
    } while (true);
}
