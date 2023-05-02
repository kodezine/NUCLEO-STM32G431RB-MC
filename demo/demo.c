#include <stdbool.h>
#include <stdint.h>

#include "demo.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim6;
uint8_t RxData [64];
uint8_t TxData [64];
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t cnt;
	if(htim == &htim6)
	{
		if(cnt > 100)
		{
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			cnt = 0;
		}
		cnt++;
	}
}

void Timer6UpdateCallback(void)
{

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		/* Retrieve Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}
__NO_RETURN void app_main(void)
{

	FDCAN_FilterTypeDef sFilterConfig;
    HAL_TIM_Base_Start_IT(&htim6);
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterID1 = 0x111;
    sFilterConfig.FilterID2 = 0;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
    	Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
    	Error_Handler();
    }

    if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
    	Error_Handler();
    }

    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
    	Error_Handler();
    }

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    do
    {
        HAL_Delay(20);                              /* busy wait delay */
    } while (true);
}
