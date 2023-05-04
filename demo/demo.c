#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "demo.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "mc_api.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim6;

/*
 * Data and Header for HCAN traffic
 */
uint8_t RxData [64];
uint8_t TxData [64];
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;

/*
 * Some motor controlling type defines
 */
typedef int16_t rpm;
typedef union _rpm_i16CAN
{
	uint8_t bytes[2];
	rpm rpm_i16CAN;
}t_rpm_i16CAN;
t_rpm_i16CAN setRPM;
/*
 * SAFE Value for RPM is Zero (no drive movement)
 */
const rpm SAFE_RPM = 0;

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
		do
		{
			if(RxHeader.Identifier != 0x111)
			{
				break;
			}
			if(RxHeader.DataLength != FDCAN_DLC_BYTES_2)
			{
				break;
			}
			memcpy(&setRPM.bytes[0], &RxData[0], sizeof(t_rpm_i16CAN));
		}while (false);
	}
}
__NO_RETURN void app_main(void)
{
	/* FDCan control logic for CANOpen compatiblity : No hardware filters, software filters only */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            0 | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
            0xFFFFFFFF) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
    	Error_Handler();
    }

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim6);
	do
    {
		rpm setRPM_i16Drive = SAFE_RPM;
		bool maxRPM_OK = false;
		bool minRPM_OK = false;
		if(setRPM.rpm_i16CAN < 15000 && setRPM.rpm_i16CAN > -15000)
		{
			maxRPM_OK = true;
		}
		if(setRPM.rpm_i16CAN > 3000 || setRPM.rpm_i16CAN < -3000)
		{
			minRPM_OK = true;
		}
		if(minRPM_OK && maxRPM_OK)
		{
			setRPM_i16Drive = setRPM.rpm_i16CAN;
		}
		/**
		 * Write a code to drive the motor with drive speed in integer
		 */
#if 0
		if(setRPM_i16Drive != SAFE_RPM)
		{
			MC_ProgramSpeedRampMotor1((int16_t)setRPM_i16Drive, 0);
			MC_StartMotor1();
		}
		else
		{
			MC_StopMotor1();
		}
#endif
        HAL_Delay(10);                              /* busy wait delay */
    } while (true);
}
