/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_can.c
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-23-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "bsp_can.h"
#include "can.h"
#include "string.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t                  TxData[UWB_TX_FIFO_SIZE];
uint8_t                  RxData[UWB_RX_FIFO_SIZE];
uint8_t                  uwb_rx_data_buf[UWB_RX_FIFO_SIZE];
uwb_info_t               uwb_data;

/**
  * @brief  Configures the CAN, transmit and receive by polling
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
	if(hcan == &hcan1)
	{
	sFilterConfig.FilterBank = 0;
	}
	if(hcan == &hcan2)
	{
	sFilterConfig.FilterBank = 14;
	}
	
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

	return HAL_OK;
}


uint32_t can_rx_flag = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef   RxHeader;
	
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
	
	if (RxHeader.StdId != UWB_CAN_RX_ID)
	{
		uint8_t index = 0;
		static uint8_t *p = (uint8_t *)&uwb_rx_data_buf;
		uint8_t unpackstep = 0;
		
		if (RxHeader.DLC == 8)
		{
			for (index = 0; index < 8; index++)
			{
				*p++ = RxData[index];
			}
			unpackstep++;
		}
		else if(RxHeader.DLC == 6)
		{
			for (index = 0; index <6; index++)
			{
				*p++ = RxData[index];
			}

			p = (uint8_t *)&uwb_rx_data_buf;

			memcpy(&uwb_data, &uwb_rx_data_buf, sizeof(uwb_info_t));
			can_rx_flag ++;
		}
		else
		{
		//it maybe be some error occur
		}
	}
	
}






