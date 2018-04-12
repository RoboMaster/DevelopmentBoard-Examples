/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_can.h
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-23-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
  
#ifndef _BSP__CAN_H
#define _BSP__CAN_H

#include "stm32f4xx_HAL.h"

#define UWB_CAN_RX_ID       (0x259)
#define UWB_RX_FIFO_SIZE    (22)
#define UWB_TX_FIFO_SIZE    (22)

typedef __packed struct
{
    int16_t coor_x;
    int16_t corr_y;
    uint16_t yaw;
    int16_t distance[6];
    uint16_t err_mask : 14;
    uint16_t sig_level : 2;
    uint16_t reserved;

}uwb_info_t;

extern uint8_t uwb_rx_data_buf[UWB_RX_FIFO_SIZE];
extern uwb_info_t               uwb_data;

HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan);


#endif
