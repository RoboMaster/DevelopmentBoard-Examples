/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       sd_card.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the sd_card.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
 
#ifndef __SD_CARD_H__
#define __SD_CARD_H__

#include "sdio.h"
 
#define ERR_MOUNT_MKFS (1)
#define ERR_OPEN       (2)

extern uint8_t err;

void sd_test(void);
void error_detect(uint8_t err);

#endif

