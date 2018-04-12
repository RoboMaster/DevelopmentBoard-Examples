/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       sd_card.c
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
 
#include "bsp_driver_sd.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "sdio.h"
#include "sd_card.h"
#include "main.h"

FATFS          SDFatFs;                               /* file system object for SD card logical drive */
static uint8_t buffer[_MAX_SS];                       /* a work buffer for the f_mkfs() */
uint32_t       byteswritten, bytesread;               /* file write/read counts */
uint8_t        rtext[100];                            /* file read buffer */
uint8_t        err;
static char    *f_name = "robomaster.txt";           /* file name */
uint8_t        wtext[] = " Welcome to Robomaster! "; /* file write buffer */
extern UART_HandleTypeDef huart6;
/**
  * @brief  read ok indicator
  * @param  
  * @retval 
  */
static void read_ok(void)
{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

/**
  * @brief      show errors by blinking led
  * @param[in]  err: error id
  * @retval 
  */
static void led_blinking(uint8_t num)
{
	uint8_t i;
	for (i = 0; i < num; i++)
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
	}
	HAL_Delay(500);
}

/**
  * @brief      show errors by blinking led
  * @param[in]  err: error id
  * @retval 
  */
void error_detect(uint8_t err)
{
		switch (err)
		{
			case ERR_MOUNT_MKFS:
			{
				led_blinking(ERR_MOUNT_MKFS);
				HAL_UART_Transmit(&huart6, (uint8_t *)" Register file system error or create file system error!\n",  \
				                 (COUNTOF(" Register file system error or create file system error!\n") - 1), 55);
				HAL_Delay(10);	
			}break;

			case ERR_OPEN:
			{
				led_blinking(ERR_OPEN);
				HAL_UART_Transmit(&huart6, (uint8_t *)" Open file error!\n", (COUNTOF(" Open file error!\n") - 1), 55);
				HAL_Delay(10);	
			}break;
			
			default:
			{
				HAL_UART_Transmit(&huart6, (uint8_t *)" Write & Read ok, welcome to Robomaster!\n", \
				                 (COUNTOF(" Write & Read ok, welcome to Robomaster!\n") - 1), 55);
				HAL_Delay(10);	
			}break;
		}
}

/**
  * @brief  doing some tests to SD card, like mount, create file, open a text etc. 
  * @param  
  * @retval 
  */
void sd_test(void)
{   
	/* Register the file system object to the FatFs module and create a FAT file system (format) on the logical drive */
	if ((f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK) && (f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, buffer, sizeof(buffer)) == FR_OK))
	{

			/* Create and Open a new text file object with write access */
			if (f_open(&SDFile, f_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)  
			{
				/* Write data to the text file */
				f_write(&SDFile, wtext, sizeof(wtext), (void *)&byteswritten);  
				/* Close the open text file */
				f_close(&SDFile); 
				
				/* Open the text file object with read access */
				if (f_open(&SDFile, f_name, FA_READ) == FR_OK)  
				{
					/* Read data from the text file */
					f_read(&SDFile, rtext, sizeof(rtext), (UINT*)&bytesread);  
					read_ok();

					/* Close the open text file */
					f_close(&SDFile);  
				}
				else
				{
					err=ERR_OPEN;
				}
			}
			else
			{
				err=ERR_OPEN;
			}
	}
	else
	{
		err=ERR_MOUNT_MKFS;
	}
	/* Unlink the SD disk I/O driver */
	FATFS_UnLinkDriver(SDPath);  
}



