/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_otg.c
  * @brief   This file provides code for the configuration
  *          of the USB_OTG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usb_otg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Changed from HCD (Host) to PCD (Device) for USB Mass Storage Slave */
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USB_OTG_FS Device-mode init */
void MX_USB_OTG_FS_PCD_Init(void)
{
  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance                 = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints       = 4;
  hpcd_USB_OTG_FS.Init.speed               = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable          = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface          = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable          = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable    = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable          = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1   = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure USB OTG FS FIFO RAM (total 320 words = 1.25 KB on STM32F407).
     These MUST be set after HAL_PCD_Init which clears DIEPTXF[n] to zero.
     Without this the Bulk IN FIFO overlaps EP0, corrupting transfers → Code 43. */
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80U);   /* RX  : 128 words (512 B) */
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40U); /* EP0 TX:  64 words (256 B) */
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x80U); /* EP2 TX: 128 words (512 B) – MSC Bulk IN (USBD_MSC_EPIN_ADDR=0x82) */
  /* Total: 0x80 + 0x40 + 0x80 = 0x140 = 320 words  ✓ */

  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(pcdHandle->Instance == USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

  /* USER CODE END USB_OTG_FS_MspInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USB_OTG_FS GPIO Configuration
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USB_OTG_FS clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    /* Interrupt priority must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
       (typically 5-6) to be safe with ThreadX.                             */
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

  /* USER CODE END USB_OTG_FS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{
  if(pcdHandle->Instance == USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

  /* USER CODE END USB_OTG_FS_MspDeInit 0 */
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

  /* USER CODE END USB_OTG_FS_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
