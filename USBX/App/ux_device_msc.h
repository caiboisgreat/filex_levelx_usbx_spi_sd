/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_msc.h
  * @author  MCD Application Team
  * @brief   USBX Device MSC header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UX_DEVICE_MSC_H__
#define __UX_DEVICE_MSC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ux_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
#include "ux_device_class_storage.h"
#include "sdio.h"
#include "lx_nor_w25q128_driver.h"

/* LUN 0 – W25Q128 NOR Flash (via LevelX) */
UINT ux_device_msc_media_read_lun0  (VOID *storage, ULONG lun, UCHAR *data_pointer,
                                     ULONG number_blocks, ULONG lba, ULONG *media_status);
UINT ux_device_msc_media_write_lun0 (VOID *storage, ULONG lun, UCHAR *data_pointer,
                                     ULONG number_blocks, ULONG lba, ULONG *media_status);
UINT ux_device_msc_media_status_lun0(VOID *storage, ULONG lun, ULONG media_id,
                                     ULONG *media_status);
UINT ux_device_msc_media_flush_lun0 (VOID *storage, ULONG lun, ULONG number_blocks,
                                     ULONG lba, ULONG *media_status);

/* LUN 1 – SD Card (via SDIO HAL) */
UINT ux_device_msc_media_read_lun1  (VOID *storage, ULONG lun, UCHAR *data_pointer,
                                     ULONG number_blocks, ULONG lba, ULONG *media_status);
UINT ux_device_msc_media_write_lun1 (VOID *storage, ULONG lun, UCHAR *data_pointer,
                                     ULONG number_blocks, ULONG lba, ULONG *media_status);
UINT ux_device_msc_media_status_lun1(VOID *storage, ULONG lun, ULONG media_id,
                                     ULONG *media_status);
UINT ux_device_msc_media_flush_lun1 (VOID *storage, ULONG lun, ULONG number_blocks,
                                     ULONG lba, ULONG *media_status);

/* Commit cached LUN0 bootstrap writes after LevelX open completes. */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif  /* __UX_DEVICE_MSC_H__ */
