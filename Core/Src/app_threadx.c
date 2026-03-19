/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
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

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lx_nor_w25q128_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static TX_THREAD lx_init_thread;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void lx_nor_init_thread_entry(ULONG arg);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
  CHAR *pointer;

  if (tx_byte_allocate(byte_pool, (VOID **)&pointer, 2048, TX_NO_WAIT) != TX_SUCCESS)
    return TX_POOL_ERROR;

  ret = tx_thread_create(&lx_init_thread,
                         "LevelX NOR Init",
                         lx_nor_init_thread_entry,
                         0,
                         pointer,
                         2048,
                         25,
                         25,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

static void lx_nor_init_thread_entry(ULONG arg)
{
  (void)arg;
  if (lx_nor_w25q128_open() == LX_SUCCESS)
  {
    /* Run a defragment pass to free any reclaimable physical sectors.
       This ensures that subsequent sector_write calls from the USB MSC
       callbacks complete quickly without triggering expensive inline
       garbage-collection during a SCSI WRITE command.                 */
    (void)lx_nor_flash_defragment(&lx_nor_w25q128_flash);
    lx_nor_w25q128_ready = 1U;
  }
}

/* USER CODE END 1 */
