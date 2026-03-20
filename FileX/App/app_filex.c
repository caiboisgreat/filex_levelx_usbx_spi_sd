/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_filex.c
  * @author  MCD Application Team
  * @brief   FileX applicative file
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
#include "app_filex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "lx_nor_w25q128_driver.h"
#include "sdio.h"
#include "ux_device_msc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FX_THREAD_STACK_SIZE      2048U
#define FX_MEDIA_BUFFER_SIZE      512U

/* NOR Flash format parameters (SFD style, no MBR) */
#define FLASH_VOLUME_NAME         "FLASH"
#define FLASH_NUM_FATS            1U
#define FLASH_DIR_ENTRIES         64U
#define FLASH_HIDDEN_SECTORS      0U
#define FLASH_BYTES_PER_SECTOR    512U
#define FLASH_SECTORS_PER_CLUSTER 4U
#define FLASH_HEADS               1U
#define FLASH_SECTORS_PER_TRACK   1U

/* SD Card format parameters */
#define SD_VOLUME_NAME            "SD_CARD"
#define SD_NUM_FATS               1U
#define SD_DIR_ENTRIES            512U
#define SD_HIDDEN_SECTORS         0U
#define SD_BYTES_PER_SECTOR       512U
#define SD_HEADS                  1U
#define SD_SECTORS_PER_TRACK      1U

#define SD_TRANSFER_TIMEOUT_MS    5000U
#define SD_MAX_RETRIES            3U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* FileX media instances */
FX_MEDIA  fx_flash_media;
FX_MEDIA  fx_sd_media;

/* Media ready flags */
volatile UINT fx_flash_media_ready = 0U;
volatile UINT fx_sd_media_ready    = 0U;
volatile UINT fx_filesystems_ready = 0U;

/* Media working buffers (must be ULONG-aligned for LevelX) */
static ULONG flash_media_buf[FX_MEDIA_BUFFER_SIZE / sizeof(ULONG)];
static ULONG sd_media_buf[FX_MEDIA_BUFFER_SIZE / sizeof(ULONG)];

/* Format work buffer (shared, used only during format) */
static ULONG format_work_buf[FX_MEDIA_BUFFER_SIZE / sizeof(ULONG)];

/* Auto-mount thread */
static TX_THREAD fx_init_thread;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void fx_init_thread_entry(ULONG arg);
static void fx_flash_driver(FX_MEDIA *media_ptr);
static void fx_sd_driver(FX_MEDIA *media_ptr);

UINT _fx_partition_offset_calculate(void *partition_sector, UINT partition,
                                    ULONG *partition_start, ULONG *partition_size);
/* USER CODE END PFP */

/**
  * @brief  Application FileX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_FileX_Init(VOID *memory_ptr)
{
  UINT ret = FX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_FileX_MEM_POOL */

  /* USER CODE END MX_FileX_MEM_POOL */

  /* USER CODE BEGIN MX_FileX_Init */
  CHAR *pointer;

  /* Initialize FileX */
  fx_system_initialize();

  /* Allocate stack for the auto-mount thread */
  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       FX_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    return FX_NOT_ENOUGH_MEMORY;

  /* Create the FileX auto-mount thread (lower priority than LevelX init) */
  ret = (UINT)tx_thread_create(&fx_init_thread,
                               "FileX AutoMount",
                               fx_init_thread_entry,
                               0,
                               pointer,
                               FX_THREAD_STACK_SIZE,
                               24, 24,
                               TX_NO_TIME_SLICE,
                               TX_AUTO_START);
  /* USER CODE END MX_FileX_Init */
  return ret;
}

/* USER CODE BEGIN 1 */

/* ===========================================================================
 *  FileX media driver for W25Q128 NOR Flash via LevelX
 * =========================================================================*/

static void fx_flash_driver(FX_MEDIA *media_ptr)
{
    UINT  status;
    ULONG logical_sector;
    UCHAR *buffer;

    switch (media_ptr->fx_media_driver_request)
    {
    case FX_DRIVER_INIT:
        media_ptr->fx_media_driver_free_sector_update = FX_TRUE;
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_UNINIT:
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_READ:
        logical_sector = media_ptr->fx_media_driver_logical_sector;
        buffer = (UCHAR *)media_ptr->fx_media_driver_buffer;
        for (ULONG i = 0; i < media_ptr->fx_media_driver_sectors; i++)
        {
            status = lx_nor_flash_sector_read(&lx_nor_w25q128_flash,
                                              logical_sector + i,
                                              buffer + (i * FLASH_BYTES_PER_SECTOR));
            if (status != LX_SUCCESS)
            {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                return;
            }
        }
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_WRITE:
        logical_sector = media_ptr->fx_media_driver_logical_sector;
        buffer = (UCHAR *)media_ptr->fx_media_driver_buffer;
        for (ULONG i = 0; i < media_ptr->fx_media_driver_sectors; i++)
        {
            status = lx_nor_flash_sector_write(&lx_nor_w25q128_flash,
                                               logical_sector + i,
                                               buffer + (i * FLASH_BYTES_PER_SECTOR));
            if (status != LX_SUCCESS)
            {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                return;
            }
            ux_device_msc_lun0_notify_sector_written(logical_sector + i);
        }
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_BOOT_READ:
        buffer = (UCHAR *)media_ptr->fx_media_driver_buffer;
        status = lx_nor_flash_sector_read(&lx_nor_w25q128_flash, 0, buffer);
        if (status != LX_SUCCESS)
        {
            media_ptr->fx_media_driver_status = FX_IO_ERROR;
            return;
        }
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_BOOT_WRITE:
        buffer = (UCHAR *)media_ptr->fx_media_driver_buffer;
        status = lx_nor_flash_sector_write(&lx_nor_w25q128_flash, 0, buffer);
        if (status == LX_SUCCESS)
        {
            ux_device_msc_lun0_notify_sector_written(0);
            media_ptr->fx_media_driver_status = FX_SUCCESS;
        }
        else
        {
            media_ptr->fx_media_driver_status = FX_IO_ERROR;
        }
        break;

    case FX_DRIVER_RELEASE_SECTORS:
        logical_sector = media_ptr->fx_media_driver_logical_sector;
        for (ULONG i = 0; i < media_ptr->fx_media_driver_sectors; i++)
        {
            lx_nor_flash_sector_release(&lx_nor_w25q128_flash,
                                        logical_sector + i);
            ux_device_msc_lun0_notify_sector_released(logical_sector + i);
        }
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_FLUSH:
    case FX_DRIVER_ABORT:
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    default:
        media_ptr->fx_media_driver_status = FX_IO_ERROR;
        break;
    }
}

/* ===========================================================================
 *  FileX media driver for SD Card via SDIO HAL (polling mode)
 * =========================================================================*/

static UINT fx_sd_read_sectors(UCHAR *dest, ULONG sector, UINT count)
{
    for (uint32_t attempt = 0; attempt < SD_MAX_RETRIES; attempt++)
    {
        hsd.Instance->DCTRL = 0U;
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        hsd.State     = HAL_SD_STATE_READY;
        hsd.ErrorCode = HAL_SD_ERROR_NONE;
        hsd.Context   = SD_CONTEXT_NONE;

        if (HAL_SD_ReadBlocks(&hsd, dest, sector, count,
                              SD_TRANSFER_TIMEOUT_MS) != HAL_OK)
        {
            (void)SDMMC_CmdStopTransfer(hsd.Instance);
            __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
            tx_thread_sleep(10);
            continue;
        }
        uint32_t tick = HAL_GetTick();
        while ((HAL_GetTick() - tick) < SD_TRANSFER_TIMEOUT_MS)
        {
            if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
                return FX_SUCCESS;
            tx_thread_sleep(1);
        }
        (void)SDMMC_CmdStopTransfer(hsd.Instance);
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        tx_thread_sleep(10);
    }
    return FX_IO_ERROR;
}

static UINT fx_sd_write_sectors(UCHAR *src, ULONG sector, UINT count)
{
    for (uint32_t attempt = 0; attempt < SD_MAX_RETRIES; attempt++)
    {
        hsd.Instance->DCTRL = 0U;
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        hsd.State     = HAL_SD_STATE_READY;
        hsd.ErrorCode = HAL_SD_ERROR_NONE;
        hsd.Context   = SD_CONTEXT_NONE;

        if (HAL_SD_WriteBlocks(&hsd, src, sector, count,
                               SD_TRANSFER_TIMEOUT_MS) != HAL_OK)
        {
            (void)SDMMC_CmdStopTransfer(hsd.Instance);
            __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
            tx_thread_sleep(10);
            continue;
        }
        uint32_t tick = HAL_GetTick();
        while ((HAL_GetTick() - tick) < SD_TRANSFER_TIMEOUT_MS)
        {
            if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
                return FX_SUCCESS;
            tx_thread_sleep(1);
        }
        (void)SDMMC_CmdStopTransfer(hsd.Instance);
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        tx_thread_sleep(10);
    }
    return FX_IO_ERROR;
}

static void fx_sd_driver(FX_MEDIA *media_ptr)
{
    UINT  status;
    ULONG partition_start;
    ULONG partition_size;

    switch (media_ptr->fx_media_driver_request)
    {
    case FX_DRIVER_INIT:
        /* Disable DMA interrupts – use polling mode only (same as MSC) */
        HAL_NVIC_DisableIRQ(SDIO_IRQn);
        HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
        HAL_NVIC_DisableIRQ(DMA2_Stream6_IRQn);
        /* Slow clock for safe polling: 48/(6+2) = 6 MHz */
        MODIFY_REG(SDIO->CLKCR, SDIO_CLKCR_CLKDIV, 6U);
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_UNINIT:
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_READ:
        status = fx_sd_read_sectors(
                     (UCHAR *)media_ptr->fx_media_driver_buffer,
                     media_ptr->fx_media_driver_logical_sector +
                         media_ptr->fx_media_hidden_sectors,
                     media_ptr->fx_media_driver_sectors);
        media_ptr->fx_media_driver_status = status;
        break;

    case FX_DRIVER_WRITE:
        status = fx_sd_write_sectors(
                     (UCHAR *)media_ptr->fx_media_driver_buffer,
                     media_ptr->fx_media_driver_logical_sector +
                         media_ptr->fx_media_hidden_sectors,
                     media_ptr->fx_media_driver_sectors);
        media_ptr->fx_media_driver_status = status;
        break;

    case FX_DRIVER_BOOT_READ:
        /* Read sector 0 */
        status = fx_sd_read_sectors(
                     (UCHAR *)media_ptr->fx_media_driver_buffer, 0, 1);
        if (status != FX_SUCCESS)
        {
            media_ptr->fx_media_driver_status = FX_IO_ERROR;
            break;
        }
        /* Check for MBR – get actual boot sector offset if partitioned */
        partition_start = 0;
        status = _fx_partition_offset_calculate(
                     media_ptr->fx_media_driver_buffer, 0,
                     &partition_start, &partition_size);
        if ((status == FX_SUCCESS) && (partition_start > 0))
        {
            status = fx_sd_read_sectors(
                         (UCHAR *)media_ptr->fx_media_driver_buffer,
                         partition_start, 1);
            if (status != FX_SUCCESS)
            {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }
            media_ptr->fx_media_hidden_sectors = partition_start;
        }
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    case FX_DRIVER_BOOT_WRITE:
        status = fx_sd_write_sectors(
                     (UCHAR *)media_ptr->fx_media_driver_buffer,
                     media_ptr->fx_media_hidden_sectors, 1);
        media_ptr->fx_media_driver_status =
            (status == FX_SUCCESS) ? FX_SUCCESS : FX_IO_ERROR;
        break;

    case FX_DRIVER_FLUSH:
    case FX_DRIVER_ABORT:
    case FX_DRIVER_RELEASE_SECTORS:
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;

    default:
        media_ptr->fx_media_driver_status = FX_IO_ERROR;
        break;
    }
}

/* ===========================================================================
 *  Auto-mount thread: detect filesystem, format if absent, then mount
 * =========================================================================*/

static void fx_init_thread_entry(ULONG arg)
{
    (void)arg;
    UINT status;

    /* ------------------------------------------------------------------ */
    /*                    NOR Flash (LUN 0)                                */
    /* ------------------------------------------------------------------ */

    /* Wait for LevelX to finish initialisation (timeout after 60 s) */
    {
        UINT wait_count = 0;
        while (!lx_nor_w25q128_ready && (wait_count < 600U))
        {
            tx_thread_sleep(10);   /* 100 ms */
            wait_count++;
        }
    }

    if (!lx_nor_w25q128_ready)
        goto skip_flash;       /* LevelX init failed – skip NOR Flash */

    /* Try to mount the Flash filesystem */
    status = fx_media_open(&fx_flash_media, "NOR Flash",
                           fx_flash_driver, NULL,
                           flash_media_buf, sizeof(flash_media_buf));

    if ((status == FX_BOOT_ERROR) || (status == FX_MEDIA_INVALID) ||
        (status == FX_BUFFER_ERROR))
    {
        /* No valid filesystem – format as FAT (SFD, no MBR) */
        status = fx_media_format(&fx_flash_media,
                                 fx_flash_driver,       /* driver         */
                                 NULL,                   /* driver_info    */
                                 (UCHAR *)format_work_buf,
                                 sizeof(format_work_buf),
                                 FLASH_VOLUME_NAME,
                                 FLASH_NUM_FATS,
                                 FLASH_DIR_ENTRIES,
                                 FLASH_HIDDEN_SECTORS,
                                 W25Q128_LX_USABLE_SECTORS,
                                 FLASH_BYTES_PER_SECTOR,
                                 FLASH_SECTORS_PER_CLUSTER,
                                 FLASH_HEADS,
                                 FLASH_SECTORS_PER_TRACK);

        if (status == FX_SUCCESS)
        {
            /* Format succeeded – mount again */
            status = fx_media_open(&fx_flash_media, "NOR Flash",
                                   fx_flash_driver, NULL,
                                   flash_media_buf, sizeof(flash_media_buf));
        }
    }

    if (status == FX_SUCCESS)
    {
        /* Close the media so MSC has exclusive LevelX access –
           concurrent FileX + MSC on the same flash causes corruption. */
        fx_media_close(&fx_flash_media);

        /* Rebuild the MSC written-bitmap from LevelX metadata.
           On reboot the bitmap is BSS-zero, so without this step MSC
           reads return all-zeros for sectors written in prior boots.  */
        ux_device_msc_lun0_rebuild_written_bitmap();

        fx_flash_media_ready = 1U;
    }

skip_flash:
    ;   /* empty statement required before declaration after label */
    /* ------------------------------------------------------------------ */
    /*                    SD Card (LUN 1)                                  */
    /* ------------------------------------------------------------------ */

    /* Get card geometry */
    HAL_SD_CardInfoTypeDef card_info = {0};
    HAL_SD_GetCardInfo(&hsd, &card_info);

    if (card_info.LogBlockNbr > 0)
    {
        ULONG sd_total_sectors = card_info.LogBlockNbr;

        /* Choose sectors_per_cluster based on card capacity */
        UINT sd_spc;
        if (sd_total_sectors <= 32768U)          /* <= 16 MB  */
            sd_spc = 2U;
        else if (sd_total_sectors <= 262144U)    /* <= 128 MB */
            sd_spc = 4U;
        else if (sd_total_sectors <= 4194304U)   /* <= 2 GB   */
            sd_spc = 8U;
        else                                     /* > 2 GB    */
            sd_spc = 64U;

        /* Try to mount the SD filesystem */
        status = fx_media_open(&fx_sd_media, "SD Card",
                               fx_sd_driver, NULL,
                               sd_media_buf, sizeof(sd_media_buf));

        if ((status == FX_BOOT_ERROR) || (status == FX_MEDIA_INVALID) ||
            (status == FX_BUFFER_ERROR))
        {
            /* No valid filesystem – format */
            status = fx_media_format(&fx_sd_media,
                                     fx_sd_driver,
                                     NULL,
                                     (UCHAR *)format_work_buf,
                                     sizeof(format_work_buf),
                                     SD_VOLUME_NAME,
                                     SD_NUM_FATS,
                                     SD_DIR_ENTRIES,
                                     SD_HIDDEN_SECTORS,
                                     sd_total_sectors,
                                     SD_BYTES_PER_SECTOR,
                                     sd_spc,
                                     SD_HEADS,
                                     SD_SECTORS_PER_TRACK);

            if (status == FX_SUCCESS)
            {
                status = fx_media_open(&fx_sd_media, "SD Card",
                                       fx_sd_driver, NULL,
                                       sd_media_buf, sizeof(sd_media_buf));
            }
        }

        if (status == FX_SUCCESS)
        {
            fx_media_close(&fx_sd_media);
            fx_sd_media_ready = 1U;
        }
    }

    /* Both filesystems processed – signal USB thread to start enumeration */
    fx_filesystems_ready = 1U;

    /* Thread completed – suspend forever */
    tx_thread_suspend(tx_thread_identify());
}

/* USER CODE END 1 */
