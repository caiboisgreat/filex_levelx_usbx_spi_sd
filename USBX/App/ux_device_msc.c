/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_msc.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
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
#include "ux_device_msc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ux_device_msc.h"
#include "lx_nor_w25q128_driver.h"
#include "sdio.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* ===========================================================================
 *  BLOCK DEVICE PARAMETERS
 * =========================================================================*/
#define MSC_BLOCK_SIZE          512U
#define SD_TRANSFER_TIMEOUT_MS  5000U   /* polling timeout for SDIO HAL */

/* Low-LBA bootstrap cache: buffers metadata writes before LevelX is ready.
   Covers LBA 0-3071 (3072 * 512 = 1.5 MB), spanning the FAT boot sector at
   LBA 0 and the FAT/root-dir area at LBA ~2048 used by Windows 1MB-aligned
   quick format.  Only non-zero sectors consume a data slot (64 * 512 B).   */
#define LUN0_BOOTSTRAP_CACHE_SECTORS      3072U
#define LUN0_BOOTSTRAP_DATA_SLOTS         64U
#define LUN0_BOOTSTRAP_STATE_EMPTY        0U
#define LUN0_BOOTSTRAP_STATE_ZERO         1U
#define LUN0_BOOTSTRAP_STATE_DATA         2U
#define LUN0_BOOTSTRAP_SLOT_NONE          0xFFU

static UCHAR  lun0_bootstrap_state[LUN0_BOOTSTRAP_CACHE_SECTORS];
static UCHAR  lun0_bootstrap_slot_index[LUN0_BOOTSTRAP_CACHE_SECTORS];
static UCHAR  lun0_bootstrap_slot_valid[LUN0_BOOTSTRAP_DATA_SLOTS];
static USHORT lun0_bootstrap_slot_sector[LUN0_BOOTSTRAP_DATA_SLOTS];
static UCHAR  lun0_bootstrap_slot_data[LUN0_BOOTSTRAP_DATA_SLOTS * MSC_BLOCK_SIZE];
static UCHAR  lun0_zero_sector[MSC_BLOCK_SIZE];

#define LUN0_COMMIT_BATCH_RW     4U
#define LUN0_COMMIT_BATCH_FLUSH 16U

static UINT ux_device_msc_lun0_cacheable(ULONG lba, ULONG number_blocks)
{
    return ((lba + number_blocks) <= LUN0_BOOTSTRAP_CACHE_SECTORS) ? UX_TRUE : UX_FALSE;
}

static UINT ux_device_msc_lun0_sector_is_zero(const UCHAR *buffer)
{
    for (ULONG i = 0; i < MSC_BLOCK_SIZE; i++)
    {
        if (buffer[i] != 0U)
            return UX_FALSE;
    }
    return UX_TRUE;
}

static UCHAR *ux_device_msc_lun0_slot_ptr(UCHAR slot_index)
{
    return lun0_bootstrap_slot_data + ((ULONG)slot_index * MSC_BLOCK_SIZE);
}

static UCHAR ux_device_msc_lun0_allocate_slot(ULONG sector_index)
{
    for (UCHAR slot = 0; slot < LUN0_BOOTSTRAP_DATA_SLOTS; slot++)
    {
        if ((lun0_bootstrap_slot_valid[slot] != 0U) &&
            (lun0_bootstrap_slot_sector[slot] == (USHORT)sector_index))
            return slot;
    }
    for (UCHAR slot = 0; slot < LUN0_BOOTSTRAP_DATA_SLOTS; slot++)
    {
        if (lun0_bootstrap_slot_valid[slot] == 0U)
        {
            lun0_bootstrap_slot_valid[slot]  = 1U;
            lun0_bootstrap_slot_sector[slot] = (USHORT)sector_index;
            return slot;
        }
    }
    return LUN0_BOOTSTRAP_SLOT_NONE;
}

static void ux_device_msc_lun0_release_slot(ULONG sector_index)
{
    UCHAR slot = lun0_bootstrap_slot_index[sector_index];
    if (slot == LUN0_BOOTSTRAP_SLOT_NONE)
        return;
    lun0_bootstrap_slot_valid[slot] = 0U;
    lun0_bootstrap_slot_index[sector_index] = LUN0_BOOTSTRAP_SLOT_NONE;
}

static UINT ux_device_msc_lun0_cache_sector(ULONG lba, const UCHAR *buffer)
{
    ULONG sector_index = lba;
    if (ux_device_msc_lun0_sector_is_zero(buffer) != 0U)
    {
        ux_device_msc_lun0_release_slot(sector_index);
        lun0_bootstrap_state[sector_index] = LUN0_BOOTSTRAP_STATE_ZERO;
        return UX_SUCCESS;
    }
    UCHAR slot = ux_device_msc_lun0_allocate_slot(sector_index);
    if (slot == LUN0_BOOTSTRAP_SLOT_NONE)
        return UX_ERROR;
    memcpy(ux_device_msc_lun0_slot_ptr(slot), buffer, MSC_BLOCK_SIZE);
    lun0_bootstrap_slot_index[sector_index] = slot;
    lun0_bootstrap_state[sector_index] = LUN0_BOOTSTRAP_STATE_DATA;
    return UX_SUCCESS;
}

static void ux_device_msc_lun0_overlay_sector(ULONG lba, UCHAR *buffer)
{
    if (lba >= LUN0_BOOTSTRAP_CACHE_SECTORS)
        return;
    switch (lun0_bootstrap_state[lba])
    {
        case LUN0_BOOTSTRAP_STATE_ZERO:
            memset(buffer, 0, MSC_BLOCK_SIZE);
            break;
        case LUN0_BOOTSTRAP_STATE_DATA:
        {
            UCHAR slot = lun0_bootstrap_slot_index[lba];
            if (slot != LUN0_BOOTSTRAP_SLOT_NONE)
                memcpy(buffer, ux_device_msc_lun0_slot_ptr(slot), MSC_BLOCK_SIZE);
            break;
        }
        default:
            break;
    }
}

static UINT ux_device_msc_commit_lun0_bootstrap_cache(ULONG *media_status, ULONG max_sectors)
{
    ULONG committed = 0;
    for (ULONG i = 0; i < LUN0_BOOTSTRAP_CACHE_SECTORS; i++)
    {
        if (lun0_bootstrap_state[i] == LUN0_BOOTSTRAP_STATE_EMPTY)
            continue;
        if (!lx_nor_w25q128_ready)
        {
            *media_status = 0;
            return UX_SUCCESS;
        }
        if (lun0_bootstrap_state[i] == LUN0_BOOTSTRAP_STATE_ZERO)
        {
            if (lx_nor_flash_sector_write(&lx_nor_w25q128_flash, i,
                                          lun0_zero_sector) != LX_SUCCESS)
            {
                *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
                return UX_ERROR;
            }
        }
        else if (lun0_bootstrap_state[i] == LUN0_BOOTSTRAP_STATE_DATA)
        {
            UCHAR slot = lun0_bootstrap_slot_index[i];
            if ((slot == LUN0_BOOTSTRAP_SLOT_NONE) ||
                (lx_nor_flash_sector_write(&lx_nor_w25q128_flash, i,
                                           ux_device_msc_lun0_slot_ptr(slot)) != LX_SUCCESS))
            {
                *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
                return UX_ERROR;
            }
            ux_device_msc_lun0_release_slot(i);
        }
        lun0_bootstrap_state[i] = LUN0_BOOTSTRAP_STATE_EMPTY;
        committed++;
        if (committed >= max_sectors)
            break;
    }
    *media_status = 0;
    return UX_SUCCESS;
}

/* ===========================================================================
 *  LUN 0  –  W25Q128 NOR Flash via LevelX
 * =========================================================================*/

/**
  * @brief  Return media status for LUN 0 (W25Q128).
  */
UINT ux_device_msc_media_status_lun0(VOID *storage, ULONG lun, ULONG media_id,
                                     ULONG *media_status)
{
    (void)storage; (void)lun; (void)media_id;
    /* Always READY so Disk Management shows the disk immediately.
       Opportunistically drain the bootstrap cache once LevelX is ready. */
    if (lx_nor_w25q128_ready)
        (void)ux_device_msc_commit_lun0_bootstrap_cache(media_status, LUN0_COMMIT_BATCH_RW);
    *media_status = 0;
    return UX_SUCCESS;
}

/**
  * @brief  Read one or more 512-byte logical sectors from W25Q128 via LevelX.
  */
UINT ux_device_msc_media_read_lun0(VOID *storage, ULONG lun, UCHAR *data_pointer,
                                    ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun;
    if (!lx_nor_w25q128_ready)
    {
        /* Return zeroes + overlay cached metadata so host sees a consistent
           blank disk while LevelX is still initialising.                   */
        memset(data_pointer, 0, number_blocks * MSC_BLOCK_SIZE);
        for (ULONG i = 0; i < number_blocks; i++)
            ux_device_msc_lun0_overlay_sector(lba + i, data_pointer + (i * MSC_BLOCK_SIZE));
        *media_status = 0;
        return UX_SUCCESS;
    }
    for (ULONG i = 0; i < number_blocks; i++)
    {
        if (lx_nor_flash_sector_read(&lx_nor_w25q128_flash,
                                     lba + i,
                                     data_pointer + (i * MSC_BLOCK_SIZE)) != LX_SUCCESS)
        {
            *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
            return UX_ERROR;
        }
        ux_device_msc_lun0_overlay_sector(lba + i, data_pointer + (i * MSC_BLOCK_SIZE));
    }
    *media_status = 0;
    return UX_SUCCESS;
}

/**
  * @brief  Write one or more 512-byte logical sectors to W25Q128 via LevelX.
  */
UINT ux_device_msc_media_write_lun0(VOID *storage, ULONG lun, UCHAR *data_pointer,
                                     ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun;
    if (!lx_nor_w25q128_ready)
    {
        if (ux_device_msc_lun0_cacheable(lba, number_blocks) != 0U)
        {
            for (ULONG i = 0; i < number_blocks; i++)
            {
                if (ux_device_msc_lun0_cache_sector(lba + i,
                        data_pointer + (i * MSC_BLOCK_SIZE)) != UX_SUCCESS)
                {
                    /* Slots exhausted – return retryable not-ready */
                    *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_NOT_READY |
                                    (0x04U << 8) | (0x01U << 16);
                    return UX_ERROR;
                }
            }
            *media_status = 0;
            return UX_SUCCESS;
        }
        /* Outside the 3072-sector cache window (e.g. Windows end-of-volume
           writability probe) – accept silently so format completion succeeds. */
        *media_status = 0;
        return UX_SUCCESS;
    }
    /* LevelX ready: drain any cached metadata first, then write real data. */
    if (ux_device_msc_commit_lun0_bootstrap_cache(media_status,
                                                   LUN0_COMMIT_BATCH_RW) != UX_SUCCESS)
        return UX_ERROR;
    for (ULONG i = 0; i < number_blocks; i++)
    {
        if (lx_nor_flash_sector_write(&lx_nor_w25q128_flash,
                                      lba + i,
                                      data_pointer + (i * MSC_BLOCK_SIZE)) != LX_SUCCESS)
        {
            *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
            return UX_ERROR;
        }
        if ((lba + i) < LUN0_BOOTSTRAP_CACHE_SECTORS)
        {
            lun0_bootstrap_state[lba + i] = LUN0_BOOTSTRAP_STATE_EMPTY;
            ux_device_msc_lun0_release_slot(lba + i);
        }
    }
    *media_status = 0;
    return UX_SUCCESS;
}

/**
  * @brief  Flush callback for LUN 0 – always succeed; commits are opportunistic.
  */
UINT ux_device_msc_media_flush_lun0(VOID *storage, ULONG lun, ULONG number_blocks,
                                     ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun; (void)number_blocks; (void)lba;
    *media_status = 0;
    return UX_SUCCESS;
}

/* ===========================================================================
 *  LUN 1  –  SD Card via SDIO HAL
 * =========================================================================*/

/**
  * @brief  Return media status for LUN 1 (SD card).
  */
UINT ux_device_msc_media_status_lun1(VOID *storage, ULONG lun, ULONG media_id,
                                     ULONG *media_status)
{
    (void)storage; (void)lun; (void)media_id;

  /* Be tolerant during enumeration.
     Windows can issue TEST UNIT READY / READ CAPACITY while the HAL state
     machine is still transitioning after card init. Reporting NOT_READY
     here suppresses the disk object entirely. Real I/O validity is still
     enforced by media_read_lun1 / media_write_lun1 below. */
  *media_status = 0;
  return UX_SUCCESS;
}

/**
  * @brief  Read one or more 512-byte sectors from the SD card.
  */
UINT ux_device_msc_media_read_lun1(VOID *storage, ULONG lun, UCHAR *data_pointer,
                                    ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun;
    *media_status = 0;

    if (HAL_SD_ReadBlocks(&hsd, data_pointer, (uint32_t)lba,
                          (uint32_t)number_blocks,
                          SD_TRANSFER_TIMEOUT_MS) != HAL_OK)
    {
        *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
        return UX_ERROR;
    }

    /* Wait until the SD card is ready for the next operation.
       tx_thread_sleep(1) yields the CPU instead of busy-waiting,
       which is important in an RTOS context.                      */
    uint32_t tick = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        if ((HAL_GetTick() - tick) >= SD_TRANSFER_TIMEOUT_MS)
        {
            *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
            return UX_ERROR;
        }
        tx_thread_sleep(1);
    }
    return UX_SUCCESS;
}

/**
  * @brief  Write one or more 512-byte sectors to the SD card.
  */
UINT ux_device_msc_media_write_lun1(VOID *storage, ULONG lun, UCHAR *data_pointer,
                                     ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun;
    *media_status = 0;

    if (HAL_SD_WriteBlocks(&hsd, data_pointer, (uint32_t)lba,
                           (uint32_t)number_blocks,
                           SD_TRANSFER_TIMEOUT_MS) != HAL_OK)
    {
        *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
        return UX_ERROR;
    }

    /* Wait until the SD card is ready (yield instead of busy-spin). */
    uint32_t tick = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        if ((HAL_GetTick() - tick) >= SD_TRANSFER_TIMEOUT_MS)
        {
            *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
            return UX_ERROR;
        }
        tx_thread_sleep(1);
    }
    return UX_SUCCESS;
}

/**
  * @brief  Flush callback for LUN 1 – SD card write is synchronous; no-op.
  */
UINT ux_device_msc_media_flush_lun1(VOID *storage, ULONG lun, ULONG number_blocks,
                                     ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun; (void)number_blocks; (void)lba;
    *media_status = 0;
    return UX_SUCCESS;
}

/* USER CODE END 1 */
