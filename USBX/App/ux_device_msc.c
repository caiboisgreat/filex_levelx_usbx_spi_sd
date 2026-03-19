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
#define SD_MAX_RETRIES          3U      /* retry count for SD read/write */

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
static volatile UINT lun0_bootstrap_initialized = 0U;

#define LUN0_COMMIT_BATCH_RW     4U

/* Bitmap tracking which LBAs have a valid LevelX mapping.
   LevelX's lx_nor_flash_sector_read allocates a NEW physical sector for
   every read of an unmapped LBA (filling it with erased-flash garbage and
   wasting free sectors).  We guard against this by only calling sector_read
   for LBAs we know were successfully written via sector_write.
   32130 sectors / 8 bits = 4017 bytes.                                    */
#define LUN0_WRITTEN_BITMAP_WORDS  ((W25Q128_LX_USABLE_SECTORS + 31U) / 32U)
static ULONG  lun0_lx_written_bitmap[LUN0_WRITTEN_BITMAP_WORDS];

static void lun0_written_set(ULONG lba)
{
    if (lba < W25Q128_LX_USABLE_SECTORS)
        lun0_lx_written_bitmap[lba >> 5] |= (1UL << (lba & 31U));
}

static void lun0_written_clear(ULONG lba)
{
    if (lba < W25Q128_LX_USABLE_SECTORS)
        lun0_lx_written_bitmap[lba >> 5] &= ~(1UL << (lba & 31U));
}

static UINT lun0_written_test(ULONG lba)
{
    if (lba >= W25Q128_LX_USABLE_SECTORS)
        return 0U;
    return (lun0_lx_written_bitmap[lba >> 5] >> (lba & 31U)) & 1U;
}

static void ux_device_msc_lun0_bootstrap_init(void)
{
    if (lun0_bootstrap_initialized)
        return;
    memset(lun0_bootstrap_slot_index, LUN0_BOOTSTRAP_SLOT_NONE,
           sizeof(lun0_bootstrap_slot_index));
    lun0_bootstrap_initialized = 1U;
}

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
            /* Zero sectors don't need to be persisted in LevelX because the
               read path already returns zeroes for unmapped sectors.
               If there is a stale mapping for this LBA, release it.       */
            (void)lx_nor_flash_sector_release(&lx_nor_w25q128_flash, i);
            lun0_written_clear(i);
        }
        else if (lun0_bootstrap_state[i] == LUN0_BOOTSTRAP_STATE_DATA)
        {
            UCHAR slot = lun0_bootstrap_slot_index[i];
            if (slot == LUN0_BOOTSTRAP_SLOT_NONE)
            {
                /* inconsistent state: clear and move on */
                lun0_bootstrap_state[i] = LUN0_BOOTSTRAP_STATE_EMPTY;
                continue;
            }
            if (lx_nor_flash_sector_write(&lx_nor_w25q128_flash, i,
                                          ux_device_msc_lun0_slot_ptr(slot)) != LX_SUCCESS)
                continue;   /* skip on error, retry on next opportunity */
            lun0_written_set(i);
            ux_device_msc_lun0_release_slot(i);
        }
        lun0_bootstrap_state[i] = LUN0_BOOTSTRAP_STATE_EMPTY;
        committed++;
        if (committed >= max_sectors)
            break;
    }
    /* Bootstrap commit is always best-effort: a transient flash write failure
       must never propagate as a SCSI HARDWARE_ERROR to the Windows host.
       Skipped entries remain in the cache and are retried next time.        */
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
    ux_device_msc_lun0_bootstrap_init();
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
    ux_device_msc_lun0_bootstrap_init();
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
        UCHAR *sector_buf = data_pointer + (i * MSC_BLOCK_SIZE);
        ULONG  cur_lba = lba + i;

        /* Only call lx_nor_flash_sector_read for sectors that were
           previously written via sector_write.  LevelX's sector_read
           allocates a fresh physical sector for every read of an unmapped
           LBA (filling it with erased-flash 0xFF garbage and consuming a
           free sector).  Avoiding that prevents free-sector exhaustion
           which would cause subsequent sector_write to fail.            */
        if (lun0_written_test(cur_lba))
        {
            if (lx_nor_flash_sector_read(&lx_nor_w25q128_flash, cur_lba, sector_buf) != LX_SUCCESS)
                memset(sector_buf, 0, MSC_BLOCK_SIZE);
        }
        else
        {
            memset(sector_buf, 0, MSC_BLOCK_SIZE);
        }
        ux_device_msc_lun0_overlay_sector(cur_lba, sector_buf);
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
    ux_device_msc_lun0_bootstrap_init();
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
    /* LevelX ready: drain ALL cached metadata, then write real data. */
    (void)ux_device_msc_commit_lun0_bootstrap_cache(media_status, LUN0_BOOTSTRAP_CACHE_SECTORS);
    for (ULONG i = 0; i < number_blocks; i++)
    {
        UCHAR *src = data_pointer + (i * MSC_BLOCK_SIZE);
        ULONG  cur = lba + i;

        /* All-zero sector: release any existing LevelX mapping instead of
           wasting a physical sector.  The read path already returns zeroes
           for unmapped sectors, so nothing more is needed.                 */
        if (ux_device_msc_lun0_sector_is_zero(src) != 0U)
        {
            (void)lx_nor_flash_sector_release(&lx_nor_w25q128_flash, cur);
            lun0_written_clear(cur);
            if (cur < LUN0_BOOTSTRAP_CACHE_SECTORS)
            {
                if (lun0_bootstrap_state[cur] == LUN0_BOOTSTRAP_STATE_DATA)
                    ux_device_msc_lun0_release_slot(cur);
                lun0_bootstrap_state[cur] = LUN0_BOOTSTRAP_STATE_ZERO;
            }
        }
        else if (lx_nor_flash_sector_write(&lx_nor_w25q128_flash,
                                           cur, src) != LX_SUCCESS)
        {
            /* LevelX write failed (transient SPI error or no free sectors).
               Fall back to the bootstrap cache so the data survives until
               the next commit attempt.  For LBAs outside the cache window
               the data is silently accepted – returning HARDWARE_ERROR
               would cause Windows "format did not complete successfully". */
            if (cur < LUN0_BOOTSTRAP_CACHE_SECTORS)
                (void)ux_device_msc_lun0_cache_sector(cur, src);
            continue;
        }
        else
        {
            /* Successful LevelX write: mark LBA as written and clear cache */
            lun0_written_set(cur);
            if (cur < LUN0_BOOTSTRAP_CACHE_SECTORS)
            {
                if (lun0_bootstrap_state[cur] == LUN0_BOOTSTRAP_STATE_DATA)
                    ux_device_msc_lun0_release_slot(cur);
                lun0_bootstrap_state[cur] = LUN0_BOOTSTRAP_STATE_EMPTY;
            }
        }
    }
    *media_status = 0;
    return UX_SUCCESS;
}

/**
  * @brief  Flush callback for LUN 0 – commit all cached bootstrap sectors.
  *
  * Windows sends SYNCHRONIZE CACHE after completing a write sequence (e.g.
  * MBR initialisation, format).  If any bootstrap-cached sectors have not
  * yet been committed to LevelX flash, drain them now.
  */
UINT ux_device_msc_media_flush_lun0(VOID *storage, ULONG lun, ULONG number_blocks,
                                     ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun; (void)number_blocks; (void)lba;
    if (lx_nor_w25q128_ready)
        (void)ux_device_msc_commit_lun0_bootstrap_cache(media_status, LUN0_BOOTSTRAP_CACHE_SECTORS);
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

    /* Permanently disable SDIO-related interrupts.  CubeMX enables them in
       HAL_SD_MspInit / MX_DMA_Init, but they race with polling-mode
       HAL_SD_ReadBlocks / WriteBlocks and corrupt the HAL state machine.
       This is idempotent, so safe to call on every TEST UNIT READY.      */
    HAL_NVIC_DisableIRQ(SDIO_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);   /* SDIO RX DMA */
    HAL_NVIC_DisableIRQ(DMA2_Stream6_IRQn);   /* SDIO TX DMA */

    /* Reduce SDIO clock for safe polling under RTOS.  Init at ClockDiv=0
       (24 MHz) must succeed first; we slow it down here to ~6 MHz.
       At 24 MHz 4-bit the FIFO overruns if any ISR delays the polling
       loop by >5 us.  At 6 MHz the window is ~21 us — safe margin.
       Only touches CLKDIV[7:0], leaves all other CLKCR bits intact.   */
    MODIFY_REG(SDIO->CLKCR, SDIO_CLKCR_CLKDIV, 6U);  /* 48/(6+2) = 6 MHz */

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

    for (uint32_t attempt = 0; attempt < SD_MAX_RETRIES; attempt++)
    {
        /* Reset SDIO data-path & HAL state machine. */
        hsd.Instance->DCTRL = 0U;
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        hsd.State     = HAL_SD_STATE_READY;
        hsd.ErrorCode = HAL_SD_ERROR_NONE;
        hsd.Context   = SD_CONTEXT_NONE;

        HAL_StatusTypeDef hal_status = HAL_SD_ReadBlocks(
                &hsd, data_pointer, (uint32_t)lba,
                (uint32_t)number_blocks, SD_TRANSFER_TIMEOUT_MS);

        if (hal_status != HAL_OK)
        {
            /* Send CMD12 to abort any stuck multi-block transfer on card. */
            (void)SDMMC_CmdStopTransfer(hsd.Instance);
            __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
            tx_thread_sleep(10);   /* give card time to settle */
            continue;
        }

        /* Wait for card to return to TRANSFER state. */
        uint32_t tick = HAL_GetTick();
        UINT card_ok = 0;
        while ((HAL_GetTick() - tick) < SD_TRANSFER_TIMEOUT_MS)
        {
            if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
            {
                card_ok = 1;
                break;
            }
            tx_thread_sleep(1);
        }
        if (card_ok)
            return UX_SUCCESS;

        /* Card-state wait timed out — CMD12 + retry. */
        (void)SDMMC_CmdStopTransfer(hsd.Instance);
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        tx_thread_sleep(10);
    }

    *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
    return UX_ERROR;
}

/**
  * @brief  Write one or more 512-byte sectors to the SD card.
  */
UINT ux_device_msc_media_write_lun1(VOID *storage, ULONG lun, UCHAR *data_pointer,
                                     ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    (void)storage; (void)lun;
    *media_status = 0;

    for (uint32_t attempt = 0; attempt < SD_MAX_RETRIES; attempt++)
    {
        hsd.Instance->DCTRL = 0U;
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        hsd.State     = HAL_SD_STATE_READY;
        hsd.ErrorCode = HAL_SD_ERROR_NONE;
        hsd.Context   = SD_CONTEXT_NONE;

        HAL_StatusTypeDef hal_status = HAL_SD_WriteBlocks(
                &hsd, data_pointer, (uint32_t)lba,
                (uint32_t)number_blocks, SD_TRANSFER_TIMEOUT_MS);

        if (hal_status != HAL_OK)
        {
            (void)SDMMC_CmdStopTransfer(hsd.Instance);
            __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
            tx_thread_sleep(10);
            continue;
        }

        uint32_t tick = HAL_GetTick();
        UINT card_ok = 0;
        while ((HAL_GetTick() - tick) < SD_TRANSFER_TIMEOUT_MS)
        {
            if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
            {
                card_ok = 1;
                break;
            }
            tx_thread_sleep(1);
        }
        if (card_ok)
            return UX_SUCCESS;

        (void)SDMMC_CmdStopTransfer(hsd.Instance);
        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS);
        tx_thread_sleep(10);
    }

    *media_status = UX_SLAVE_CLASS_STORAGE_SENSE_KEY_HARDWARE_ERROR;
    return UX_ERROR;
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
