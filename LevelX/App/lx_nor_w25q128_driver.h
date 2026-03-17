/**
  ******************************************************************************
  * @file    lx_nor_w25q128_driver.h
  * @brief   LevelX NOR driver for W25Q128 via SPI1
  *
  * Flash geometry used by LevelX (64KB erase blocks):
  *   total_blocks      = 256
  *   words_per_block   = 16384  (64KB / 4 bytes)
  *
  * After lx_nor_flash_open() LevelX computes:
  *   physical_sectors_per_block = 126
  *   total_physical_sectors     = 32256
  *
  * For USB MSC we expose:
  *   LUN 0 last_lba = W25Q128_LX_USABLE_SECTORS - 1
  *   (one block reserved for GC)
  ******************************************************************************
  */
#ifndef __LX_NOR_W25Q128_DRIVER_H__
#define __LX_NOR_W25Q128_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lx_api.h"

/* LevelX NOR instance – visible to ux_device_msc.c for sector read/write */
extern LX_NOR_FLASH lx_nor_w25q128_flash;

/* Set to 1 once lx_nor_w25q128_open() has successfully completed.
   Read by the USB MSC status callback to report NOT_READY while
   the background LevelX scan/format is still in progress.         */
extern volatile UINT lx_nor_w25q128_ready;

/* Total usable logical sectors (reserve 1 block for garbage-collection) */
#define W25Q128_LX_BLOCK_COUNT          256U
#define W25Q128_LX_WORDS_PER_BLOCK      16384U          /* 64KB / 4 */
/*   physical_sectors_per_block = 126 (computed by LevelX open)       */
#define W25Q128_LX_PHYS_SECTORS_PER_BLK 126U
#define W25Q128_LX_TOTAL_PHYS_SECTORS   (W25Q128_LX_BLOCK_COUNT * W25Q128_LX_PHYS_SECTORS_PER_BLK)
/* Reserve 1 block (126 sectors) for internal GC                       */
#define W25Q128_LX_USABLE_SECTORS       (W25Q128_LX_TOTAL_PHYS_SECTORS - W25Q128_LX_PHYS_SECTORS_PER_BLK)

/* LevelX driver init entry (passed to lx_nor_flash_open) */
UINT lx_nor_w25q128_driver_initialize(LX_NOR_FLASH *nor_flash);

/* Open / close helpers */
UINT lx_nor_w25q128_open(void);
UINT lx_nor_w25q128_close(void);

#ifdef __cplusplus
}
#endif

#endif /* __LX_NOR_W25Q128_DRIVER_H__ */
