/**
  ******************************************************************************
  * @file    lx_nor_w25q128_driver.c
  * @brief   LevelX NOR driver for W25Q128 (SPI1, 64KB block erase)
  *
  * LevelX passes a ULONG* "flash_address" to the read/write driver.
  * When lx_nor_flash_base_address = NULL (0), this pointer IS the
  * word-offset from the start of flash:
  *   byte_address = (uint32_t)flash_address * sizeof(ULONG)
  *                = (uint32_t)flash_address << 2
  *
  * The block_erase callback receives the block INDEX (0..255); byte address
  * is block * 64KB = block * 65536.
  ******************************************************************************
  */

#include "lx_nor_w25q128_driver.h"
#include "w25q128.h"

/* ---- LevelX NOR instance ------------------------------------------------ */
LX_NOR_FLASH lx_nor_w25q128_flash;

/* Ready flag – set to 1 by lx_nor_w25q128_open() on success */
volatile UINT lx_nor_w25q128_ready = 0U;

/* Sector buffer required by LevelX (LX_NOR_SECTOR_SIZE ULONGs = 512 bytes) */
static ULONG sector_buffer[LX_NOR_SECTOR_SIZE];

/* ---- Low-level driver callbacks ----------------------------------------- */

/**
  * @brief  Read 'words' ULONGs from the NOR flash into 'destination'.
  * @param  flash_address  Word-aligned offset pointer (base_address = NULL)
  * @param  destination    Output buffer
  * @param  words          Number of 32-bit words to read
  */
static UINT lx_nor_w25q128_driver_read(ULONG *flash_address,
                                        ULONG *destination,
                                        ULONG  words)
{
    uint32_t byte_addr = (uint32_t)((uintptr_t)flash_address) << 2;
    uint32_t byte_len  = words << 2;

    if (W25Q128_Read((uint8_t*)destination, byte_addr, byte_len) != HAL_OK)
        return LX_ERROR;

    return LX_SUCCESS;
}

/**
  * @brief  Write 'words' ULONGs from 'source' to the NOR flash.
  *         The target area must already be erased (0xFF).
  */
static UINT lx_nor_w25q128_driver_write(ULONG *flash_address,
                                         ULONG *source,
                                         ULONG  words)
{
    uint32_t byte_addr = (uint32_t)((uintptr_t)flash_address) << 2;
    uint32_t byte_len  = words << 2;

    if (W25Q128_Write((const uint8_t*)source, byte_addr, byte_len) != HAL_OK)
        return LX_ERROR;

    return LX_SUCCESS;
}

/**
  * @brief  Erase one 64KB block.
  * @param  block        Block index (0..255)
  * @param  erase_count  Passed by LevelX (not used here)
  */
static UINT lx_nor_w25q128_driver_block_erase(ULONG block, ULONG erase_count)
{
    (void)erase_count;
    uint32_t byte_addr = block * W25Q128_BLOCK_SIZE;   /* block * 65536 */

    if (W25Q128_EraseBlock64K(byte_addr) != HAL_OK)
        return LX_ERROR;

    return LX_SUCCESS;
}

/**
  * @brief  Verify that every byte in the block is 0xFF.
  */
static UINT lx_nor_w25q128_driver_block_erased_verify(ULONG block)
{
    uint32_t byte_addr = block * W25Q128_BLOCK_SIZE;
    uint8_t  temp[256];

    for (uint32_t off = 0; off < W25Q128_BLOCK_SIZE; off += sizeof(temp))
    {
        if (W25Q128_Read(temp, byte_addr + off, sizeof(temp)) != HAL_OK)
            return LX_ERROR;

        for (uint32_t i = 0; i < sizeof(temp); i++)
        {
            if (temp[i] != 0xFF)
                return LX_ERROR;
        }
    }
    return LX_SUCCESS;
}

/**
  * @brief  System error handler – called by LevelX on internal error.
  */
static UINT lx_nor_w25q128_driver_system_error(UINT error_code)
{
    (void)error_code;
    /* Non-fatal: return LX_SUCCESS so LevelX can attempt recovery */
    return LX_SUCCESS;
}

/* ---- Driver initialization entry --------------------------------------- */

/**
  * @brief  Called by lx_nor_flash_open() to configure the NOR instance.
  */
UINT lx_nor_w25q128_driver_initialize(LX_NOR_FLASH *nor_flash)
{
    /* Flash geometry */
    nor_flash->lx_nor_flash_total_blocks    = W25Q128_LX_BLOCK_COUNT;
    nor_flash->lx_nor_flash_words_per_block = W25Q128_LX_WORDS_PER_BLOCK;

    /* For SPI flash, use NULL base address.  LevelX passes word-offset
       pointers to the read/write drivers (see comments at top).          */
    nor_flash->lx_nor_flash_base_address = (ULONG*)NULL;

    /* Sector buffer (must be LX_NOR_SECTOR_SIZE ULONGs = 512 bytes) */
    nor_flash->lx_nor_flash_sector_buffer = sector_buffer;

    /* Driver callbacks */
    nor_flash->lx_nor_flash_driver_read               = lx_nor_w25q128_driver_read;
    nor_flash->lx_nor_flash_driver_write              = lx_nor_w25q128_driver_write;
    nor_flash->lx_nor_flash_driver_block_erase        = lx_nor_w25q128_driver_block_erase;
    nor_flash->lx_nor_flash_driver_block_erased_verify= lx_nor_w25q128_driver_block_erased_verify;
    nor_flash->lx_nor_flash_driver_system_error       = lx_nor_w25q128_driver_system_error;

    return LX_SUCCESS;
}

/* ---- Open / Close ------------------------------------------------------- */

/**
  * @brief  Initialize W25Q128 hardware and open the LevelX NOR instance.
  *
  * Call once from application init (before USBX MSC callbacks are invoked).
  */
UINT lx_nor_w25q128_open(void)
{
    UINT status;

    /* Hardware init */
    if (W25Q128_Init() != HAL_OK)
        return LX_ERROR;

    /* Open LevelX NOR instance (may take time on first-time format) */
    status = lx_nor_flash_open(&lx_nor_w25q128_flash,
                               "W25Q128",
                               lx_nor_w25q128_driver_initialize);

    return status;
}

UINT lx_nor_w25q128_close(void)
{
    return lx_nor_flash_close(&lx_nor_w25q128_flash);
}
