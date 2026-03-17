/**
  ******************************************************************************
  * @file    w25q128.c
  * @brief   W25Q128 SPI NOR Flash driver
  *
  * All SPI transfers use hspi1 (SPI1) in polling mode.
  * PB14 is used as software-controlled chip-select.
  ******************************************************************************
  */

#include "w25q128.h"
#include <string.h>

/* Timeout for HAL_SPI_Transmit/Receive polling calls */
#define SPI_TIMEOUT_MS   100U

/* ---- Private helpers ---------------------------------------------------- */

static HAL_StatusTypeDef spi_transmit(const uint8_t *pData, uint16_t size)
{
    return HAL_SPI_Transmit(&hspi1, (uint8_t*)pData, size, SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef spi_receive(uint8_t *pData, uint16_t size)
{
    return HAL_SPI_Receive(&hspi1, pData, size, SPI_TIMEOUT_MS);
}

static void write_enable(void)
{
    uint8_t cmd = W25Q128_CMD_WRITE_ENABLE;
    W25Q128_CS_LOW();
    spi_transmit(&cmd, 1);
    W25Q128_CS_HIGH();
}

/* ---- Public API --------------------------------------------------------- */

/**
  * @brief  Initialize the W25Q128 driver.
  *         Releases power-down and verifies JEDEC ID.
  */
HAL_StatusTypeDef W25Q128_Init(void)
{
    uint8_t cmd = W25Q128_CMD_RELEASE_POWER_DOWN;
    W25Q128_CS_LOW();
    spi_transmit(&cmd, 1);
    W25Q128_CS_HIGH();
    HAL_Delay(1);   /* tRES1 = 3 us max, 1 ms is more than sufficient */

    uint32_t id = W25Q128_ReadID();
    /* Winbond W25Q128 JEDEC ID: Mfr=0xEF, Type=0x40, Capacity=0x18 */
    if ((id >> 16) != 0xEF)
        return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief  Read 24-bit JEDEC ID: [Manufacturer | Memory Type | Capacity]
  */
uint32_t W25Q128_ReadID(void)
{
    uint8_t cmd = W25Q128_CMD_READ_JEDEC_ID;
    uint8_t id[3] = {0};
    W25Q128_CS_LOW();
    spi_transmit(&cmd, 1);
    spi_receive(id, 3);
    W25Q128_CS_HIGH();
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

/**
  * @brief  Read the Status Register 1.
  */
uint8_t W25Q128_ReadStatusReg1(void)
{
    uint8_t cmd = W25Q128_CMD_READ_SR1;
    uint8_t sr  = 0;
    W25Q128_CS_LOW();
    spi_transmit(&cmd, 1);
    spi_receive(&sr, 1);
    W25Q128_CS_HIGH();
    return sr;
}

/**
  * @brief  Poll BUSY bit until clear or timeout.
  */
HAL_StatusTypeDef W25Q128_WaitForReady(uint32_t timeout_ms)
{
    uint32_t tick = HAL_GetTick();
    while (W25Q128_ReadStatusReg1() & W25Q128_SR1_BUSY)
    {
        if ((HAL_GetTick() - tick) >= timeout_ms)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

/**
  * @brief  Read arbitrary bytes from flash.
  * @param  pData    destination buffer
  * @param  addr     24-bit byte address
  * @param  len      number of bytes to read
  */
HAL_StatusTypeDef W25Q128_Read(uint8_t *pData, uint32_t addr, uint32_t len)
{
    if (len == 0) return HAL_OK;
    if (W25Q128_WaitForReady(W25Q128_WRITE_TIMEOUT_MS) != HAL_OK)
        return HAL_TIMEOUT;

    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_READ_DATA;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >>  8);
    cmd[3] = (uint8_t)(addr);

    W25Q128_CS_LOW();
    spi_transmit(cmd, 4);
    HAL_StatusTypeDef ret = spi_receive(pData, (uint16_t)len);
    W25Q128_CS_HIGH();
    return ret;
}

/**
  * @brief  Write arbitrary bytes to flash (handles page boundaries).
  *         Destination area must already be erased (0xFF).
  * @param  pData    source buffer
  * @param  addr     24-bit byte address
  * @param  len      number of bytes to write
  */
HAL_StatusTypeDef W25Q128_Write(const uint8_t *pData, uint32_t addr, uint32_t len)
{
    if (len == 0) return HAL_OK;

    const uint8_t *src = pData;
    uint32_t       cur_addr = addr;
    uint32_t       rem = len;

    while (rem > 0)
    {
        /* How many bytes fit in the current page? */
        uint32_t page_off  = cur_addr % W25Q128_PAGE_SIZE;
        uint32_t page_room = W25Q128_PAGE_SIZE - page_off;
        uint32_t to_write  = (rem < page_room) ? rem : page_room;

        write_enable();

        uint8_t cmd[4];
        cmd[0] = W25Q128_CMD_PAGE_PROGRAM;
        cmd[1] = (uint8_t)(cur_addr >> 16);
        cmd[2] = (uint8_t)(cur_addr >>  8);
        cmd[3] = (uint8_t)(cur_addr);

        W25Q128_CS_LOW();
        spi_transmit(cmd, 4);
        spi_transmit(src, (uint16_t)to_write);
        W25Q128_CS_HIGH();

        if (W25Q128_WaitForReady(W25Q128_WRITE_TIMEOUT_MS) != HAL_OK)
            return HAL_TIMEOUT;

        src      += to_write;
        cur_addr += to_write;
        rem      -= to_write;
    }
    return HAL_OK;
}

/**
  * @brief  Erase a 4KB sector.
  * @param  sector_addr  Any byte address within the target sector.
  */
HAL_StatusTypeDef W25Q128_EraseSector(uint32_t sector_addr)
{
    if (W25Q128_WaitForReady(W25Q128_WRITE_TIMEOUT_MS) != HAL_OK)
        return HAL_TIMEOUT;

    write_enable();

    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_SECTOR_ERASE_4K;
    cmd[1] = (uint8_t)(sector_addr >> 16);
    cmd[2] = (uint8_t)(sector_addr >>  8);
    cmd[3] = (uint8_t)(sector_addr);

    W25Q128_CS_LOW();
    spi_transmit(cmd, 4);
    W25Q128_CS_HIGH();

    return W25Q128_WaitForReady(W25Q128_SECTOR_ERASE_TIMEOUT_MS);
}

/**
  * @brief  Erase a 64KB block.
  * @param  block_addr  Any byte address within the target 64KB block.
  */
HAL_StatusTypeDef W25Q128_EraseBlock64K(uint32_t block_addr)
{
    if (W25Q128_WaitForReady(W25Q128_WRITE_TIMEOUT_MS) != HAL_OK)
        return HAL_TIMEOUT;

    write_enable();

    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_BLOCK_ERASE_64K;
    cmd[1] = (uint8_t)(block_addr >> 16);
    cmd[2] = (uint8_t)(block_addr >>  8);
    cmd[3] = (uint8_t)(block_addr);

    W25Q128_CS_LOW();
    spi_transmit(cmd, 4);
    W25Q128_CS_HIGH();

    return W25Q128_WaitForReady(W25Q128_BLOCK_ERASE_TIMEOUT_MS);
}

/**
  * @brief  Full chip erase.
  */
HAL_StatusTypeDef W25Q128_EraseChip(void)
{
    if (W25Q128_WaitForReady(W25Q128_WRITE_TIMEOUT_MS) != HAL_OK)
        return HAL_TIMEOUT;

    write_enable();

    uint8_t cmd = W25Q128_CMD_CHIP_ERASE;
    W25Q128_CS_LOW();
    spi_transmit(&cmd, 1);
    W25Q128_CS_HIGH();

    return W25Q128_WaitForReady(W25Q128_CHIP_ERASE_TIMEOUT_MS);
}
