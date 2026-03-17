/**
  ******************************************************************************
  * @file    w25q128.h
  * @brief   W25Q128 SPI NOR Flash driver header
  *
  * W25Q128: 128Mbit (16MB) SPI NOR Flash
  *   - 256 blocks x 64KB
  *   - 4096 sectors x 4KB
  *   - 65536 pages x 256B
  *   Connected via SPI1 (PB3=SCK, PB4=MISO, PB5=MOSI)
  *   CS: PB14
  ******************************************************************************
  */
#ifndef __W25Q128_H__
#define __W25Q128_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

/* ---- Flash geometry ---- */
#define W25Q128_PAGE_SIZE           256U        /* bytes */
#define W25Q128_SECTOR_SIZE         4096U       /* bytes, minimum erase unit */
#define W25Q128_BLOCK_SIZE          65536U      /* bytes, 64KB block erase  */
#define W25Q128_TOTAL_SIZE          (16U*1024U*1024U)   /* 16 MB             */
#define W25Q128_SECTOR_COUNT        (W25Q128_TOTAL_SIZE / W25Q128_SECTOR_SIZE) /* 4096 */
#define W25Q128_BLOCK_COUNT         (W25Q128_TOTAL_SIZE / W25Q128_BLOCK_SIZE)  /* 256  */

/* ---- CS pin (PB14) ---- */
#define W25Q128_CS_GPIO_Port        GPIOB
#define W25Q128_CS_Pin              GPIO_PIN_14
#define W25Q128_CS_LOW()            HAL_GPIO_WritePin(W25Q128_CS_GPIO_Port, W25Q128_CS_Pin, GPIO_PIN_RESET)
#define W25Q128_CS_HIGH()           HAL_GPIO_WritePin(W25Q128_CS_GPIO_Port, W25Q128_CS_Pin, GPIO_PIN_SET)

/* ---- SPI commands ---- */
#define W25Q128_CMD_WRITE_ENABLE        0x06U
#define W25Q128_CMD_WRITE_DISABLE       0x04U
#define W25Q128_CMD_READ_SR1            0x05U
#define W25Q128_CMD_READ_SR2            0x35U
#define W25Q128_CMD_READ_DATA           0x03U
#define W25Q128_CMD_FAST_READ           0x0BU
#define W25Q128_CMD_PAGE_PROGRAM        0x02U
#define W25Q128_CMD_SECTOR_ERASE_4K     0x20U
#define W25Q128_CMD_BLOCK_ERASE_32K     0x52U
#define W25Q128_CMD_BLOCK_ERASE_64K     0xD8U
#define W25Q128_CMD_CHIP_ERASE          0xC7U
#define W25Q128_CMD_POWER_DOWN          0xB9U
#define W25Q128_CMD_RELEASE_POWER_DOWN  0xABU
#define W25Q128_CMD_READ_JEDEC_ID       0x9FU
#define W25Q128_CMD_READ_MFR_DEV_ID     0x90U

/* ---- Status register bits ---- */
#define W25Q128_SR1_BUSY                0x01U   /* Write in progress      */
#define W25Q128_SR1_WEL                 0x02U   /* Write enable latch     */

/* ---- Wait timeout (ms) ----
  Keep these comfortably above the W25Q128 datasheet maxima.
  LevelX may trigger reclaim/metadata updates during a logical-sector write,
  which can require a full 64 KB block erase. If the timeout is too tight,
  the low-level driver returns HAL_TIMEOUT, LevelX propagates LX_ERROR, and
  USB MSC reports a fatal hardware error to Windows during disk init. */
#define W25Q128_SECTOR_ERASE_TIMEOUT_MS  500U
#define W25Q128_BLOCK_ERASE_TIMEOUT_MS   3000U
#define W25Q128_CHIP_ERASE_TIMEOUT_MS    120000U
#define W25Q128_WRITE_TIMEOUT_MS         10U

/* ---- API ---- */
HAL_StatusTypeDef W25Q128_Init(void);
uint32_t          W25Q128_ReadID(void);
HAL_StatusTypeDef W25Q128_Read(uint8_t *pData, uint32_t addr, uint32_t len);
HAL_StatusTypeDef W25Q128_Write(const uint8_t *pData, uint32_t addr, uint32_t len);
HAL_StatusTypeDef W25Q128_EraseSector(uint32_t sector_addr);
HAL_StatusTypeDef W25Q128_EraseBlock64K(uint32_t block_addr);
HAL_StatusTypeDef W25Q128_EraseChip(void);
uint8_t           W25Q128_ReadStatusReg1(void);
HAL_StatusTypeDef W25Q128_WaitForReady(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __W25Q128_H__ */
