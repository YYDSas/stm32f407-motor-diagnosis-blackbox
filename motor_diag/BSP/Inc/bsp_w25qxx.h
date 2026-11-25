#ifndef __BSP_W25QXX_H
#define __BSP_W25QXX_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* W25Q128 参数 */
#define W25Q_JEDEC_ID_128       0xEF4018UL
#define W25Q_PAGE_SIZE          256U
#define W25Q_SECTOR_SIZE        4096U
#define W25Q_TOTAL_SIZE         (16UL * 1024UL * 1024UL)
#define W25Q_JEDEC_ID_W25Q128   0xEF4018UL
#define W25Q_JEDEC_ID_NM25Q128  0x522118UL
#define W25Q_IS_VALID_128M(id)  (((id) == W25Q_JEDEC_ID_W25Q128) || ((id) == W25Q_JEDEC_ID_NM25Q128))
/* 常用指令 */
#define W25Q_CMD_WRITE_ENABLE   0x06U
#define W25Q_CMD_READ_SR1       0x05U
#define W25Q_CMD_READ_DATA      0x03U
#define W25Q_CMD_PAGE_PROGRAM   0x02U
#define W25Q_CMD_SECTOR_ERASE   0x20U
#define W25Q_CMD_JEDEC_ID       0x9FU

uint32_t          BSP_W25Q_ReadJEDECID(void);
HAL_StatusTypeDef BSP_W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len);
HAL_StatusTypeDef BSP_W25Q_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef BSP_W25Q_EraseSector4K(uint32_t addr);

#endif /* __BSP_W25QXX_H */
