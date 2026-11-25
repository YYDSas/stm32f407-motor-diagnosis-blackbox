#include "bsp_w25qxx.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

/* 若 CubeMX 未设置 User Label，默认回退到 PB14 */
#ifndef W25Q_CS_GPIO_Port
#define W25Q_CS_GPIO_Port       GPIOB
#define W25Q_CS_Pin             GPIO_PIN_14
#endif

#define W25Q_CS_LOW()           HAL_GPIO_WritePin(W25Q_CS_GPIO_Port, W25Q_CS_Pin, GPIO_PIN_RESET)
#define W25Q_CS_HIGH()          HAL_GPIO_WritePin(W25Q_CS_GPIO_Port, W25Q_CS_Pin, GPIO_PIN_SET)

#define W25Q_SPI_TIMEOUT_MS     100U

static HAL_StatusTypeDef w25q_write_enable(void)
{
    uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
    HAL_StatusTypeDef ret;

    W25Q_CS_LOW();
    ret = HAL_SPI_Transmit(&hspi1, &cmd, 1U, W25Q_SPI_TIMEOUT_MS);
    W25Q_CS_HIGH();
    return ret;
}

static uint8_t w25q_read_sr1(void)
{
    uint8_t cmd = W25Q_CMD_READ_SR1;
    uint8_t sr1 = 0xFFU;

    W25Q_CS_LOW();
    if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, W25Q_SPI_TIMEOUT_MS) == HAL_OK)
    {
        (void)HAL_SPI_Receive(&hspi1, &sr1, 1U, W25Q_SPI_TIMEOUT_MS);
    }
    W25Q_CS_HIGH();
    return sr1;
}

/* RTOS 友好的 BUSY 轮询：擦除扇区耗时 40-100ms，期间让出 CPU 不饿死高优先级任务 */
static HAL_StatusTypeDef w25q_wait_busy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((w25q_read_sr1() & 0x01U) != 0U)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        else
        {
            HAL_Delay(1);
        }
    }
    return HAL_OK;
}

uint32_t BSP_W25Q_ReadJEDECID(void)
{
    uint8_t cmd = W25Q_CMD_JEDEC_ID;
    uint8_t rx[3] = {0U, 0U, 0U};

    W25Q_CS_LOW();
    if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, W25Q_SPI_TIMEOUT_MS) == HAL_OK)
    {
        (void)HAL_SPI_Receive(&hspi1, rx, 3U, W25Q_SPI_TIMEOUT_MS);
    }
    W25Q_CS_HIGH();

    return ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];
}

HAL_StatusTypeDef BSP_W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t hdr[4];
    HAL_StatusTypeDef ret;

    if ((buf == NULL) || (len == 0U) || ((addr + len) > W25Q_TOTAL_SIZE))
    {
        return HAL_ERROR;
    }

    hdr[0] = W25Q_CMD_READ_DATA;
    hdr[1] = (uint8_t)((addr >> 16) & 0xFFU);
    hdr[2] = (uint8_t)((addr >> 8)  & 0xFFU);
    hdr[3] = (uint8_t)(addr & 0xFFU);

    W25Q_CS_LOW();
    ret = HAL_SPI_Transmit(&hspi1, hdr, 4U, W25Q_SPI_TIMEOUT_MS);
    if (ret == HAL_OK)
    {
        ret = HAL_SPI_Receive(&hspi1, buf, (uint16_t)len, 500U);
    }
    W25Q_CS_HIGH();
    return ret;
}

/* 单页编程：要求不能跨 256B 页边界（32B 对齐记录天然满足） */
HAL_StatusTypeDef BSP_W25Q_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t hdr[4];
    HAL_StatusTypeDef ret;

    if ((data == NULL) || (len == 0U) || (len > W25Q_PAGE_SIZE) ||
        (((addr & 0xFFU) + len) > W25Q_PAGE_SIZE))
    {
        return HAL_ERROR;
    }

    ret = w25q_write_enable();
    if (ret != HAL_OK)
    {
        return ret;
    }

    hdr[0] = W25Q_CMD_PAGE_PROGRAM;
    hdr[1] = (uint8_t)((addr >> 16) & 0xFFU);
    hdr[2] = (uint8_t)((addr >> 8)  & 0xFFU);
    hdr[3] = (uint8_t)(addr & 0xFFU);

    W25Q_CS_LOW();
    ret = HAL_SPI_Transmit(&hspi1, hdr, 4U, W25Q_SPI_TIMEOUT_MS);
    if (ret == HAL_OK)
    {
        ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, W25Q_SPI_TIMEOUT_MS);
    }
    W25Q_CS_HIGH();

    if (ret != HAL_OK)
    {
        return ret;
    }
    return w25q_wait_busy(20U);
}

HAL_StatusTypeDef BSP_W25Q_EraseSector4K(uint32_t addr)
{
    uint8_t hdr[4];
    HAL_StatusTypeDef ret;

    /* 对齐到 4KB 扇区首地址 */
    addr &= ~(W25Q_SECTOR_SIZE - 1U);

    ret = w25q_write_enable();
    if (ret != HAL_OK)
    {
        return ret;
    }

    hdr[0] = W25Q_CMD_SECTOR_ERASE;
    hdr[1] = (uint8_t)((addr >> 16) & 0xFFU);
    hdr[2] = (uint8_t)((addr >> 8)  & 0xFFU);
    hdr[3] = (uint8_t)(addr & 0xFFU);

    W25Q_CS_LOW();
    ret = HAL_SPI_Transmit(&hspi1, hdr, 4U, W25Q_SPI_TIMEOUT_MS);
    W25Q_CS_HIGH();

    if (ret != HAL_OK)
    {
        return ret;
    }
    return w25q_wait_busy(500U);
}
