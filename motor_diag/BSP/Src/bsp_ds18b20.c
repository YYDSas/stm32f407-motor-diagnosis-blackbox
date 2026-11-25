#include "bsp_ds18b20.h"
#include "bsp_dwt.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

/* 探索者 V3.5 板载 1-Wire 引脚: PG9 (已配为 Open-Drain + 板载 4.7K 上拉) */
#define DS_PORT  GPIOG
#define DS_PIN   GPIO_PIN_9

static void ds_dq_low(void)
{
    HAL_GPIO_WritePin(DS_PORT, DS_PIN, GPIO_PIN_RESET);
}

static void ds_dq_release(void)
{
    /* 开漏模式下写 1 即释放总线为高阻，由 4.7K 上拉至 3.3V */
    HAL_GPIO_WritePin(DS_PORT, DS_PIN, GPIO_PIN_SET);
}

static uint8_t ds_dq_read(void)
{
    return (HAL_GPIO_ReadPin(DS_PORT, DS_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t ds_reset_presence(void)
{
    uint8_t presence;

    taskENTER_CRITICAL();
    ds_dq_low();
    BSP_DWT_DelayUs(480U);
    ds_dq_release();
    BSP_DWT_DelayUs(70U);
    presence = (ds_dq_read() == 0U) ? 1U : 0U;
    BSP_DWT_DelayUs(410U);
    taskEXIT_CRITICAL();

    return presence;
}

static void ds_write_bit(uint8_t bit)
{
    taskENTER_CRITICAL();
    if (bit != 0U)
    {
        ds_dq_low();
        BSP_DWT_DelayUs(6U);
        ds_dq_release();
        BSP_DWT_DelayUs(64U);
    }
    else
    {
        ds_dq_low();
        BSP_DWT_DelayUs(60U);
        ds_dq_release();
        BSP_DWT_DelayUs(10U);
    }
    taskEXIT_CRITICAL();
}

static uint8_t ds_read_bit(void)
{
    uint8_t bit;

    taskENTER_CRITICAL();
    ds_dq_low();
    BSP_DWT_DelayUs(3U);
    ds_dq_release();
    BSP_DWT_DelayUs(10U);
    bit = ds_dq_read();
    BSP_DWT_DelayUs(52U);
    taskEXIT_CRITICAL();

    return bit;
}

static void ds_write_byte(uint8_t data)
{
    uint8_t i;
    for (i = 0U; i < 8U; i++)
    {
        ds_write_bit((uint8_t)(data & 0x01U));
        data >>= 1;
    }
}

static uint8_t ds_read_byte(void)
{
    uint8_t i;
    uint8_t val = 0U;
    for (i = 0U; i < 8U; i++)
    {
        if (ds_read_bit() != 0U)
        {
            val |= (uint8_t)(1U << i);
        }
    }
    return val;
}

/* Maxim/Dallas 1-Wire CRC8 (Poly = 0x8C reflected) */
static uint8_t ds_crc8(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0U;
    uint8_t i, j;

    for (i = 0U; i < len; i++)
    {
        uint8_t inbyte = buf[i];
        for (j = 0U; j < 8U; j++)
        {
            uint8_t mix = (uint8_t)((crc ^ inbyte) & 0x01U);
            crc >>= 1;
            if (mix != 0U)
            {
                crc ^= 0x8CU;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}

uint8_t BSP_DS18B20_Detect(void)
{
    ds_dq_release();
    return ds_reset_presence();
}

uint8_t BSP_DS18B20_StartConvert(void)
{
    if (ds_reset_presence() == 0U)
    {
        return 0U;
    }
    ds_write_byte(0xCCU); /* Skip ROM */
    ds_write_byte(0x44U); /* Convert T */
    return 1U;
}

uint8_t BSP_DS18B20_ReadTempX10(int16_t *out_temp_x10)
{
    uint8_t sp[9];
    uint8_t i;
    int16_t raw;

    if (out_temp_x10 == NULL)
    {
        return 0U;
    }
    if (ds_reset_presence() == 0U)
    {
        return 0U;
    }

    ds_write_byte(0xCCU); /* Skip ROM */
    ds_write_byte(0xBEU); /* Read Scratchpad */
    for (i = 0U; i < 9U; i++)
    {
        sp[i] = ds_read_byte();
    }

    if (ds_crc8(sp, 8U) != sp[8])
    {
        return 0U;
    }

    raw = (int16_t)((uint16_t)sp[0] | ((uint16_t)sp[1] << 8));
    /* 分辨率 1/16 ℃ -> 转为 0.1℃: raw * 10 / 16 */
    *out_temp_x10 = (int16_t)((raw * 10) / 16);
    return 1U;
}
