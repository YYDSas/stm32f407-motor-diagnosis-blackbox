#include "bsp_bkpsram.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

#ifndef BKPSRAM_BASE
#define BKPSRAM_BASE (0x40024000U)
#endif

/* 指向片内 4 KB 备份 SRAM 起始地址 */
static BkpsramArea_t * const s_bkp = (BkpsramArea_t *)BKPSRAM_BASE;

/* CRC-16/CCITT-FALSE (Poly=0x1021, Init=0xFFFF) */
static uint16_t bkp_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint16_t bkp_calc_header_crc(const BkpsramArea_t *area)
{
    return bkp_crc16((const uint8_t *)area, (uint32_t)offsetof(BkpsramArea_t, header_crc16));
}

static uint16_t bkp_calc_payload_crc(const BkpsramArea_t *area)
{
    const uint8_t *p = (const uint8_t *)&area->frames[0];
    uint32_t len = (uint32_t)sizeof(area->frames) + (uint32_t)sizeof(area->raw_q15);
    return bkp_crc16(p, len);
}

static void bkp_format_recording(uint32_t keep_seq)
{
    memset((void *)s_bkp, 0, sizeof(BkpsramArea_t));
    s_bkp->magic         = BKPSRAM_MAGIC;
    s_bkp->seq           = keep_seq;
    s_bkp->state         = (uint8_t)BKP_STATE_RECORDING;
    s_bkp->payload_crc16 = bkp_calc_payload_crc(s_bkp);
    s_bkp->header_crc16  = bkp_calc_header_crc(s_bkp);
    __DSB();
}

uint8_t BSP_BkpSram_Init(void)
{
    /* 1. 使能电源接口时钟并解锁备份域写保护 (PWR_CR.DBP) */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    /* 2. 使能片内 4KB 备份 SRAM 时钟 (RCC_AHB1ENR.BKPSRAMEN) */
    __HAL_RCC_BKPSRAM_CLK_ENABLE();

    /* 3. 使能备份稳压器 (PWR_CSR.BRE)，保证 VBAT 断电期间数据不丢失 */
    (void)HAL_PWREx_EnableBkUpReg();

    /* 4. 校验现有内容：若是合法的故障冻结现场，立即保留供上层迁移 */
    if (BSP_BkpSram_IsFrozenValid() != 0U)
    {
        return 1U;
    }

    /* 5. 若魔数或头 CRC 无效（首次上电或纽扣电池没电），格式化为录波态 */
    if ((s_bkp->magic != BKPSRAM_MAGIC) ||
        (s_bkp->header_crc16 != bkp_calc_header_crc(s_bkp)))
    {
        bkp_format_recording(0U);
    }
    else if (s_bkp->state != (uint8_t)BKP_STATE_RECORDING)
    {
        bkp_format_recording(s_bkp->seq);
    }

    return 0U;
}

void BSP_BkpSram_PushFrame(const BkpFrameMini_t *frame,
                           const int16_t *raw_chunk,
                           uint16_t raw_len)
{
    uint16_t i;

    if (s_bkp->magic != BKPSRAM_MAGIC)
    {
        return;
    }
    if (s_bkp->state != (uint8_t)BKP_STATE_RECORDING)
    {
        return;
    }

    if (frame != NULL)
    {
        s_bkp->frames[s_bkp->frame_head] = *frame;
        s_bkp->frame_head = (uint16_t)((s_bkp->frame_head + 1U) % BKPSRAM_FRAME_HIST);
        if (s_bkp->frame_count < BKPSRAM_FRAME_HIST)
        {
            s_bkp->frame_count++;
        }
    }

    if ((raw_chunk != NULL) && (raw_len > 0U))
    {
        for (i = 0U; i < raw_len; i++)
        {
            s_bkp->raw_q15[s_bkp->raw_head] = raw_chunk[i];
            s_bkp->raw_head = (uint16_t)((s_bkp->raw_head + 1U) % BKPSRAM_RAW_SAMPLES);
            if (s_bkp->raw_count < BKPSRAM_RAW_SAMPLES)
            {
                s_bkp->raw_count++;
            }
        }
    }
}

void BSP_BkpSram_Freeze(uint8_t fault_type, uint8_t reset_csr)
{
    if (s_bkp->magic != BKPSRAM_MAGIC)
    {
        return;
    }
    /* 避免重复覆盖尚未迁移的冻结现场 */
    if (s_bkp->state == (uint8_t)BKP_STATE_FROZEN_UNMIGRATED)
    {
        return;
    }

    s_bkp->seq++;
    s_bkp->freeze_tick_ms = HAL_GetTick();
    s_bkp->fault_type     = fault_type;
    s_bkp->reset_csr      = reset_csr;
    s_bkp->payload_crc16  = bkp_calc_payload_crc(s_bkp);
    s_bkp->state          = (uint8_t)BKP_STATE_FROZEN_UNMIGRATED;
    s_bkp->header_crc16   = bkp_calc_header_crc(s_bkp);
    __DSB();
}

uint8_t BSP_BkpSram_IsFrozenValid(void)
{
    if (s_bkp->magic != BKPSRAM_MAGIC)
    {
        return 0U;
    }
    if (s_bkp->state != (uint8_t)BKP_STATE_FROZEN_UNMIGRATED)
    {
        return 0U;
    }
    if (s_bkp->header_crc16 != bkp_calc_header_crc(s_bkp))
    {
        return 0U;
    }
    if (s_bkp->payload_crc16 != bkp_calc_payload_crc(s_bkp))
    {
        return 0U;
    }
    return 1U;
}

void BSP_BkpSram_MarkMigratedAndReset(void)
{
    uint32_t next_seq = s_bkp->seq;
    s_bkp->state = (uint8_t)BKP_STATE_MIGRATED;
    __DSB();
    bkp_format_recording(next_seq);
}

void BSP_BkpSram_FillTestPatternAndFreeze(void)
{
    BkpFrameMini_t f;
    int16_t chunk[64];
    uint16_t i;
    uint16_t blk;

    /* 先重置为录波态，再写入确定性测试数据 */
    bkp_format_recording(s_bkp->seq);

    for (blk = 0U; blk < 16U; blk++)
    {
        memset(&f, 0, sizeof(f));
        f.tick_ms      = HAL_GetTick() + (uint32_t)blk * 10U;
        f.z_score      = 1.25f + 0.30f * (float)blk;
        f.rms_mg       = 12.5f + (float)blk;
        f.peak_freq_hz = 200.0f;
        f.kurtosis     = 3.10f;
        f.temp_x10     = 265;
        f.flags        = 0x0001U;

        for (i = 0U; i < 64U; i++)
        {
            chunk[i] = (int16_t)((blk * 64U + i) * 16U - 8192);
        }
        BSP_BkpSram_PushFrame(&f, chunk, 64U);
    }

    BSP_BkpSram_Freeze(0xFEU, 0x00U);
}

const BkpsramArea_t *BSP_BkpSram_GetArea(void)
{
    return s_bkp;
}
