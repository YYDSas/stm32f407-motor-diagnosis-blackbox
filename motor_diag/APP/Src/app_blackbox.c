#include "app_blackbox.h"
#include "stm32f4xx_hal.h"
#include "app_cli.h"
#include "bsp_w25qxx.h"
#include "bsp_bkpsram.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "app_supervisor.h"
#include <string.h>

#define BB_EVENT_POOL_SIZE   (4U)
#define BB_DUMP_BUF_MAX      (32U)

typedef struct {
    BlackboxRecord_t rec;
    uint8_t          has_snapshot;
    float            fs_hz;
    float            scale_max;
    int16_t          spec_q15[BB_SNAP_BINS];
} BlackboxEvent_t;

/* 大型静态缓冲区全部放入 .bss，避免占用任务栈或 FreeRTOS 堆 */
static BB_Context_t        s_bb_ctx;
static BlackboxEvent_t     s_ev_pool[BB_EVENT_POOL_SIZE];
static volatile uint8_t    s_ev_used[BB_EVENT_POOL_SIZE];
static BlackboxSnapshot_t  s_snap_io_buf;
static BlackboxRecord_t    s_dump_buf[BB_DUMP_BUF_MAX];

static SemaphoreHandle_t   s_spi1_mutex  = NULL;
static QueueHandle_t       s_bb_queue    = NULL;
static volatile uint8_t    s_bb_ready    = 0U;
static volatile uint8_t    s_stress_on   = 0U;

/* ==================== 底层 NM25Q128 适配回调 ==================== */

static int w25q_read_cb(uint32_t addr, uint8_t *buf, uint32_t len, void *user_ctx)
{
    (void)user_ctx;
    return (BSP_W25Q_Read(addr, buf, len) == HAL_OK) ? 0 : -1;
}

/* 自动按 256 字节页边界拆分写入，防止跨页回绕覆盖 */
static int w25q_write_cb(uint32_t addr, const uint8_t *buf, uint32_t len, void *user_ctx)
{
    uint32_t remain = len;
    (void)user_ctx;

    while (remain > 0U)
    {
        uint16_t page_rem = (uint16_t)(W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE));
        uint16_t chunk    = (remain < (uint32_t)page_rem) ? (uint16_t)remain : page_rem;

        if (BSP_W25Q_PageProgram(addr, buf, chunk) != HAL_OK)
        {
            return -1;
        }
        addr   += chunk;
        buf    += chunk;
        remain -= chunk;
    }
    return 0;
}

static int w25q_erase_cb(uint32_t sector_addr, void *user_ctx)
{
    (void)user_ctx;
    return (BSP_W25Q_EraseSector4K(sector_addr) == HAL_OK) ? 0 : -1;
}

static const char *bb_fault_name(uint8_t ft)
{
    switch (ft)
    {
        case BB_FAULT_HIL_800HZ:  return "HIL_800";
        case BB_FAULT_OVERLOAD:   return "OVERLOAD";
        case BB_FAULT_LOOSENESS:  return "LOOSE";
        case BB_FAULT_OVERTEMP:   return "OVERTEMP";
        case BB_FAULT_IWDG_RESET: return "IWDG_RST";
        case BB_FAULT_BKP_MIG:    return "BKP_MIG";
        case BB_FAULT_STRESS:     return "STRESS";
        default:                  return "NONE";
    }
}

/* ==================== 上电 BKPSRAM 自动迁移 ==================== */

static void bb_migrate_bkpsram_if_needed(void)
{
    const BkpsramArea_t *b;
    BlackboxRecord_t rec;

    if (BSP_BkpSram_IsFrozenValid() == 0U)
    {
        return;
    }

    b = BSP_BkpSram_GetArea();
    memset(&rec, 0, sizeof(rec));
    rec.timestamp_ms = b->freeze_tick_ms;
    rec.fault_type   = (uint8_t)BB_FAULT_BKP_MIG;
    rec.mode         = 0U;
    rec.reset_reason = b->reset_csr;
    rec.snapshot_idx = BB_SNAP_NONE;

    if (b->frame_count > 0U)
    {
        uint16_t last_idx = (uint16_t)((b->frame_head + BKPSRAM_FRAME_HIST - 1U) % BKPSRAM_FRAME_HIST);
        const BkpFrameMini_t *lf = &b->frames[last_idx];
        rec.peak_freq_hz = lf->peak_freq_hz;
        rec.z_score      = lf->z_score;
        rec.rms_mg       = lf->rms_mg;
        rec.temp_x10     = lf->temp_x10;
    }

    if (BB_Core_AppendRecord(&s_bb_ctx, &rec) == 0)
    {
        uint32_t bkp_seq = b->seq;
        uint16_t f_cnt   = b->frame_count;
        uint16_t r_cnt   = b->raw_count;
        BSP_BkpSram_MarkMigratedAndReset();
        Log_Printf("[BKP] Auto-migrated frozen seq=%lu -> Flash seq=%lu (frames=%u raw=%u)\r\n",
                   (unsigned long)bkp_seq, (unsigned long)rec.seq,
                   (unsigned int)f_cnt, (unsigned int)r_cnt);
    }
    else
    {
        Log_Printf("[BKP] Migration to Flash FAILED!\r\n");
    }
}

/* ==================== 黑匣子后台落盘任务 ==================== */

static void Task_Blackbox(void *arg)
{
    BB_FlashOps_t ops;
    uint32_t jedec;
    uint8_t idx = 0U;
    (void)arg;

    ops.read         = w25q_read_cb;
    ops.write        = w25q_write_cb;
    ops.erase_sector = w25q_erase_cb;
    ops.user_ctx     = NULL;

    /* 1. 上电校验 Flash 并扫描恢复环形游标 */
    if (xSemaphoreTake(s_spi1_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
    {
        jedec = BSP_W25Q_ReadJEDECID();
        if (W25Q_IS_VALID_128M(jedec))
        {
            if (BB_Core_Mount(&s_bb_ctx, &ops) == 0)
            {
                s_bb_ready = 1U;
                Log_Printf("[BB] Mounted JEDEC=0x%06lX | rec(valid=%u corrupt=%u next_seq=%lu) | snap(valid=%u corrupt=%u)\r\n",
                           (unsigned long)jedec,
                           (unsigned int)s_bb_ctx.rec_valid_cnt,
                           (unsigned int)s_bb_ctx.rec_corrupt_cnt,
                           (unsigned long)s_bb_ctx.next_seq,
                           (unsigned int)s_bb_ctx.snap_valid_cnt,
                           (unsigned int)s_bb_ctx.snap_corrupt_cnt);

                /* 2. 自动迁移断电前冻结在 BKPSRAM 中的现场数据 */
                bb_migrate_bkpsram_if_needed();
            }
            else
            {
                Log_Printf("[BB] Mount FAILED!\r\n");
            }
        }
        else
        {
            Log_Printf("[BB] Invalid Flash JEDEC=0x%06lX\r\n", (unsigned long)jedec);
        }
        xSemaphoreGive(s_spi1_mutex);
    }

    /* 3. 主循环：处理异步故障事件或拔电撕裂压测 */
    for (;;)
    {
        APP_Supervisor_Heartbeat(SUP_HB_BB);
        TickType_t wait_ticks = (s_stress_on != 0U) ? pdMS_TO_TICKS(20) : pdMS_TO_TICKS(500);

        if (xQueueReceive(s_bb_queue, &idx, wait_ticks) == pdTRUE)
        {
            if ((idx < BB_EVENT_POOL_SIZE) && (s_bb_ready != 0U))
            {
                BlackboxEvent_t *ev = &s_ev_pool[idx];
                uint16_t snap_slot = BB_SNAP_NONE;

                if (xSemaphoreTake(s_spi1_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
                {
                    if (ev->has_snapshot != 0U)
                    {
                        memset(&s_snap_io_buf, 0, sizeof(s_snap_io_buf));
                        s_snap_io_buf.seq_low      = (uint16_t)(s_bb_ctx.next_seq & 0xFFFFU);
                        s_snap_io_buf.timestamp_ms = ev->rec.timestamp_ms;
                        s_snap_io_buf.fs_hz        = ev->fs_hz;
                        s_snap_io_buf.scale_max    = ev->scale_max;
                        memcpy(s_snap_io_buf.spec_q15, ev->spec_q15, sizeof(s_snap_io_buf.spec_q15));

                        if (BB_Core_AppendSnapshot(&s_bb_ctx, &s_snap_io_buf, &snap_slot) != 0)
                        {
                            snap_slot = BB_SNAP_NONE;
                        }
                    }

                    ev->rec.snapshot_idx = snap_slot;
                    if (BB_Core_AppendRecord(&s_bb_ctx, &ev->rec) == 0)
                    {
                        Log_Printf("[BB] Saved seq=%lu type=%s peak=%.1fHz val=%.2f snap=%u\r\n",
                                   (unsigned long)ev->rec.seq,
                                   bb_fault_name(ev->rec.fault_type),
                                   ev->rec.peak_freq_hz,
                                   ev->rec.z_score,
                                   (unsigned int)ev->rec.snapshot_idx);
                    }
                    xSemaphoreGive(s_spi1_mutex);
                }

                taskENTER_CRITICAL();
                s_ev_used[idx] = 0U;
                taskEXIT_CRITICAL();
            }
        }
        else if ((s_stress_on != 0U) && (s_bb_ready != 0U))
        {
            /* 拔电撕裂测试模式：每 20 ms 写一条带递增序号的测试记录 */
            BlackboxRecord_t srec;
            memset(&srec, 0, sizeof(srec));
            srec.timestamp_ms = HAL_GetTick();
            srec.fault_type   = (uint8_t)BB_FAULT_STRESS;
            srec.peak_freq_hz = 200.0f;
            srec.z_score      = 9.99f;
            srec.snapshot_idx = BB_SNAP_NONE;

            if (xSemaphoreTake(s_spi1_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
            {
                if (BB_Core_AppendRecord(&s_bb_ctx, &srec) == 0)
                {
                    Log_Printf("[STRESS] seq=%lu\r\n", (unsigned long)srec.seq);
                }
                xSemaphoreGive(s_spi1_mutex);
            }
        }
    }
}

/* ==================== 对外 API ==================== */

void APP_Blackbox_Init(void)
{
    memset((void *)s_ev_used, 0, sizeof(s_ev_used));
    s_spi1_mutex = xSemaphoreCreateMutex();
    s_bb_queue   = xQueueCreate(BB_EVENT_POOL_SIZE, sizeof(uint8_t));
    xTaskCreate(Task_Blackbox, "Task_Blackbox", 512, NULL, tskIDLE_PRIORITY + 20, NULL);
}

uint8_t APP_Blackbox_PushFault(const BlackboxRecord_t *rec,
                               const float *fft_mag,
                               uint16_t mag_len,
                               float fs_hz)
{
    uint8_t idx = 0xFFU;
    uint8_t i;

    if ((rec == NULL) || (s_bb_ready == 0U) || (s_bb_queue == NULL))
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    for (i = 0U; i < BB_EVENT_POOL_SIZE; i++)
    {
        if (s_ev_used[i] == 0U)
        {
            s_ev_used[i] = 1U;
            idx = i;
            break;
        }
    }
    taskEXIT_CRITICAL();

    if (idx == 0xFFU)
    {
        return 0U;
    }

    s_ev_pool[idx].rec          = *rec;
    s_ev_pool[idx].has_snapshot = 0U;

    if ((fft_mag != NULL) && (mag_len > 0U))
    {
        uint16_t n = (mag_len < BB_SNAP_BINS) ? mag_len : BB_SNAP_BINS;
        uint16_t k;
        float max_v = 1e-6f;

        for (k = 0U; k < n; k++)
        {
            if (fft_mag[k] > max_v)
            {
                max_v = fft_mag[k];
            }
        }

        s_ev_pool[idx].has_snapshot = 1U;
        s_ev_pool[idx].fs_hz        = fs_hz;
        s_ev_pool[idx].scale_max    = max_v;

        for (k = 0U; k < BB_SNAP_BINS; k++)
        {
            if (k < n)
            {
                float norm = (fft_mag[k] / max_v) * 32767.0f;
                if (norm > 32767.0f) norm = 32767.0f;
                if (norm < 0.0f)     norm = 0.0f;
                s_ev_pool[idx].spec_q15[k] = (int16_t)norm;
            }
            else
            {
                s_ev_pool[idx].spec_q15[k] = 0;
            }
        }
    }

    if (xQueueSend(s_bb_queue, &idx, 0) != pdTRUE)
    {
        taskENTER_CRITICAL();
        s_ev_used[idx] = 0U;
        taskEXIT_CRITICAL();
        return 0U;
    }
    return 1U;
}

void APP_Blackbox_PrintDump(uint32_t max_n)
{
    uint32_t req = (max_n == 0U) ? 20U : max_n;
    uint32_t got = 0U;
    uint32_t i;

    if (req > BB_DUMP_BUF_MAX) req = BB_DUMP_BUF_MAX;
    if (s_bb_ready == 0U)
    {
        Log_Printf("[BB] Not mounted\r\n");
        return;
    }

    if (xSemaphoreTake(s_spi1_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        int rc = BB_Core_ReadRecentRecords(&s_bb_ctx, s_dump_buf, req, &got);
        xSemaphoreGive(s_spi1_mutex);

        if (rc != 0)
        {
            Log_Printf("[BB] Read error rc=%d\r\n", rc);
            return;
        }

        Log_Printf("\r\n[BB DUMP] valid=%u corrupt=%u next_seq=%lu (showing %lu)\r\n",
                   (unsigned int)s_bb_ctx.rec_valid_cnt,
                   (unsigned int)s_bb_ctx.rec_corrupt_cnt,
                   (unsigned long)s_bb_ctx.next_seq,
                   (unsigned long)got);
        Log_Printf("Seq    Tick(ms)   Type      Peak(Hz)  Score/800  RMS/Amp   Snap\r\n");
        Log_Printf("----------------------------------------------------------------\r\n");
        for (i = 0U; i < got; i++)
        {
            const BlackboxRecord_t *r = &s_dump_buf[i];
            Log_Printf("%-6lu %-10lu %-9s %-9.1f %-10.2f %-9.2f %u\r\n",
                       (unsigned long)r->seq,
                       (unsigned long)r->timestamp_ms,
                       bb_fault_name(r->fault_type),
                       r->peak_freq_hz,
                       r->z_score,
                       r->rms_mg,
                       (unsigned int)r->snapshot_idx);
        }
    }
}

void APP_Blackbox_PrintSnapshot(uint16_t slot_idx)
{
    uint16_t i;
    int rc;

    if (s_bb_ready == 0U)
    {
        Log_Printf("[BB] Not mounted\r\n");
        return;
    }

    if (xSemaphoreTake(s_spi1_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        rc = BB_Core_ReadSnapshot(&s_bb_ctx, slot_idx, &s_snap_io_buf);
        xSemaphoreGive(s_spi1_mutex);

        if (rc != 0)
        {
            Log_Printf("[SNAP] Slot %u invalid/empty (rc=%d)\r\n", (unsigned int)slot_idx, rc);
            return;
        }

        Log_Printf("[SNAP %u] seq_low=%u ts=%lu fs=%.2f scale_max=%.4f\r\n",
                   (unsigned int)slot_idx,
                   (unsigned int)s_snap_io_buf.seq_low,
                   (unsigned long)s_snap_io_buf.timestamp_ms,
                   s_snap_io_buf.fs_hz,
                   s_snap_io_buf.scale_max);

        /* 每行输出 16 个 q15 bin 值，便于 Python 上位机解析 */
        for (i = 0U; i < BB_SNAP_BINS; i++)
        {
            if ((i % 16U) == 0U)
            {
                Log_Printf("B%03u:", (unsigned int)i);
            }
            Log_Printf("%d%s", (int)s_snap_io_buf.spec_q15[i],
                       (((i + 1U) % 16U) == 0U || (i + 1U) == BB_SNAP_BINS) ? "\r\n" : ",");
        }
    }
}

void APP_Blackbox_ClearAll(void)
{
    if (s_bb_ready == 0U)
    {
        Log_Printf("[BB] Not mounted\r\n");
        return;
    }

    s_stress_on = 0U;
    if (xSemaphoreTake(s_spi1_mutex, pdMS_TO_TICKS(3000)) == pdTRUE)
    {
        int rc = BB_Core_FormatAll(&s_bb_ctx);
        xSemaphoreGive(s_spi1_mutex);
        Log_Printf("[BB] FormatAll %s (rc=%d)\r\n", (rc == 0) ? "OK" : "FAIL", rc);
    }
}

void APP_Blackbox_SetStressMode(uint8_t enable)
{
    s_stress_on = (enable != 0U) ? 1U : 0U;
    Log_Printf("[STRESS] Flash tearing stress mode = %s\r\n", s_stress_on ? "ON (20ms/rec)" : "OFF");
}
