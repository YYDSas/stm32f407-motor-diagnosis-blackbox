#include "bb_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 模拟 128 KB NOR Flash (0x000000 ~ 0x01FFFF)，覆盖全部摘要区与快照区 */
#define SIM_FLASH_SIZE   (128U * 1024U)

typedef struct {
    uint8_t  mem[SIM_FLASH_SIZE];
    int32_t  cut_after_bytes;  /* >=0 时触发掉电字节截断模拟；-1 表示正常不掉电 */
    uint32_t total_erases;
} SimFlash_t;

static int sim_read(uint32_t addr, uint8_t *buf, uint32_t len, void *user_ctx)
{
    SimFlash_t *f = (SimFlash_t *)user_ctx;
    if ((addr + len) > SIM_FLASH_SIZE) return -1;
    memcpy(buf, &f->mem[addr], len);
    return 0;
}

static int sim_write(uint32_t addr, const uint8_t *buf, uint32_t len, void *user_ctx)
{
    SimFlash_t *f = (SimFlash_t *)user_ctx;
    uint32_t i;
    if ((addr + len) > SIM_FLASH_SIZE) return -1;

    for (i = 0U; i < len; i++)
    {
        if (f->cut_after_bytes == 0)
        {
            /* 模拟掉电瞬间中断写入 */
            return -99;
        }
        /* 严格遵循 NOR Flash 物理特性：只能 1 -> 0，不能 0 -> 1 */
        f->mem[addr + i] &= buf[i];
        if (f->cut_after_bytes > 0)
        {
            f->cut_after_bytes--;
        }
    }
    return 0;
}

static int sim_erase_sector(uint32_t sector_addr, void *user_ctx)
{
    SimFlash_t *f = (SimFlash_t *)user_ctx;
    if (((sector_addr % BB_SECTOR_SIZE) != 0U) || ((sector_addr + BB_SECTOR_SIZE) > SIM_FLASH_SIZE))
    {
        return -1;
    }
    memset(&f->mem[sector_addr], 0xFF, BB_SECTOR_SIZE);
    f->total_erases++;
    return 0;
}

static void sim_init(SimFlash_t *f)
{
    memset(f->mem, 0xFF, sizeof(f->mem));
    f->cut_after_bytes = -1;
    f->total_erases    = 0U;
}

static BB_FlashOps_t make_ops(SimFlash_t *f)
{
    BB_FlashOps_t ops;
    ops.read         = sim_read;
    ops.write        = sim_write;
    ops.erase_sector = sim_erase_sector;
    ops.user_ctx     = f;
    return ops;
}

static int test_basic_and_wraparound(void)
{
    SimFlash_t flash;
    BB_FlashOps_t ops = make_ops(&flash);
    BB_Context_t ctx;
    BlackboxRecord_t rec;
    BlackboxRecord_t out[16];
    uint32_t cnt = 0U;
    uint32_t i;

    sim_init(&flash);
    if (BB_Core_Mount(&ctx, &ops) != 0) return -1;
    if ((ctx.next_seq != 1U) || (ctx.rec_write_slot != 0U)) return -2;

    /* 写入 1200 条记录（超过 1024 槽位上限，触发跨扇区回绕擦除最旧扇区） */
    for (i = 0U; i < 1200U; i++)
    {
        memset(&rec, 0, sizeof(rec));
        rec.timestamp_ms = 100U * (i + 1U);
        rec.peak_freq_hz = 200.0f;
        rec.z_score      = 4.5f;
        rec.rms_mg       = 25.0f;
        rec.temp_x10     = 285;
        rec.fault_type   = (uint8_t)BB_FAULT_HIL_800HZ;
        rec.snapshot_idx = BB_SNAP_NONE;
        if (BB_Core_AppendRecord(&ctx, &rec) != 0) return -3;
    }

    /* 模拟掉电重启重新 Mount */
    if (BB_Core_Mount(&ctx, &ops) != 0) return -4;
    if (ctx.next_seq != 1201U) return -5;
    if (ctx.rec_corrupt_cnt != 0U) return -6;
    if (ctx.rec_valid_cnt < (7U * BB_REC_PER_SECTOR)) return -7;

    /* 校验最近 10 条记录序号连续递减: 1200, 1199, ..., 1191 */
    if (BB_Core_ReadRecentRecords(&ctx, out, 10U, &cnt) != 0) return -8;
    if (cnt != 10U) return -9;
    for (i = 0U; i < 10U; i++)
    {
        if (out[i].seq != (1200U - i)) return -10;
    }

    printf("[PASS] test_basic_and_wraparound (written=1200, retained=%u, next_seq=%u)\n",
           (unsigned)ctx.rec_valid_cnt, (unsigned)ctx.next_seq);
    return 0;
}

static int test_1000_tearing_cuts(void)
{
    SimFlash_t flash;
    BB_FlashOps_t ops = make_ops(&flash);
    BB_Context_t ctx;
    BlackboxRecord_t rec;
    BlackboxRecord_t recent[4];
    uint32_t cnt = 0U;
    uint32_t iter;
    uint32_t caught_tears = 0U;

    sim_init(&flash);
    if (BB_Core_Mount(&ctx, &ops) != 0) return -1;

    srand(20261125U);

    for (iter = 0U; iter < 1000U; iter++)
    {
        uint32_t expected_last_seq;

        /* 1. 正常写入 1 条合法记录 */
        flash.cut_after_bytes = -1;
        memset(&rec, 0, sizeof(rec));
        rec.timestamp_ms = iter * 50U;
        rec.peak_freq_hz = 400.0f;
        rec.z_score      = 5.2f;
        rec.fault_type   = (uint8_t)BB_FAULT_STRESS;
        rec.snapshot_idx = BB_SNAP_NONE;
        if (BB_Core_AppendRecord(&ctx, &rec) != 0) return -2;
        expected_last_seq = rec.seq;

        /* 2. 写入下一条记录时，在 1..31 字节的随机偏移处强制断电（写撕裂） */
        flash.cut_after_bytes = (int32_t)(1 + (rand() % 31));
        memset(&rec, 0, sizeof(rec));
        rec.timestamp_ms = iter * 50U + 10U;
        rec.fault_type   = (uint8_t)BB_FAULT_OVERLOAD;
        rec.snapshot_idx = BB_SNAP_NONE;
        if (BB_Core_AppendRecord(&ctx, &rec) == 0)
        {
            return -3; /* 应当失败 */
        }

        /* 3. 恢复供电并重新 Mount：必须 100% 拦截撕裂记录，且无损保留上一条合法记录 */
        flash.cut_after_bytes = -1;
        if (BB_Core_Mount(&ctx, &ops) != 0) return -4;
        if (ctx.next_seq != (expected_last_seq + 1U)) return -5;
        if (ctx.rec_corrupt_cnt == 0U) return -6;

        if (BB_Core_ReadRecentRecords(&ctx, recent, 1U, &cnt) != 0) return -7;
        if ((cnt != 1U) || (recent[0].seq != expected_last_seq)) return -8;

        caught_tears++;
    }

    printf("[PASS] test_1000_tearing_cuts (cuts=%u, recovered=100%%, crc_intercept=100%%)\n",
           (unsigned)caught_tears);
    return 0;
}

static int test_snapshot_and_tearing(void)
{
    SimFlash_t flash;
    BB_FlashOps_t ops = make_ops(&flash);
    BB_Context_t ctx;
    BlackboxSnapshot_t snap;
    BlackboxSnapshot_t readback;
    uint16_t slot = 0U;
    uint16_t i;

    sim_init(&flash);
    if (BB_Core_Mount(&ctx, &ops) != 0) return -1;

    /* 正常写入一条 1 KB 频谱快照 */
    memset(&snap, 0, sizeof(snap));
    snap.seq_low      = 1U;
    snap.timestamp_ms = 5555U;
    snap.fs_hz        = 3200.0f;
    snap.scale_max    = 1.25f;
    for (i = 0U; i < BB_SNAP_BINS; i++)
    {
        snap.spec_q15[i] = (int16_t)(i * 30U);
    }
    if (BB_Core_AppendSnapshot(&ctx, &snap, &slot) != 0) return -2;
    if (slot != 0U) return -3;

    /* 写入第 2 条快照中途在 800 字节处撕裂断电 */
    flash.cut_after_bytes = 800;
    snap.seq_low = 2U;
    snap.timestamp_ms = 6666U;
    if (BB_Core_AppendSnapshot(&ctx, &snap, &slot) == 0) return -4;

    /* 重启 Mount 验证：第 0 条完好，第 1 条被拦截，新写游标自动跳过坏槽指向 slot 2 */
    flash.cut_after_bytes = -1;
    if (BB_Core_Mount(&ctx, &ops) != 0) return -5;
    if ((ctx.snap_valid_cnt != 1U) || (ctx.snap_corrupt_cnt != 1U)) return -6;
    if (ctx.snap_write_slot != 2U) return -7;

    if (BB_Core_ReadSnapshot(&ctx, 0U, &readback) != 0) return -8;
    if ((readback.seq_low != 1U) || (readback.spec_q15[501] != (int16_t)(501 * 30))) return -9;
    if (BB_Core_ReadSnapshot(&ctx, 1U, &readback) == 0) return -10; /* 损坏槽必须拒绝读取 */

    printf("[PASS] test_snapshot_and_tearing (valid=%u, corrupt=%u, next_slot=%u)\n",
           (unsigned)ctx.snap_valid_cnt, (unsigned)ctx.snap_corrupt_cnt,
           (unsigned)ctx.snap_write_slot);
    return 0;
}

int main(void)
{
    int rc;
    printf("=== Blackbox Core Host Unit Tests ===\n");

    rc = test_basic_and_wraparound();
    if (rc != 0) { printf("[FAIL] test_basic_and_wraparound rc=%d\n", rc); return 1; }

    rc = test_1000_tearing_cuts();
    if (rc != 0) { printf("[FAIL] test_1000_tearing_cuts rc=%d\n", rc); return 1; }

    rc = test_snapshot_and_tearing();
    if (rc != 0) { printf("[FAIL] test_snapshot_and_tearing rc=%d\n", rc); return 1; }

    printf("=== ALL UNIT TESTS PASSED ===\n");
    return 0;
}
