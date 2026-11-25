#include "bb_core.h"
#include <stddef.h>
#include <string.h>

static uint8_t bb_is_all_ff(const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    for (i = 0U; i < len; i++)
    {
        if (buf[i] != 0xFFU)
        {
            return 0U;
        }
    }
    return 1U;
}

static int bb_is_sector_blank(BB_Context_t *ctx, uint32_t sector_addr)
{
    uint8_t  chunk[64];
    uint32_t off;
    for (off = 0U; off < BB_SECTOR_SIZE; off += (uint32_t)sizeof(chunk))
    {
        if (ctx->ops.read(sector_addr + off, chunk, (uint32_t)sizeof(chunk), ctx->ops.user_ctx) != 0)
        {
            return -1;
        }
        if (bb_is_all_ff(chunk, (uint32_t)sizeof(chunk)) == 0U)
        {
            return 0;
        }
    }
    return 1;
}

uint16_t BB_Core_CRC16(const uint8_t *data, uint32_t len)
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

uint8_t BB_Core_ValidateRecord(const BlackboxRecord_t *rec)
{
    uint16_t calc;
    if (rec == NULL)
    {
        return 0U;
    }
    if ((rec->magic != BB_REC_MAGIC) || (rec->commit_flag != BB_REC_COMMIT_FLAG))
    {
        return 0U;
    }
    calc = BB_Core_CRC16((const uint8_t *)rec, (uint32_t)offsetof(BlackboxRecord_t, crc16));
    return (calc == rec->crc16) ? 1U : 0U;
}

uint8_t BB_Core_ValidateSnapshot(const BlackboxSnapshot_t *snap)
{
    uint16_t calc;
    if (snap == NULL)
    {
        return 0U;
    }
    if ((snap->magic != BB_SNAP_MAGIC) || (snap->commit_magic != BB_SNAP_COMMIT_MAGIC))
    {
        return 0U;
    }
    calc = BB_Core_CRC16((const uint8_t *)snap, (uint32_t)offsetof(BlackboxSnapshot_t, crc16));
    return (calc == snap->crc16) ? 1U : 0U;
}

static int bb_mount_records(BB_Context_t *ctx)
{
    BlackboxRecord_t rec;
    uint16_t slot;
    uint32_t max_seq = 0U;
    uint16_t max_slot = 0U;
    uint16_t last_corrupt_slot = 0U;
    uint8_t  has_valid = 0U;
    uint8_t  has_corrupt = 0U;

    ctx->rec_valid_cnt   = 0U;
    ctx->rec_corrupt_cnt = 0U;

    for (slot = 0U; slot < BB_REC_TOTAL_SLOTS; slot++)
    {
        uint32_t addr = BB_REC_BASE_ADDR + (uint32_t)slot * BB_REC_SIZE;
        if (ctx->ops.read(addr, (uint8_t *)&rec, BB_REC_SIZE, ctx->ops.user_ctx) != 0)
        {
            return -1;
        }
        if (bb_is_all_ff((const uint8_t *)&rec, BB_REC_SIZE) != 0U)
        {
            continue;
        }
        if (BB_Core_ValidateRecord(&rec) != 0U)
        {
            ctx->rec_valid_cnt++;
            if ((has_valid == 0U) || (rec.seq > max_seq))
            {
                max_seq   = rec.seq;
                max_slot  = slot;
                has_valid = 1U;
            }
        }
        else
        {
            ctx->rec_corrupt_cnt++;
            last_corrupt_slot = slot;
            has_corrupt = 1U;
        }
    }

    if (has_valid != 0U)
    {
        uint16_t cand = (uint16_t)((max_slot + 1U) % BB_REC_TOTAL_SLOTS);
        uint16_t step;
        ctx->next_seq = max_seq + 1U;

        /* 跳过紧跟在最新合法记录之后的半残撕裂槽位，直到遇到全 0xFF 空白槽位或新扇区起点 */
        for (step = 0U; step < BB_REC_PER_SECTOR; step++)
        {
            uint32_t addr = BB_REC_BASE_ADDR + (uint32_t)cand * BB_REC_SIZE;
            if (ctx->ops.read(addr, (uint8_t *)&rec, BB_REC_SIZE, ctx->ops.user_ctx) != 0)
            {
                return -1;
            }
            if (bb_is_all_ff((const uint8_t *)&rec, BB_REC_SIZE) != 0U)
            {
                break;
            }
            if ((cand % BB_REC_PER_SECTOR) == 0U)
            {
                break;
            }
            cand = (uint16_t)((cand + 1U) % BB_REC_TOTAL_SLOTS);
        }
        ctx->rec_write_slot = cand;
    }
    else
    {
        ctx->next_seq = 1U;
        ctx->rec_write_slot = (has_corrupt != 0U)
                              ? (uint16_t)((last_corrupt_slot + 1U) % BB_REC_TOTAL_SLOTS)
                              : 0U;
    }

    return 0;
}

static int bb_mount_snapshots(BB_Context_t *ctx)
{
    static BlackboxSnapshot_t s_snap_tmp;
    uint16_t slot;
    uint16_t last_valid_slot = 0U;
    uint16_t last_corrupt_slot = 0U;
    uint32_t max_ts = 0U;
    uint8_t  has_valid = 0U;
    uint8_t  has_corrupt = 0U;

    ctx->snap_valid_cnt   = 0U;
    ctx->snap_corrupt_cnt = 0U;

    for (slot = 0U; slot < BB_SNAP_TOTAL_SLOTS; slot++)
    {
        uint32_t addr = BB_SNAP_BASE_ADDR + (uint32_t)slot * BB_SNAP_SIZE;
        if (ctx->ops.read(addr, (uint8_t *)&s_snap_tmp, BB_SNAP_SIZE, ctx->ops.user_ctx) != 0)
        {
            return -1;
        }
        if (bb_is_all_ff((const uint8_t *)&s_snap_tmp, BB_SNAP_SIZE) != 0U)
        {
            continue;
        }
        if (BB_Core_ValidateSnapshot(&s_snap_tmp) != 0U)
        {
            ctx->snap_valid_cnt++;
            if ((has_valid == 0U) || (s_snap_tmp.timestamp_ms >= max_ts))
            {
                max_ts = s_snap_tmp.timestamp_ms;
                last_valid_slot = slot;
                has_valid = 1U;
            }
        }
        else
        {
            ctx->snap_corrupt_cnt++;
            last_corrupt_slot = slot;
            has_corrupt = 1U;
        }
    }

    if (has_valid != 0U)
    {
        uint16_t cand = (uint16_t)((last_valid_slot + 1U) % BB_SNAP_TOTAL_SLOTS);
        uint16_t step;
        for (step = 0U; step < BB_SNAP_PER_SECTOR; step++)
        {
            uint32_t addr = BB_SNAP_BASE_ADDR + (uint32_t)cand * BB_SNAP_SIZE;
            if (ctx->ops.read(addr, (uint8_t *)&s_snap_tmp, BB_SNAP_SIZE, ctx->ops.user_ctx) != 0)
            {
                return -1;
            }
            if (bb_is_all_ff((const uint8_t *)&s_snap_tmp, BB_SNAP_SIZE) != 0U)
            {
                break;
            }
            if ((cand % BB_SNAP_PER_SECTOR) == 0U)
            {
                break;
            }
            cand = (uint16_t)((cand + 1U) % BB_SNAP_TOTAL_SLOTS);
        }
        ctx->snap_write_slot = cand;
    }
    else
    {
        ctx->snap_write_slot = (has_corrupt != 0U)
                               ? (uint16_t)((last_corrupt_slot + 1U) % BB_SNAP_TOTAL_SLOTS)
                               : 0U;
    }

    return 0;
}

int BB_Core_Mount(BB_Context_t *ctx, const BB_FlashOps_t *ops)
{
    if ((ctx == NULL) || (ops == NULL) ||
        (ops->read == NULL) || (ops->write == NULL) || (ops->erase_sector == NULL))
    {
        return -1;
    }

    memset(ctx, 0, sizeof(BB_Context_t));
    ctx->ops = *ops;

    if (bb_mount_records(ctx) != 0)
    {
        return -2;
    }
    if (bb_mount_snapshots(ctx) != 0)
    {
        return -3;
    }

    ctx->mounted = 1U;
    return 0;
}

int BB_Core_FormatAll(BB_Context_t *ctx)
{
    uint32_t i;
    if ((ctx == NULL) || (ctx->ops.erase_sector == NULL))
    {
        return -1;
    }

    for (i = 0U; i < BB_REC_SECTOR_COUNT; i++)
    {
        if (ctx->ops.erase_sector(BB_REC_BASE_ADDR + i * BB_SECTOR_SIZE, ctx->ops.user_ctx) != 0)
        {
            return -2;
        }
    }
    for (i = 0U; i < BB_SNAP_SECTOR_COUNT; i++)
    {
        if (ctx->ops.erase_sector(BB_SNAP_BASE_ADDR + i * BB_SECTOR_SIZE, ctx->ops.user_ctx) != 0)
        {
            return -3;
        }
    }

    ctx->next_seq         = 1U;
    ctx->rec_write_slot   = 0U;
    ctx->rec_valid_cnt    = 0U;
    ctx->rec_corrupt_cnt  = 0U;
    ctx->snap_write_slot  = 0U;
    ctx->snap_valid_cnt   = 0U;
    ctx->snap_corrupt_cnt = 0U;
    ctx->mounted          = 1U;
    return 0;
}

int BB_Core_AppendRecord(BB_Context_t *ctx, BlackboxRecord_t *rec)
{
    uint32_t slot_addr;
    uint32_t sector_addr;
    uint32_t phase1_len;
    uint32_t phase2_len;
    const uint8_t *raw_bytes;

    if ((ctx == NULL) || (rec == NULL) || (ctx->mounted == 0U))
    {
        return -1;
    }

    /* 若游标落在扇区起点且该扇区非空，先擦除整个最旧扇区 */
    if ((ctx->rec_write_slot % BB_REC_PER_SECTOR) == 0U)
    {
        sector_addr = BB_REC_BASE_ADDR + (uint32_t)ctx->rec_write_slot * BB_REC_SIZE;
        int blank = bb_is_sector_blank(ctx, sector_addr);
        if (blank < 0)
        {
            return -2;
        }
        if (blank == 0)
        {
            if (ctx->ops.erase_sector(sector_addr, ctx->ops.user_ctx) != 0)
            {
                return -3;
            }
        }
    }

    rec->magic       = BB_REC_MAGIC;
    rec->seq         = ctx->next_seq;
    rec->commit_flag = BB_REC_COMMIT_FLAG;
    rec->crc16       = BB_Core_CRC16((const uint8_t *)rec,
                                     (uint32_t)offsetof(BlackboxRecord_t, crc16));

    slot_addr  = BB_REC_BASE_ADDR + (uint32_t)ctx->rec_write_slot * BB_REC_SIZE;
    phase1_len = (uint32_t)offsetof(BlackboxRecord_t, commit_flag); /* 前 29 字节 */
    phase2_len = BB_REC_SIZE - phase1_len;                           /* 后 3 字节: commit_flag + crc16 */
    raw_bytes  = (const uint8_t *)rec;

    /* 两阶段抗撕裂提交：Phase 1 写主体，Phase 2 最后写提交标记与 CRC */
    if (ctx->ops.write(slot_addr, raw_bytes, phase1_len, ctx->ops.user_ctx) != 0)
    {
        return -4;
    }
    if (ctx->ops.write(slot_addr + phase1_len, raw_bytes + phase1_len, phase2_len, ctx->ops.user_ctx) != 0)
    {
        return -5;
    }

    ctx->rec_write_slot = (uint16_t)((ctx->rec_write_slot + 1U) % BB_REC_TOTAL_SLOTS);
    ctx->next_seq++;
    if (ctx->rec_valid_cnt < BB_REC_TOTAL_SLOTS)
    {
        ctx->rec_valid_cnt++;
    }
    return 0;
}

int BB_Core_AppendSnapshot(BB_Context_t *ctx, BlackboxSnapshot_t *snap, uint16_t *out_slot)
{
    uint16_t target_slot;
    uint32_t slot_addr;
    uint32_t sector_addr;
    uint32_t phase1_len;
    uint32_t phase2_len;
    const uint8_t *raw_bytes;

    if ((ctx == NULL) || (snap == NULL) || (ctx->mounted == 0U))
    {
        return -1;
    }

    target_slot = ctx->snap_write_slot;

    if ((target_slot % BB_SNAP_PER_SECTOR) == 0U)
    {
        sector_addr = BB_SNAP_BASE_ADDR + (uint32_t)target_slot * BB_SNAP_SIZE;
        int blank = bb_is_sector_blank(ctx, sector_addr);
        if (blank < 0)
        {
            return -2;
        }
        if (blank == 0)
        {
            if (ctx->ops.erase_sector(sector_addr, ctx->ops.user_ctx) != 0)
            {
                return -3;
            }
        }
    }

    snap->magic        = BB_SNAP_MAGIC;
    snap->commit_magic = BB_SNAP_COMMIT_MAGIC;
    snap->crc16        = BB_Core_CRC16((const uint8_t *)snap,
                                       (uint32_t)offsetof(BlackboxSnapshot_t, crc16));

    slot_addr  = BB_SNAP_BASE_ADDR + (uint32_t)target_slot * BB_SNAP_SIZE;
    phase1_len = (uint32_t)offsetof(BlackboxSnapshot_t, commit_magic); /* 前 1020 字节 */
    phase2_len = BB_SNAP_SIZE - phase1_len;                             /* 后 4 字节 */
    raw_bytes  = (const uint8_t *)snap;

    if (ctx->ops.write(slot_addr, raw_bytes, phase1_len, ctx->ops.user_ctx) != 0)
    {
        return -4;
    }
    if (ctx->ops.write(slot_addr + phase1_len, raw_bytes + phase1_len, phase2_len, ctx->ops.user_ctx) != 0)
    {
        return -5;
    }

    if (out_slot != NULL)
    {
        *out_slot = target_slot;
    }

    ctx->snap_write_slot = (uint16_t)((target_slot + 1U) % BB_SNAP_TOTAL_SLOTS);
    if (ctx->snap_valid_cnt < BB_SNAP_TOTAL_SLOTS)
    {
        ctx->snap_valid_cnt++;
    }
    return 0;
}

int BB_Core_ReadRecentRecords(BB_Context_t *ctx,
                              BlackboxRecord_t *out_arr,
                              uint32_t max_count,
                              uint32_t *out_count)
{
    uint32_t found = 0U;
    uint16_t step;
    uint16_t slot;
    BlackboxRecord_t rec;

    if ((ctx == NULL) || (out_arr == NULL) || (out_count == NULL) || (ctx->mounted == 0U))
    {
        return -1;
    }

    /* 从当前写游标往前逆序扫描，按最新→最旧顺序返回合法记录 */
    slot = ctx->rec_write_slot;
    for (step = 0U; (step < BB_REC_TOTAL_SLOTS) && (found < max_count); step++)
    {
        slot = (slot == 0U) ? (uint16_t)(BB_REC_TOTAL_SLOTS - 1U) : (uint16_t)(slot - 1U);
        uint32_t addr = BB_REC_BASE_ADDR + (uint32_t)slot * BB_REC_SIZE;
        if (ctx->ops.read(addr, (uint8_t *)&rec, BB_REC_SIZE, ctx->ops.user_ctx) != 0)
        {
            return -2;
        }
        if (BB_Core_ValidateRecord(&rec) != 0U)
        {
            out_arr[found++] = rec;
        }
    }

    *out_count = found;
    return 0;
}

int BB_Core_ReadSnapshot(BB_Context_t *ctx, uint16_t slot_idx, BlackboxSnapshot_t *out_snap)
{
    uint32_t addr;
    if ((ctx == NULL) || (out_snap == NULL) || (ctx->mounted == 0U) || (slot_idx >= BB_SNAP_TOTAL_SLOTS))
    {
        return -1;
    }

    addr = BB_SNAP_BASE_ADDR + (uint32_t)slot_idx * BB_SNAP_SIZE;
    if (ctx->ops.read(addr, (uint8_t *)out_snap, BB_SNAP_SIZE, ctx->ops.user_ctx) != 0)
    {
        return -2;
    }
    if (BB_Core_ValidateSnapshot(out_snap) == 0U)
    {
        return -3;
    }
    return 0;
}
