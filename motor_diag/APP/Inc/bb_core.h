#ifndef __BB_CORE_H
#define __BB_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NM25Q128 / W25Q128 4 KB 扇区对齐分区布局 */
#define BB_SECTOR_SIZE        (4096U)

/* 摘要环形区: Sector 2..9 (0x002000 ~ 0x009FFF, 32 KB, 1024 条) */
#define BB_REC_BASE_ADDR      (0x002000U)
#define BB_REC_SECTOR_COUNT   (8U)
#define BB_REC_SIZE           (32U)
#define BB_REC_PER_SECTOR     (BB_SECTOR_SIZE / BB_REC_SIZE)   /* 128 */
#define BB_REC_TOTAL_SLOTS    (BB_REC_SECTOR_COUNT * BB_REC_PER_SECTOR) /* 1024 */

/* 频谱快照环形区: Sector 10..25 (0x00A000 ~ 0x019FFF, 64 KB, 64 条) */
#define BB_SNAP_BASE_ADDR     (0x00A000U)
#define BB_SNAP_SECTOR_COUNT  (16U)
#define BB_SNAP_SIZE          (1024U)
#define BB_SNAP_PER_SECTOR    (BB_SECTOR_SIZE / BB_SNAP_SIZE)  /* 4 */
#define BB_SNAP_TOTAL_SLOTS   (BB_SNAP_SECTOR_COUNT * BB_SNAP_PER_SECTOR) /* 64 */

#define BB_REC_MAGIC          (0xA55AU)
#define BB_REC_COMMIT_FLAG    (0x5AU)
#define BB_SNAP_MAGIC         (0x5AA5U)
#define BB_SNAP_COMMIT_MAGIC  (0xAA55U)
#define BB_SNAP_NONE          (0xFFFFU)
#define BB_SNAP_BINS          (502U)

/* 故障类型编码 */
typedef enum {
    BB_FAULT_NONE       = 0U,
    BB_FAULT_HIL_800HZ  = 1U,
    BB_FAULT_OVERLOAD   = 2U,
    BB_FAULT_LOOSENESS  = 3U,
    BB_FAULT_OVERTEMP   = 4U,
    BB_FAULT_IWDG_RESET = 5U,
    BB_FAULT_BKP_MIG    = 6U,
    BB_FAULT_STRESS     = 7U
} BB_FaultType_t;

/* 32 字节故障摘要记录（4 字节自然对齐，无填充空洞） */
typedef struct {
    uint16_t magic;         /* 0xA55A */
    uint16_t snapshot_idx;  /* 关联快照槽位 (0..63), 0xFFFF 表示无快照 */
    uint32_t seq;           /* 单调递增序号 (由 BB_Core 自动填入) */
    uint32_t timestamp_ms;  /* 系统 tick */
    float    peak_freq_hz;  /* 主峰频率 (Hz) */
    float    z_score;       /* 异常得分 (HIL 模式下可存 800Hz 幅值) */
    float    rms_mg;        /* 时域 RMS (mg) 或 HIL 主峰幅值 (V) */
    int16_t  temp_x10;      /* 温度 x10 (℃) */
    uint8_t  fault_type;    /* BB_FaultType_t */
    uint8_t  mode;          /* 0=HIL, 1=PHY */
    uint8_t  reset_reason;  /* RCC->CSR 编码 */
    uint8_t  commit_flag;   /* Phase 2 最后写入: 0x5A */
    uint16_t crc16;         /* Phase 2 最后写入: 覆盖前 30 字节 */
} BlackboxRecord_t;

/* 1024 字节故障频谱快照（4 字节自然对齐，每扇区整存 4 条） */
typedef struct {
    uint16_t magic;                  /* 0x5AA5 */
    uint16_t seq_low;                /* 关联摘要 seq 低 16 位 */
    uint32_t timestamp_ms;
    float    fs_hz;                  /* 采样率 */
    float    scale_max;              /* q15 满量程对应的实际物理幅值 */
    int16_t  spec_q15[BB_SNAP_BINS]; /* 502 点归一化频谱 */
    uint16_t commit_magic;           /* Phase 2 最后写入: 0xAA55 */
    uint16_t crc16;                  /* Phase 2 最后写入: 覆盖前 1022 字节 */
} BlackboxSnapshot_t;

/* Keil AC5 / C99 兼容编译期大小断言 */
typedef char bb_rec_size_chk[(sizeof(BlackboxRecord_t) == 32U) ? 1 : -1];
typedef char bb_snap_size_chk[(sizeof(BlackboxSnapshot_t) == 1024U) ? 1 : -1];

/* 底层 Flash 抽象接口（供 STM32 NM25Q128 与 PC 单元测试复用） */
typedef int (*BB_ReadFn)(uint32_t addr, uint8_t *buf, uint32_t len, void *user_ctx);
typedef int (*BB_WriteFn)(uint32_t addr, const uint8_t *buf, uint32_t len, void *user_ctx);
typedef int (*BB_EraseSectorFn)(uint32_t sector_addr, void *user_ctx);

typedef struct {
    BB_ReadFn        read;
    BB_WriteFn       write;
    BB_EraseSectorFn erase_sector;
    void            *user_ctx;
} BB_FlashOps_t;

/* 黑匣子运行时上下文 */
typedef struct {
    BB_FlashOps_t ops;
    uint32_t      next_seq;
    uint16_t      rec_write_slot;
    uint16_t      rec_valid_cnt;
    uint16_t      rec_corrupt_cnt;
    uint16_t      snap_write_slot;
    uint16_t      snap_valid_cnt;
    uint16_t      snap_corrupt_cnt;
    uint8_t       mounted;
} BB_Context_t;

uint16_t BB_Core_CRC16(const uint8_t *data, uint32_t len);
uint8_t  BB_Core_ValidateRecord(const BlackboxRecord_t *rec);
uint8_t  BB_Core_ValidateSnapshot(const BlackboxSnapshot_t *snap);

int BB_Core_Mount(BB_Context_t *ctx, const BB_FlashOps_t *ops);
int BB_Core_FormatAll(BB_Context_t *ctx);
int BB_Core_AppendRecord(BB_Context_t *ctx, BlackboxRecord_t *rec);
int BB_Core_AppendSnapshot(BB_Context_t *ctx, BlackboxSnapshot_t *snap, uint16_t *out_slot);
int BB_Core_ReadRecentRecords(BB_Context_t *ctx,
                              BlackboxRecord_t *out_arr,
                              uint32_t max_count,
                              uint32_t *out_count);
int BB_Core_ReadSnapshot(BB_Context_t *ctx, uint16_t slot_idx, BlackboxSnapshot_t *out_snap);

#ifdef __cplusplus
}
#endif

#endif /* __BB_CORE_H */
