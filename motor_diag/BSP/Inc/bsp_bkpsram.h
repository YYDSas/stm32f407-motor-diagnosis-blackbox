#ifndef __BSP_BKPSRAM_H
#define __BSP_BKPSRAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BKPSRAM_MAGIC          (0x424B5032U)  /* "BKP2" */
#define BKPSRAM_FRAME_HIST     (32U)          /* 故障前最近 32 帧特征轨迹 */
#define BKPSRAM_RAW_SAMPLES    (1024U)        /* 故障前最近 1024 点 q15 原始波形 */

/* BKPSRAM 状态机定义 */
typedef enum {
    BKP_STATE_RECORDING         = 0x01U,  /* 正常运行：滚动录波 */
    BKP_STATE_FROZEN_UNMIGRATED = 0xA5U,  /* 故障冻结：等待上电迁移至 Flash */
    BKP_STATE_MIGRATED          = 0x5AU   /* 已完成迁移 */
} BkpsramState_t;

/* 单帧微型特征轨迹（精确 32 字节，4 字节自然对齐） */
typedef struct {
    uint32_t tick_ms;
    float    z_score;
    float    rms_mg;
    float    peak_freq_hz;
    float    kurtosis;
    int16_t  temp_x10;
    uint16_t flags;
    uint32_t reserved[2];
} BkpFrameMini_t;

/* 4 KB 备份 SRAM 布局（总计 3104 B <= 4096 B） */
typedef struct {
    uint32_t       magic;
    uint32_t       seq;
    uint32_t       freeze_tick_ms;
    uint16_t       frame_head;     /* 下一个写入位置: 0..31 */
    uint16_t       frame_count;    /* 有效帧数量: 0..32 */
    uint16_t       raw_head;       /* 下一个写入位置: 0..1023 */
    uint16_t       raw_count;      /* 有效采样点数量: 0..1024 */
    uint8_t        state;          /* BkpsramState_t */
    uint8_t        fault_type;
    uint8_t        reset_csr;
    uint8_t        reserved0;
    uint16_t       payload_crc16;  /* 覆盖 frames[] + raw_q15[] */
    uint16_t       header_crc16;   /* 覆盖 magic..payload_crc16 (前 26 字节) */
    uint32_t       reserved1;
    BkpFrameMini_t frames[BKPSRAM_FRAME_HIST];
    int16_t        raw_q15[BKPSRAM_RAW_SAMPLES];
} BkpsramArea_t;

/* Keil AC5 / C99 兼容编译期大小断言 */
typedef char bkp_frame_size_chk[(sizeof(BkpFrameMini_t) == 32U) ? 1 : -1];
typedef char bkp_area_size_chk[(sizeof(BkpsramArea_t) <= 4096U) ? 1 : -1];

/**
 * @brief  使能 PWR 备份域与 4KB BKPSRAM 稳压器，并检查掉电前现场数据
 * @retval 1: 检测到合法的故障冻结现场（等待迁移至 Flash）；0: 正常录波态
 */
uint8_t BSP_BkpSram_Init(void);

/**
 * @brief  滚动写入一帧特征轨迹与对应原始波形块（仅在 RECORDING 状态下生效）
 */
void BSP_BkpSram_PushFrame(const BkpFrameMini_t *frame,
                           const int16_t *raw_chunk,
                           uint16_t raw_len);

/**
 * @brief  故障触发时立即冻结 BKPSRAM 并计算双 CRC16
 */
void BSP_BkpSram_Freeze(uint8_t fault_type, uint8_t reset_csr);

/**
 * @brief  校验当前 BKPSRAM 是否存在完整有效的未迁移冻结现场
 * @retval 1: 有效；0: 无效或未冻结
 */
uint8_t BSP_BkpSram_IsFrozenValid(void);

/**
 * @brief  Flash 迁移完成后清除冻结态并恢复滚动录波（保留单调递增 seq）
 */
void BSP_BkpSram_MarkMigratedAndReset(void);

/**
 * @brief  CLI 测试辅助函数：写入确定性测试波形并冻结，用于 10 分钟断电保活验证
 */
void BSP_BkpSram_FillTestPatternAndFreeze(void);

/**
 * @brief  获取只读指针用于 CLI 打印或迁移至 Flash
 */
const BkpsramArea_t *BSP_BkpSram_GetArea(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BKPSRAM_H */
