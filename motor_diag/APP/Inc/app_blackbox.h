#ifndef __APP_BLACKBOX_H
#define __APP_BLACKBOX_H

#include "bb_core.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化黑匣子互斥量、队列并创建 Task_Blackbox (优先级 20)
 */
void APP_Blackbox_Init(void);

/**
 * @brief  非阻塞提交一条故障事件（可选附带频谱幅值数组，自动归一化压缩为 502 点 q15）
 * @param  rec      摘要记录（magic/seq/commit_flag/crc16/snapshot_idx 由黑匣子自动填充）
 * @param  fft_mag  单边频谱幅值数组指针（为 NULL 时仅写摘要，不写快照）
 * @param  mag_len  fft_mag 长度（通常为 512）
 * @param  fs_hz    当前采样率 (Hz)
 * @retval 1: 入队成功；0: 队列/对象池满或未就绪
 */
uint8_t APP_Blackbox_PushFault(const BlackboxRecord_t *rec,
                               const float *fft_mag,
                               uint16_t mag_len,
                               float fs_hz);

/**
 * @brief  CLI 命令支持：打印最近 max_n 条摘要记录（0 表示默认打印 20 条）
 */
void APP_Blackbox_PrintDump(uint32_t max_n);

/**
 * @brief  CLI 命令支持：打印指定槽位 (0..63) 的 502 点频谱快照
 */
void APP_Blackbox_PrintSnapshot(uint16_t slot_idx);

/**
 * @brief  CLI 命令支持：擦除全部摘要区与快照区并重置游标
 */
void APP_Blackbox_ClearAll(void);

/**
 * @brief  CLI 命令支持：开启/关闭 20 ms 周期写入压测（用于 50 次拔电撕裂测试）
 */
void APP_Blackbox_SetStressMode(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BLACKBOX_H */
