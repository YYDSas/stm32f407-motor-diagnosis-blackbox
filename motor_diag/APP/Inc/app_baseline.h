#ifndef __APP_BASELINE_H
#define __APP_BASELINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BASELINE_FEAT_DIM        (8U)
#define BASELINE_DEFAULT_FRAMES  (30U)    /* 30 帧 * 160 ms ≈ 4.8 s 健康自学习 */
#define BASELINE_TH_HIGH_DEFAULT (4.0f)   /* 进入告警阈值 (z-score) */
#define BASELINE_TH_LOW_DEFAULT  (2.5f)   /* 退出告警滞回阈值 */
#define BASELINE_EXIT_CONSEC     (5U)     /* 连续 5 帧低于 TH_LOW 才恢复正常 */

/* 8 维时频特征索引定义 */
typedef enum {
    FEAT_RMS_MG     = 0U,  /* 时域 RMS (mg) */
    FEAT_KURTOSIS   = 1U,  /* 时域峭度 */
    FEAT_CREST      = 2U,  /* 峰值因子 */
    FEAT_STEP_1X    = 3U,  /* 步进基频 f_step 能量 */
    FEAT_STEP_2X    = 4U,  /* 2 次谐波能量 */
    FEAT_STEP_3X    = 5U,  /* 3 次谐波能量 */
    FEAT_LOW_BAND   = 6U,  /* 10-150 Hz 松动宽带能量 */
    FEAT_HIGH_BAND  = 7U   /* 900-1500 Hz 高频冲击能量 */
} BaselineFeatIdx_t;

typedef enum {
    BASE_STATE_UNINIT      = 0U,
    BASE_STATE_CALIBRATING = 1U,
    BASE_STATE_NORMAL      = 2U,
    BASE_STATE_FAULT       = 3U
} BaselineState_t;

typedef struct {
    uint8_t  state;
    uint16_t cal_target;
    uint16_t cal_count;
    uint8_t  exit_consec;
    float    th_high;
    float    th_low;
    float    mean[BASELINE_FEAT_DIM];
    float    m2[BASELINE_FEAT_DIM];
    float    std[BASELINE_FEAT_DIM];
    float    min_std[BASELINE_FEAT_DIM];
    float    weight[BASELINE_FEAT_DIM];
} BaselineModel_t;

typedef struct {
    uint8_t  state;          /* BaselineState_t */
    uint8_t  dominant_feat;  /* 主导异常特征索引 (0..7) */
    uint8_t  fault_code;     /* 映射后的故障码: 0=正常, 2=过载/失步, 3=松动 */
    uint16_t cal_progress;   /* 已学习帧数 */
    uint16_t cal_target;     /* 目标学习帧数 */
    float    max_z;          /* 综合异常得分 max(w_k * z_k) */
} BaselineEval_t;

/**
 * @brief  初始化自学习基线模型并进入校准态
 * @param  cal_frames: 健康学习帧数（0 表示使用默认 30 帧）
 */
void APP_Baseline_StartCal(BaselineModel_t *m, uint16_t cal_frames);

/**
 * @brief  输入一帧 8 维特征，执行 Welford 在线学习或 z-score 滞回判定
 */
void APP_Baseline_Update(BaselineModel_t *m,
                         const float feat[BASELINE_FEAT_DIM],
                         BaselineEval_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BASELINE_H */
