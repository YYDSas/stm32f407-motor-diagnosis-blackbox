#ifndef __APP_DSP_H
#define __APP_DSP_H

#include "app_baseline.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    peak_freq_hz;    /* 主峰频率 (Hz) */
    float    peak_amp_v;      /* HIL 主峰幅值 (V) 或 PHY 时域 RMS (mg) */
    float    harm_800_amp_v;  /* HIL 800Hz 幅值 (V) 或 PHY 综合 z-score */
    float    kurtosis;        /* PHY 时域峭度 */
    float    crest_factor;    /* PHY 峰值因子 */
    uint32_t fft_cost_us;     /* DWT 实测 1024 点 FFT 耗时 (us) */
    uint32_t phy_ovr_cnt;     /* ADXL345 FIFO 累计溢出次数 */
    uint32_t phy_frame_cnt;   /* 物理模式已处理帧数 */
    uint8_t  fault_flag;      /* 1 = 当前处于故障告警态 */
    uint8_t  fault_code;      /* 故障类型码 (BB_FaultType_t) */
    uint8_t  sig_mode;        /* HIL: 0=200Hz正常, 1=200+800Hz故障 */
    uint8_t  chan_mode;       /* 0 = HIL 通道, 1 = PHY (ADXL345 3200Hz) */
    uint8_t  base_state;      /* BaselineState_t */
    uint8_t  dominant_feat;   /* 主导异常特征编号 (0..7) */
    uint16_t cal_progress;    /* 基线已学习帧数 */
    uint16_t cal_target;      /* 基线目标学习帧数 */
} DSP_Result_t;

void APP_DSP_Init(void);
void APP_DSP_SetSignalMode(uint8_t fault_mode);
uint8_t APP_DSP_SetChannelMode(uint8_t phy_mode);
void APP_DSP_TriggerCalibration(uint16_t frames);
void APP_DSP_GetResult(DSP_Result_t *out);
void APP_DSP_GetLatestSpectrum(float *out_mag, uint16_t max_len);
SemaphoreHandle_t APP_DSP_GetSpi2Mutex(void);

/* 在 main.c HAL_GPIO_EXTI_Callback(GPIO_PIN_0) 中调用，唤醒 Task_DSP_FFT */
void APP_DSP_OnAdxlExtiFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DSP_H */
