#ifndef __APP_DSP_H
#define __APP_DSP_H

#include <stdint.h>

typedef struct {
    float    peak_freq_hz;    /* 主峰频率 (Hz) */
    float    peak_amp_v;      /* 主峰幅值 (V) */
    float    harm_800_amp_v;  /* 800Hz 异常谐波幅值 (V) */
    uint32_t fft_cost_us;     /* DWT 实测 1024 点 FFT 耗时 (us) */
    uint8_t  fault_flag;      /* 1 = 检出异常谐波越限 */
    uint8_t  sig_mode;        /* 0 = 200Hz 正常波, 1 = 200+800Hz 故障波 */
} DSP_Result_t;

void APP_DSP_Init(void);
void APP_DSP_SetSignalMode(uint8_t fault_mode);
void APP_DSP_GetResult(DSP_Result_t *out);

#endif
