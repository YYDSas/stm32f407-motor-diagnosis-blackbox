#include "app_dsp.h"
#include "app_cli.h"
#include "bsp_dwt.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include <math.h>

#define FFT_LEN          1024U
#define ADC_FS_HZ        10240.16f
#define DAC_LUT_LEN      128U
#define FAULT_800_TH_V   0.25f   /* 800Hz 谐波幅值越限阈值 (V) */

/* 大数组全部放静态全局区 (.bss), 严禁放任务栈防止爆栈 */
static uint16_t  s_adc_dma_buf[FFT_LEN * 2];
static uint16_t  s_dac_norm_lut[DAC_LUT_LEN];
static uint16_t  s_dac_fault_lut[DAC_LUT_LEN];

static float32_t s_fft_in[FFT_LEN];
static float32_t s_fft_out[FFT_LEN];
static float32_t s_fft_mag[FFT_LEN / 2];
static float32_t s_hanning[FFT_LEN];

static arm_rfft_fast_instance_f32 s_fft_inst;
static TaskHandle_t s_fft_task_handle = NULL;
static volatile DSP_Result_t s_result = {0};

/* 预计算 128 点 DAC 波形表与 1024 点汉宁窗 */
static void DSP_BuildTables(void)
{
    uint32_t i;
    for (i = 0; i < DAC_LUT_LEN; i++)
    {
        float rad = 2.0f * PI * (float)i / (float)DAC_LUT_LEN;
        /* 1.65V 直流偏置 (2048), 正常波 200Hz (±0.9V) */
        float v_norm  = 1.65f + 0.90f * arm_sin_f32(rad);
        /* 故障波: 200Hz (±0.7V) + 800Hz 异常四倍频谐波 (±0.5V) */
        float v_fault = 1.65f + 0.70f * arm_sin_f32(rad)
                              + 0.50f * arm_sin_f32(4.0f * rad);

        s_dac_norm_lut[i]  = (uint16_t)(v_norm  * 4095.0f / 3.3f);
        s_dac_fault_lut[i] = (uint16_t)(v_fault * 4095.0f / 3.3f);
    }
    for (i = 0; i < FFT_LEN; i++)
    {
        s_hanning[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float)i / (float)(FFT_LEN - 1)));
    }
}

void APP_DSP_SetSignalMode(uint8_t fault_mode)
{
    uint16_t *lut = fault_mode ? s_dac_fault_lut : s_dac_norm_lut;
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)lut, DAC_LUT_LEN, DAC_ALIGN_12B_R);
    s_result.sig_mode = fault_mode ? 1U : 0U;
}

void APP_DSP_GetResult(DSP_Result_t *out)
{
    if (out != NULL)
    {
        taskENTER_CRITICAL();
        *out = s_result;
        taskEXIT_CRITICAL();
    }
}

/* ADC DMA 半满回调 -> 通知 FFT 处理前半区 [0..1023] */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    BaseType_t xWoken = pdFALSE;
    if (hadc->Instance == ADC1 && s_fft_task_handle != NULL)
    {
        xTaskNotifyFromISR(s_fft_task_handle, 0x01U, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

/* ADC DMA 全满回调 -> 通知 FFT 处理后半区 [1024..2047] */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    BaseType_t xWoken = pdFALSE;
    if (hadc->Instance == ADC1 && s_fft_task_handle != NULL)
    {
        xTaskNotifyFromISR(s_fft_task_handle, 0x02U, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

static void DSP_ProcessHalf(const uint16_t *raw)
{
    uint32_t i, max_idx = 0;
    float32_t mean = 0.0f, max_val = 0.0f;
    uint32_t t0 = BSP_DWT_GetUs();

    for (i = 0; i < FFT_LEN; i++) mean += (float32_t)raw[i];
    mean /= (float32_t)FFT_LEN;

    /* 去直流 + 转电压 + 乘汉宁窗 (补偿系数 x2) */
    for (i = 0; i < FFT_LEN; i++)
    {
        float32_t v = ((float32_t)raw[i] - mean) * (3.3f / 4095.0f);
        s_fft_in[i] = v * s_hanning[i];
    }

    arm_rfft_fast_f32(&s_fft_inst, s_fft_in, s_fft_out, 0);
    arm_cmplx_mag_f32(s_fft_out, s_fft_mag, FFT_LEN / 2);

    /* 从 bin=2 (20Hz) 起寻主峰, 避开直流泄漏 */
    arm_max_f32(&s_fft_mag[2], (FFT_LEN / 2) - 2, &max_val, &max_idx);

    /* 检查 800Hz 附近 (bin 79..81) 最大幅值 */
    float32_t h800 = 0.0f;
    uint32_t  h800_idx = 0;
    arm_max_f32(&s_fft_mag[79], 3, &h800, &h800_idx);

    float32_t peak_hz = (float32_t)(max_idx + 2U) * (ADC_FS_HZ / (float32_t)FFT_LEN);
    float32_t peak_v  = (max_val * 4.0f) / (float32_t)FFT_LEN;
    float32_t h800_v  = (h800    * 4.0f) / (float32_t)FFT_LEN;
    uint32_t  cost_us = BSP_DWT_GetUs() - t0;

    taskENTER_CRITICAL();
    s_result.peak_freq_hz   = peak_hz;
    s_result.peak_amp_v     = peak_v;
    s_result.harm_800_amp_v = h800_v;
    s_result.fft_cost_us    = cost_us;
    s_result.fault_flag     = (h800_v > FAULT_800_TH_V) ? 1U : 0U;
    taskEXIT_CRITICAL();
}

static void Task_DSP_FFT(void *arg)
{
    uint32_t notify_val = 0;
    (void)arg;

    DSP_BuildTables();
    arm_rfft_fast_init_f32(&s_fft_inst, FFT_LEN);

    /* 启动 DAC1 波形输出与 ADC1 DMA 双缓冲采集 */
    HAL_TIM_Base_Start(&htim6);
    APP_DSP_SetSignalMode(0);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma_buf, FFT_LEN * 2);
    HAL_TIM_Base_Start(&htim2);

    for (;;)
    {
        xTaskNotifyWait(0, 0xFFFFFFFFU, &notify_val, portMAX_DELAY);
        if (notify_val & 0x01U) DSP_ProcessHalf(&s_adc_dma_buf[0]);
        if (notify_val & 0x02U) DSP_ProcessHalf(&s_adc_dma_buf[FFT_LEN]);
    }
}

void APP_DSP_Init(void)
{
    /* 优先级 28 (高于 defaultTask 的 24 与 Task_CLI 的 2) */
    xTaskCreate(Task_DSP_FFT, "Task_DSP_FFT", 512, NULL, tskIDLE_PRIORITY + 28, &s_fft_task_handle);
}
