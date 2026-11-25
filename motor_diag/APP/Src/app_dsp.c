#include "app_dsp.h"
#include "stm32f4xx_hal.h"
#include "app_cli.h"
#include "app_blackbox.h"
#include "bsp_dwt.h"
#include "bsp_bkpsram.h"
#include "bsp_adxl345.h"
#include "bsp_stepper.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include "app_supervisor.h"
#define FFT_LEN          1024U
#define PHY_HOP_LEN      512U      /* 50% 重叠滑窗: 512 / 3200 = 160 ms/帧 */
#define ADC_FS_HZ        10240.16f
#define PHY_FS_HZ        3200.0f
#define DAC_LUT_LEN      128U
#define FAULT_800_TH_V   0.25f

/* 大数组全部放静态 .bss，严禁放任务栈 */
static uint16_t  s_adc_dma_buf[FFT_LEN * 2];
static uint16_t  s_dac_norm_lut[DAC_LUT_LEN];
static uint16_t  s_dac_fault_lut[DAC_LUT_LEN];

static float32_t s_fft_in[FFT_LEN];
static float32_t s_fft_out[FFT_LEN];
static float32_t s_fft_mag[FFT_LEN / 2];
static float32_t s_hanning[FFT_LEN];

/* 物理通道 50% 重叠滑窗缓冲（采用实测 +14.5dB 高信噪比的 X 轴） */
static int16_t   s_phy_prev_half[PHY_HOP_LEN];
static int16_t   s_phy_curr_half[PHY_HOP_LEN];
static uint16_t  s_phy_fill_idx   = 0U;
static uint8_t   s_phy_primed     = 0U;
static uint32_t  s_phy_ovr_total  = 0U;
static uint32_t  s_phy_frame_cnt  = 0U;

static arm_rfft_fast_instance_f32 s_fft_inst;
static TaskHandle_t               s_fft_task_handle = NULL;
static SemaphoreHandle_t          s_spi2_mutex      = NULL;
static BaselineModel_t            s_baseline;
static volatile uint8_t           s_chan_mode       = 0U; /* 0=HIL, 1=PHY */
static volatile DSP_Result_t      s_result;

static void DSP_BuildTables(void)
{
    uint32_t i;
    for (i = 0U; i < DAC_LUT_LEN; i++)
    {
        float rad = 2.0f * PI * (float)i / (float)DAC_LUT_LEN;
        float v_norm  = 1.65f + 0.90f * arm_sin_f32(rad);
        float v_fault = 1.65f + 0.70f * arm_sin_f32(rad)
                              + 0.50f * arm_sin_f32(4.0f * rad);

        s_dac_norm_lut[i]  = (uint16_t)(v_norm  * 4095.0f / 3.3f);
        s_dac_fault_lut[i] = (uint16_t)(v_fault * 4095.0f / 3.3f);
    }
    for (i = 0U; i < FFT_LEN; i++)
    {
        s_hanning[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float)i / (float)(FFT_LEN - 1U)));
    }
}

void APP_DSP_SetSignalMode(uint8_t fault_mode)
{
    uint16_t *lut = fault_mode ? s_dac_fault_lut : s_dac_norm_lut;
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)lut, DAC_LUT_LEN, DAC_ALIGN_12B_R);
    s_result.sig_mode = fault_mode ? 1U : 0U;
}

uint8_t APP_DSP_SetChannelMode(uint8_t phy_mode)
{
    if (phy_mode != 0U)
    {
        uint8_t ok = 0U;
        if ((s_spi2_mutex != NULL) &&
            (xSemaphoreTake(s_spi2_mutex, pdMS_TO_TICKS(500)) == pdTRUE))
        {
            /* 实测百问网 ADXL345 GPIO 引脚硬连至 INT1 (use_int2=0) */
            ok = BSP_ADXL345_Init(0U);
            xSemaphoreGive(s_spi2_mutex);
        }
        if (ok == 0U)
        {
            return 0U;
        }

        taskENTER_CRITICAL();
        s_phy_fill_idx  = 0U;
        s_phy_primed    = 0U;
        s_phy_ovr_total = 0U;
        s_phy_frame_cnt = 0U;
        s_chan_mode     = 1U;
        s_result.chan_mode = 1U;
        taskEXIT_CRITICAL();

        APP_Baseline_StartCal(&s_baseline, BASELINE_DEFAULT_FRAMES);
        return 1U;
    }
    else
    {
        taskENTER_CRITICAL();
        s_chan_mode        = 0U;
        s_result.chan_mode = 0U;
        taskEXIT_CRITICAL();
        return 1U;
    }
}

void APP_DSP_TriggerCalibration(uint16_t frames)
{
    taskENTER_CRITICAL();
    APP_Baseline_StartCal(&s_baseline, frames);
    taskEXIT_CRITICAL();
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

void APP_DSP_GetLatestSpectrum(float *out_mag, uint16_t max_len)
{
    uint16_t n;
    if ((out_mag == NULL) || (max_len == 0U))
    {
        return;
    }
    n = (max_len < (FFT_LEN / 2U)) ? max_len : (FFT_LEN / 2U);
    taskENTER_CRITICAL();
    memcpy(out_mag, s_fft_mag, (size_t)n * sizeof(float));
    taskEXIT_CRITICAL();
}

SemaphoreHandle_t APP_DSP_GetSpi2Mutex(void)
{
    return s_spi2_mutex;
}

/* ==================== 中断唤醒入口 ==================== */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    BaseType_t xWoken = pdFALSE;
    if ((hadc->Instance == ADC1) && (s_fft_task_handle != NULL) && (s_chan_mode == 0U))
    {
        xTaskNotifyFromISR(s_fft_task_handle, 0x01U, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    BaseType_t xWoken = pdFALSE;
    if ((hadc->Instance == ADC1) && (s_fft_task_handle != NULL) && (s_chan_mode == 0U))
    {
        xTaskNotifyFromISR(s_fft_task_handle, 0x02U, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

void APP_DSP_OnAdxlExtiFromISR(void)
{
    BaseType_t xWoken = pdFALSE;
    BSP_ADXL345_OnExtiIrq();
    if ((s_fft_task_handle != NULL) && (s_chan_mode != 0U))
    {
        xTaskNotifyFromISR(s_fft_task_handle, 0x04U, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

/* ==================== HIL 通道处理 (保留 v0.2/v0.3 验证链路) ==================== */

static void DSP_ProcessHilHalf(const uint16_t *raw)
{
    static uint32_t s_last_bb_tick = 0U;
    uint32_t i, max_idx = 0U, h800_idx = 0U;
    float32_t mean = 0.0f, max_val = 0.0f, h800 = 0.0f;
    uint32_t t0 = BSP_DWT_GetUs();

    for (i = 0U; i < FFT_LEN; i++) mean += (float32_t)raw[i];
    mean /= (float32_t)FFT_LEN;

    for (i = 0U; i < FFT_LEN; i++)
    {
        float32_t v = ((float32_t)raw[i] - mean) * (3.3f / 4095.0f);
        s_fft_in[i] = v * s_hanning[i];
    }

    arm_rfft_fast_f32(&s_fft_inst, s_fft_in, s_fft_out, 0);
    arm_cmplx_mag_f32(s_fft_out, s_fft_mag, FFT_LEN / 2U);

    arm_max_f32(&s_fft_mag[2], (FFT_LEN / 2U) - 2U, &max_val, &max_idx);
    arm_max_f32(&s_fft_mag[79], 3U, &h800, &h800_idx);

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
    s_result.fault_code     = (h800_v > FAULT_800_TH_V) ? (uint8_t)BB_FAULT_HIL_800HZ : 0U;
    taskEXIT_CRITICAL();

    {
        BkpFrameMini_t bf;
        memset(&bf, 0, sizeof(bf));
        bf.tick_ms      = HAL_GetTick();
        bf.z_score      = h800_v;
        bf.rms_mg       = peak_v;
        bf.peak_freq_hz = peak_hz;
        BSP_BkpSram_PushFrame(&bf, NULL, 0U);

        if ((h800_v > FAULT_800_TH_V) && ((HAL_GetTick() - s_last_bb_tick) >= 1000U))
        {
            BlackboxRecord_t rec;
            s_last_bb_tick = HAL_GetTick();
            BSP_BkpSram_Freeze((uint8_t)BB_FAULT_HIL_800HZ, 0U);

            memset(&rec, 0, sizeof(rec));
            rec.timestamp_ms = s_last_bb_tick;
            rec.peak_freq_hz = peak_hz;
            rec.z_score      = h800_v;
            rec.rms_mg       = peak_v;
            rec.fault_type   = (uint8_t)BB_FAULT_HIL_800HZ;
            rec.mode         = 0U;
            APP_Blackbox_PushFault(&rec, s_fft_mag, FFT_LEN / 2U, ADC_FS_HZ);
        }
    }
}

/* ==================== PHY 物理通道 50% 重叠滑窗 FFT + 自学习基线 ==================== */

static float dsp_band_energy_mg(uint16_t bin_lo, uint16_t bin_hi)
{
    uint16_t k;
    float sum_sq = 0.0f;
    if (bin_lo < 2U) bin_lo = 2U;
    if (bin_hi >= (FFT_LEN / 2U)) bin_hi = (FFT_LEN / 2U) - 1U;
    for (k = bin_lo; k <= bin_hi; k++)
    {
        float a = (s_fft_mag[k] * 4.0f) / (float)FFT_LEN;
        sum_sq += a * a;
    }
    return sqrtf(sum_sq);
}

static void DSP_ProcessPhyWindow(void)
{
    static uint32_t s_last_phy_bb_tick = 0U;
    uint32_t i, max_idx = 0U;
    float32_t mean = 0.0f, max_val = 0.0f;
    float32_t sum_sq = 0.0f, sum_p4 = 0.0f, peak_abs = 0.0f;
    float feat[BASELINE_FEAT_DIM];
    BaselineEval_t ev;
    uint16_t f_step;
    uint32_t t0 = BSP_DWT_GetUs();

    /* 1. 拼接 [前 512 点 + 当前 512 点] 计算均值 (mg) */
    for (i = 0U; i < PHY_HOP_LEN; i++)
    {
        mean += (float32_t)s_phy_prev_half[i] * ADXL345_MG_PER_LSB;
        mean += (float32_t)s_phy_curr_half[i] * ADXL345_MG_PER_LSB;
    }
    mean /= (float32_t)FFT_LEN;

    /* 2. 去直流 + 时域 RMS/峭度/峰值因子 + 加汉宁窗 */
    for (i = 0U; i < FFT_LEN; i++)
    {
        int16_t raw_s = (i < PHY_HOP_LEN) ? s_phy_prev_half[i] : s_phy_curr_half[i - PHY_HOP_LEN];
        float32_t x   = (float32_t)raw_s * ADXL345_MG_PER_LSB - mean;
        float32_t ax  = fabsf(x);
        float32_t x2  = x * x;

        sum_sq += x2;
        sum_p4 += x2 * x2;
        if (ax > peak_abs) peak_abs = ax;

        s_fft_in[i] = x * s_hanning[i];
    }

    float rms_mg = sqrtf(sum_sq / (float)FFT_LEN);
    float var    = sum_sq / (float)FFT_LEN;
    float kurt   = (var > 1e-4f) ? ((sum_p4 / (float)FFT_LEN) / (var * var)) : 0.0f;
    float crest  = (rms_mg > 1e-3f) ? (peak_abs / rms_mg) : 0.0f;

    /* 3. 1024 点实数 FFT (3.125 Hz/bin) */
    arm_rfft_fast_f32(&s_fft_inst, s_fft_in, s_fft_out, 0);
    arm_cmplx_mag_f32(s_fft_out, s_fft_mag, FFT_LEN / 2U);
    arm_max_f32(&s_fft_mag[3], (FFT_LEN / 2U) - 3U, &max_val, &max_idx);

    float peak_hz = (float)(max_idx + 3U) * (PHY_FS_HZ / (float)FFT_LEN);

    /* 4. 提取 8 维时频特征 */
    f_step = BSP_Stepper_GetStepHz();
    if (f_step == 0U) f_step = 400U;
    {
        uint16_t b1 = (uint16_t)(((float)f_step / (PHY_FS_HZ / (float)FFT_LEN)) + 0.5f);
        uint16_t b2 = (uint16_t)(b1 * 2U);
        uint16_t b3 = (uint16_t)(b1 * 3U);

        feat[FEAT_RMS_MG]    = rms_mg;
        feat[FEAT_KURTOSIS]  = kurt;
        feat[FEAT_CREST]     = crest;
        feat[FEAT_STEP_1X]   = dsp_band_energy_mg((b1 > 2U) ? (b1 - 2U) : 2U, b1 + 2U);
        feat[FEAT_STEP_2X]   = dsp_band_energy_mg((b2 > 2U) ? (b2 - 2U) : 2U, b2 + 2U);
        feat[FEAT_STEP_3X]   = (b3 < 508U) ? dsp_band_energy_mg(b3 - 2U, b3 + 2U) : 0.0f;
        feat[FEAT_LOW_BAND]  = dsp_band_energy_mg(3U, 48U);    /* ~9.4 - 150 Hz */
        feat[FEAT_HIGH_BAND] = dsp_band_energy_mg(288U, 480U); /* 900 - 1500 Hz */
    }

    /* 5. Welford 自学习基线 + z-score 滞回判定 */
    APP_Baseline_Update(&s_baseline, feat, &ev);

    uint32_t cost_us = BSP_DWT_GetUs() - t0;
    s_phy_frame_cnt++;

    taskENTER_CRITICAL();
    s_result.peak_freq_hz   = peak_hz;
    s_result.peak_amp_v     = rms_mg;
    s_result.harm_800_amp_v = ev.max_z;
    s_result.kurtosis       = kurt;
    s_result.crest_factor   = crest;
    s_result.fft_cost_us    = cost_us;
    s_result.phy_ovr_cnt    = s_phy_ovr_total;
    s_result.phy_frame_cnt  = s_phy_frame_cnt;
    s_result.fault_flag     = (ev.state == (uint8_t)BASE_STATE_FAULT) ? 1U : 0U;
    s_result.fault_code     = ev.fault_code;
    s_result.base_state     = ev.state;
    s_result.dominant_feat  = ev.dominant_feat;
    s_result.cal_progress   = ev.cal_progress;
    s_result.cal_target     = ev.cal_target;
    taskEXIT_CRITICAL();

    /* 6. BKPSRAM 滚动录波 + 故障冻结/黑匣子落盘 (1 s 节流) */
    {
        BkpFrameMini_t bf;
        memset(&bf, 0, sizeof(bf));
        bf.tick_ms      = HAL_GetTick();
        bf.z_score      = ev.max_z;
        bf.rms_mg       = rms_mg;
        bf.peak_freq_hz = peak_hz;
        bf.kurtosis     = kurt;
        BSP_BkpSram_PushFrame(&bf, s_phy_curr_half, PHY_HOP_LEN);

        if ((ev.state == (uint8_t)BASE_STATE_FAULT) && (ev.max_z >= s_baseline.th_high) &&
            ((HAL_GetTick() - s_last_phy_bb_tick) >= 1000U))
        {
            BlackboxRecord_t rec;
            s_last_phy_bb_tick = HAL_GetTick();
            BSP_BkpSram_Freeze(ev.fault_code, 0U);

            memset(&rec, 0, sizeof(rec));
            rec.timestamp_ms = s_last_phy_bb_tick;
            rec.peak_freq_hz = peak_hz;
            rec.z_score      = ev.max_z;
            rec.rms_mg       = rms_mg;
            rec.fault_type   = ev.fault_code;
            rec.mode         = 1U;
            APP_Blackbox_PushFault(&rec, s_fft_mag, FFT_LEN / 2U, PHY_FS_HZ);
        }
    }
}

static void DSP_DrainAdxlFifo(void)
{
    ADXL345_Sample_t batch[32];
    uint8_t ovr = 0U;
    uint8_t n = 0U;
    uint8_t i;

    if ((s_spi2_mutex != NULL) &&
        (xSemaphoreTake(s_spi2_mutex, pdMS_TO_TICKS(50)) == pdTRUE))
    {
        n = BSP_ADXL345_ReadFifoBatch(batch, 32U, &ovr);
        xSemaphoreGive(s_spi2_mutex);
    }
    if (ovr != 0U)
    {
        s_phy_ovr_total++;
    }

    for (i = 0U; i < n; i++)
    {
        s_phy_curr_half[s_phy_fill_idx++] = batch[i].x;
        if (s_phy_fill_idx >= PHY_HOP_LEN)
        {
            if (s_phy_primed != 0U)
            {
                DSP_ProcessPhyWindow();
            }
            else
            {
                s_phy_primed = 1U;
            }
            memcpy(s_phy_prev_half, s_phy_curr_half, sizeof(s_phy_prev_half));
            s_phy_fill_idx = 0U;
        }
    }
}

/* ==================== 主 DSP 任务 ==================== */

static void Task_DSP_FFT(void *arg)
{
    uint32_t notify_val = 0U;
    (void)arg;

    DSP_BuildTables();
    arm_rfft_fast_init_f32(&s_fft_inst, FFT_LEN);
    APP_Baseline_StartCal(&s_baseline, BASELINE_DEFAULT_FRAMES);

    /* 默认以 HIL 模式启动（保证开机兼容 W2/W3 验证），可随时用 'mode phy' 切换 */
    HAL_TIM_Base_Start(&htim6);
    APP_DSP_SetSignalMode(0U);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma_buf, FFT_LEN * 2U);
    HAL_TIM_Base_Start(&htim2);

    for (;;)
    {
        if (APP_Supervisor_IsFftDeadlockInjected() != 0U)
        {
           for (;;) { __NOP(); }
        }
        APP_Supervisor_Heartbeat(SUP_HB_DSP);
        TickType_t wait_t = (s_chan_mode != 0U) ? pdMS_TO_TICKS(20) : portMAX_DELAY;
        BaseType_t got = xTaskNotifyWait(0U, 0xFFFFFFFFU, &notify_val, wait_t);

        if (s_chan_mode == 0U)
        {
            if (got == pdTRUE)
            {
                if ((notify_val & 0x01U) != 0U) DSP_ProcessHilHalf(&s_adc_dma_buf[0]);
                if ((notify_val & 0x02U) != 0U) DSP_ProcessHilHalf(&s_adc_dma_buf[FFT_LEN]);
            }
        }
        else
        {
            /* 收到 EXTI0 通知或 20ms 防卡死兜底超时均抽取 FIFO */
            if ((got == pdFALSE) || ((notify_val & 0x04U) != 0U))
            {
                DSP_DrainAdxlFifo();
            }
        }
    }
}

void APP_DSP_Init(void)
{
    memset((void *)&s_result, 0, sizeof(s_result));
    s_spi2_mutex = xSemaphoreCreateMutex();
    xTaskCreate(Task_DSP_FFT, "Task_DSP_FFT", 640, NULL, tskIDLE_PRIORITY + 28, &s_fft_task_handle);
}
