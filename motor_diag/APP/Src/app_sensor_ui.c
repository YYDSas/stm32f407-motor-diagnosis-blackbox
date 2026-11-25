#include "app_sensor_ui.h"
#include "app_dsp.h"
#include "app_blackbox.h"
#include "app_cli.h"
#include "bsp_ds18b20.h"
#include "bsp_oled.h"
#include "bsp_bkpsram.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

static volatile int16_t s_temp_x10    = 0;
static volatile uint8_t s_temp_valid  = 0U;
static volatile int16_t s_temp_th_x10 = 350; /* 默认 35.0℃ 过温告警（手指捏热即可触发） */

/* 静态频谱缓存放 .bss，避免占 Task_UI_OLED 栈 */
static float s_ui_mag[512];

static void Task_Sensor(void *arg)
{
    uint32_t last_ot_tick = 0U;
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(300));
    Log_Printf("[TEMP] DS18B20 detect=%s, overtemp_th=%.1f C\r\n",
               BSP_DS18B20_Detect() ? "OK" : "NOT_FOUND",
               (float)s_temp_th_x10 / 10.0f);

    for (;;)
    {
        if (BSP_DS18B20_StartConvert() != 0U)
        {
            int16_t t_x10 = 0;
            vTaskDelay(pdMS_TO_TICKS(800)); /* 12-bit 转换最大 750 ms */

            if (BSP_DS18B20_ReadTempX10(&t_x10) != 0U)
            {
                s_temp_x10   = t_x10;
                s_temp_valid = 1U;

                if ((t_x10 >= s_temp_th_x10) && ((HAL_GetTick() - last_ot_tick) >= 2000U))
                {
                    DSP_Result_t dr;
                    BlackboxRecord_t rec;
                    last_ot_tick = HAL_GetTick();

                    APP_DSP_GetResult(&dr);
                    BSP_BkpSram_Freeze((uint8_t)BB_FAULT_OVERTEMP, 0U);

                    memset(&rec, 0, sizeof(rec));
                    rec.timestamp_ms = last_ot_tick;
                    rec.peak_freq_hz = dr.peak_freq_hz;
                    rec.z_score      = (float)t_x10 / 10.0f;
                    rec.rms_mg       = dr.peak_amp_v;
                    rec.temp_x10     = t_x10;
                    rec.fault_type   = (uint8_t)BB_FAULT_OVERTEMP;
                    rec.mode         = dr.chan_mode;
                    APP_Blackbox_PushFault(&rec, NULL, 0U, 0.0f);

                    Log_Printf("[TEMP] OVERTEMP ALERT! T=%.1f C >= %.1f C\r\n",
                               (float)t_x10 / 10.0f, (float)s_temp_th_x10 / 10.0f);
                }
            }
        }
        else
        {
            s_temp_valid = 0U;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void Task_UI_OLED(void *arg)
{
    SemaphoreHandle_t spi2_mtx;
    uint8_t bars[64];
    char line0[24];
    char line1[24];
    uint8_t p;
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(200));
    spi2_mtx = APP_DSP_GetSpi2Mutex();

    if ((spi2_mtx != NULL) && (xSemaphoreTake(spi2_mtx, pdMS_TO_TICKS(500)) == pdTRUE))
    {
        BSP_OLED_Init();
        xSemaphoreGive(spi2_mtx);
    }

    for (;;)
    {
        DSP_Result_t r;
        int16_t t_x10 = 0;
        uint8_t t_ok;
        float max_bin = 1e-3f;
        uint8_t b;

        APP_DSP_GetResult(&r);
        APP_DSP_GetLatestSpectrum(s_ui_mag, 512U);
        t_ok = APP_SensorUI_GetTempX10(&t_x10);

        /* 1. 组装两行状态文本（Page 0 与 Page 1） */
        {
            const char *m_str = (r.chan_mode != 0U) ? "PHY" : "HIL";
            const char *s_str = "NORM";
            if ((t_ok != 0U) && (t_x10 >= s_temp_th_x10))
            {
                s_str = "OT!!";
            }
            else if ((r.chan_mode != 0U) && (r.base_state == (uint8_t)BASE_STATE_CALIBRATING))
            {
                s_str = "CAL ";
            }
            else if (r.fault_flag != 0U)
            {
                s_str = "FLT!";
            }

            if (t_ok != 0U)
            {
                snprintf(line0, sizeof(line0), "%s %s T:%2d.%1dC",
                         m_str, s_str, (int)(t_x10 / 10), (int)((t_x10 >= 0 ? t_x10 : -t_x10) % 10));
            }
            else
            {
                snprintf(line0, sizeof(line0), "%s %s T: --.-C", m_str, s_str);
            }

            if (r.chan_mode != 0U)
            {
                snprintf(line1, sizeof(line1), "PK:%4u Z:%4.1f",
                         (unsigned int)(r.peak_freq_hz + 0.5f), r.harm_800_amp_v);
            }
            else
            {
                snprintf(line1, sizeof(line1), "PK:%4u H8:%.2f",
                         (unsigned int)(r.peak_freq_hz + 0.5f), r.harm_800_amp_v);
            }
        }

        /* 2. 将 512 点频谱压缩为 64 柱 (每 8 个 bin 取峰值，自动量程归一化到 0..48px) */
        for (b = 0U; b < 64U; b++)
        {
            uint16_t start = (uint16_t)(b * 8U);
            uint16_t k;
            float pk = 0.0f;
            if (start < 2U) start = 2U; /* 避开直流 bin 0..1 */
            for (k = start; k < (uint16_t)((b + 1U) * 8U); k++)
            {
                if (s_ui_mag[k] > pk) pk = s_ui_mag[k];
            }
            if (pk > max_bin) max_bin = pk;
        }
        for (b = 0U; b < 64U; b++)
        {
            uint16_t start = (uint16_t)(b * 8U);
            uint16_t k;
            float pk = 0.0f;
            if (start < 2U) start = 2U;
            for (k = start; k < (uint16_t)((b + 1U) * 8U); k++)
            {
                if (s_ui_mag[k] > pk) pk = s_ui_mag[k];
            }
            bars[b] = (uint8_t)((pk / max_bin) * 46.0f + 0.5f);
        }

        /* 3. 先在纯内存中渲染帧缓冲，再按页短时加锁刷写 SPI2（每页 ~0.4ms，绝不阻塞 ADXL345） */
        BSP_OLED_Clear();
        BSP_OLED_DrawStr(0U, 0U, line0);
        BSP_OLED_DrawStr(1U, 0U, line1);
        BSP_OLED_DrawSpectrum64(bars);

        if (spi2_mtx != NULL)
        {
            for (p = 0U; p < OLED_PAGES; p++)
            {
                if (xSemaphoreTake(spi2_mtx, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    BSP_OLED_FlushPage(p);
                    xSemaphoreGive(spi2_mtx);
                }
                taskYIELD();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void APP_SensorUI_Init(void)
{
    xTaskCreate(Task_Sensor,  "Task_Sensor",  384, NULL, tskIDLE_PRIORITY + 22, NULL);
    xTaskCreate(Task_UI_OLED, "Task_UI_OLED", 384, NULL, tskIDLE_PRIORITY + 10, NULL);
}

uint8_t APP_SensorUI_GetTempX10(int16_t *out_temp_x10)
{
    if (out_temp_x10 != NULL)
    {
        *out_temp_x10 = s_temp_x10;
    }
    return s_temp_valid;
}

void APP_SensorUI_SetTempThX10(int16_t th_x10)
{
    s_temp_th_x10 = th_x10;
}

int16_t APP_SensorUI_GetTempThX10(void)
{
    return s_temp_th_x10;
}
