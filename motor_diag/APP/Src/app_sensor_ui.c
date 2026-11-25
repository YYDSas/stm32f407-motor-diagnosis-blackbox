/**
 ****************************************************************************************************
 * @file        app_sensor_ui.c
 * @brief       Task_Sensor (DS18B20 过温监测, 优先级 22) 与 Task_UI_LCD (3.5寸 480x320 彩色频谱仪, 优先级 10)
 * @note        1. FSMC 并行总线独立于 SPI2，Task_UI_LCD 无需竞争 xSpi2Mutex，ADXL345 独享 SPI2
 *              2. 96 柱频谱采用差分矩形刷新 + 峰值保持小红帽，10 FPS 流畅无闪烁
 ****************************************************************************************************
 */

#include "app_sensor_ui.h"
#include "app_dsp.h"
#include "app_blackbox.h"
#include "app_cli.h"
#include "bsp_ds18b20.h"
#include "bsp_bkpsram.h"
#include "lcd.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_supervisor.h"
#include <stdio.h>
#include <string.h>

/* 布局常量 (面向 480x320 横屏；兼容更大分辨率居中/左上对齐) */
#define UI_BAR_COUNT        96U
#define UI_SPEC_X0          12U
#define UI_SPEC_Y0          46U
#define UI_SPEC_W           292U   /* 96 * 3 = 288 px 内部绘图区 + 左右 2px 边距 */
#define UI_SPEC_H           224U
#define UI_SPEC_BOTTOM      (UI_SPEC_Y0 + UI_SPEC_H - 2U)
#define UI_BAR_MAX_H        (UI_SPEC_H - 8U)

#define UI_PANEL_X0         314U
#define UI_PANEL_Y0         46U
#define UI_PANEL_W          156U
#define UI_PANEL_H          238U

/* 自定义深色仪表盘配色 (RGB565) */
#define UI_COL_BG           0x0841U /* 深炭黑背景 */
#define UI_COL_CARD         0x10A2U /* 卡片深灰底 */
#define UI_COL_BORDER       0x31A6U /* 暗灰边框 */
#define UI_COL_GRID         0x2104U /* 频谱网格线 */
#define UI_COL_HDR_NORM     0x022AU /* 正常深蓝顶栏 */
#define UI_COL_HDR_CAL      0xA280U /* 校准暗橙顶栏 */
#define UI_COL_HDR_FAULT    0xB800U /* 故障暗红顶栏 */

static volatile int16_t s_temp_x10    = 0;
static volatile uint8_t s_temp_valid  = 0U;
static volatile int16_t s_temp_th_x10 = 350; /* 默认 35.0℃ 过温阈值 */

/* 大数组放 .bss，避免占用 FreeRTOS 任务栈 */
static float    s_ui_mag[512];
static uint16_t s_prev_h[UI_BAR_COUNT];
static uint16_t s_peak_h[UI_BAR_COUNT];

static const char *ui_fault_name(uint8_t code)
{
    switch (code)
    {
        case BB_FAULT_HIL_800HZ: return "HIL_800HZ";
        case BB_FAULT_OVERLOAD:  return "OVERLOAD";
        case BB_FAULT_LOOSENESS: return "LOOSENESS";
        case BB_FAULT_OVERTEMP:  return "OVERTEMP";
        default:                return "FAULT";
    }
}

static uint16_t ui_bar_color(uint16_t h)
{
    if (h > (uint16_t)((UI_BAR_MAX_H * 75U) / 100U))
    {
        return RED;
    }
    if (h > (uint16_t)((UI_BAR_MAX_H * 45U) / 100U))
    {
        return YELLOW;
    }
    return CYAN;
}

static void ui_draw_static_frame(void)
{
    lcd_clear(UI_COL_BG);

    /* 频谱框与右侧遥测卡片边框 */
    lcd_draw_rectangle(UI_SPEC_X0 - 1U, UI_SPEC_Y0 - 1U,
                       (uint16_t)(UI_SPEC_X0 + UI_SPEC_W), (uint16_t)(UI_SPEC_Y0 + UI_SPEC_H),
                       UI_COL_BORDER);

    lcd_fill_fast(UI_PANEL_X0, UI_PANEL_Y0,
                  (uint16_t)(UI_PANEL_X0 + UI_PANEL_W - 1U),
                  (uint16_t)(UI_PANEL_Y0 + UI_PANEL_H - 1U),
                  UI_COL_CARD);
    lcd_draw_rectangle(UI_PANEL_X0, UI_PANEL_Y0,
                       (uint16_t)(UI_PANEL_X0 + UI_PANEL_W - 1U),
                       (uint16_t)(UI_PANEL_Y0 + UI_PANEL_H - 1U),
                       UI_COL_BORDER);

    /* 右侧卡片固定标签 */
    g_back_color = UI_COL_CARD;
    lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U),  54U, 140U, 16U, 12U, "PEAK FREQ",   GRAY);
    lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U),  94U, 140U, 16U, 12U, "RMS / AMP",   GRAY);
    lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 134U, 140U, 16U, 12U, "Z-SCORE/H8",  GRAY);
    lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 174U, 140U, 16U, 12U, "MOTOR TEMP",  GRAY);
    lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 214U, 140U, 16U, 12U, "KURT / CF",   GRAY);
    lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 252U, 140U, 16U, 12U, "FFT / OVR",   GRAY);

    /* 底部信息条 */
    lcd_fill_fast(0U, 294U, (uint16_t)(lcddev.width - 1U), (uint16_t)(lcddev.height - 1U), 0x1082U);
    lcd_draw_hline(0U, 293U, lcddev.width, UI_COL_BORDER);
}

static void ui_draw_freq_axis(uint8_t phy_mode)
{
    g_back_color = UI_COL_BG;
    lcd_fill_fast(UI_SPEC_X0, (uint16_t)(UI_SPEC_Y0 + UI_SPEC_H + 3U),
                  (uint16_t)(UI_SPEC_X0 + UI_SPEC_W), 290U, UI_COL_BG);

    if (phy_mode != 0U)
    {
        lcd_show_string(UI_SPEC_X0,        274U, 40U, 14U, 12U, "0Hz",   LGRAY);
        lcd_show_string(UI_SPEC_X0 + 68U,  274U, 48U, 14U, 12U, "400",   LGRAY);
        lcd_show_string(UI_SPEC_X0 + 138U, 274U, 48U, 14U, 12U, "800",   LGRAY);
        lcd_show_string(UI_SPEC_X0 + 204U, 274U, 48U, 14U, 12U, "1200",  LGRAY);
        lcd_show_string(UI_SPEC_X0 + 258U, 274U, 48U, 14U, 12U, "1500",  LGRAY);
    }
    else
    {
        lcd_show_string(UI_SPEC_X0,        274U, 40U, 14U, 12U, "0Hz",   LGRAY);
        lcd_show_string(UI_SPEC_X0 + 68U,  274U, 48U, 14U, 12U, "1.2k",  LGRAY);
        lcd_show_string(UI_SPEC_X0 + 138U, 274U, 48U, 14U, 12U, "2.5k",  LGRAY);
        lcd_show_string(UI_SPEC_X0 + 204U, 274U, 48U, 14U, 12U, "3.7k",  LGRAY);
        lcd_show_string(UI_SPEC_X0 + 258U, 274U, 48U, 14U, 12U, "4.7k",  LGRAY);
    }
}

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
        APP_Supervisor_Heartbeat(SUP_HB_SENSOR);
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

static void Task_UI_LCD(void *arg)
{
    uint8_t  last_hdr_state = 0xFFU;
    uint8_t  last_chan_mode = 0xFFU;
    char     buf[48];
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_init();
    lcd_display_dir(1U); /* 480x320 横屏 */
    memset(s_prev_h, 0, sizeof(s_prev_h));
    memset(s_peak_h, 0, sizeof(s_peak_h));

    ui_draw_static_frame();

    for (;;)
    {
        APP_Supervisor_Heartbeat(SUP_HB_UI);
        vTaskDelay(pdMS_TO_TICKS(100));
        DSP_Result_t r;
        int16_t  t_x10 = 0;
        uint8_t  t_ok;
        uint8_t  is_ot;
        uint8_t  hdr_state;
        uint16_t hdr_bg;
        uint16_t b;
        float    max_bin = 1e-3f;

        APP_DSP_GetResult(&r);
        APP_DSP_GetLatestSpectrum(s_ui_mag, 512U);
        t_ok  = APP_SensorUI_GetTempX10(&t_x10);
        is_ot = (uint8_t)((t_ok != 0U) && (t_x10 >= s_temp_th_x10));

        /* 1. 顶部状态栏 (仅在状态切换时整块重绘底色) */
        if (is_ot != 0U)
        {
            hdr_state = 3U;
            hdr_bg    = UI_COL_HDR_FAULT;
        }
        else if (r.fault_flag != 0U)
        {
            hdr_state = 2U;
            hdr_bg    = UI_COL_HDR_FAULT;
        }
        else if ((r.chan_mode != 0U) && (r.base_state == (uint8_t)BASE_STATE_CALIBRATING))
        {
            hdr_state = 1U;
            hdr_bg    = UI_COL_HDR_CAL;
        }
        else
        {
            hdr_state = 0U;
            hdr_bg    = UI_COL_HDR_NORM;
        }

        if (hdr_state != last_hdr_state)
        {
            lcd_fill_fast(0U, 0U, (uint16_t)(lcddev.width - 1U), 33U, hdr_bg);
            lcd_draw_hline(0U, 34U, lcddev.width, UI_COL_BORDER);
            last_hdr_state = hdr_state;
        }

        g_back_color = hdr_bg;
        if (is_ot != 0U)
        {
            snprintf(buf, sizeof(buf), "MOTOR DIAG | %-3s | ALERT: OVERTEMP!   ",
                     r.chan_mode ? "PHY" : "HIL");
        }
        else if (r.fault_flag != 0U)
        {
            snprintf(buf, sizeof(buf), "MOTOR DIAG | %-3s | FAULT: %-10s  ",
                     r.chan_mode ? "PHY" : "HIL", ui_fault_name(r.fault_code));
        }
        else if ((r.chan_mode != 0U) && (r.base_state == (uint8_t)BASE_STATE_CALIBRATING))
        {
            snprintf(buf, sizeof(buf), "MOTOR DIAG | PHY | CAL %2u/%-2u ...      ",
                     (unsigned int)r.cal_progress, (unsigned int)r.cal_target);
        }
        else
        {
            snprintf(buf, sizeof(buf), "MOTOR DIAG | %-3s | NORMAL             ",
                     r.chan_mode ? "PHY" : "HIL");
        }
        lcd_show_string(12U, 9U, 456U, 16U, 16U, buf, WHITE);

        /* 2. 若模式切换则刷新 X 轴刻度标签 */
        if (r.chan_mode != last_chan_mode)
        {
            ui_draw_freq_axis(r.chan_mode);
            last_chan_mode = r.chan_mode;
        }

        /* 3. 将 512 点频谱压缩为 96 根柱 (bin 2..481，每 5 bin 取峰值) 并差分刷新 */
        for (b = 0U; b < UI_BAR_COUNT; b++)
        {
            uint16_t start = (uint16_t)(2U + b * 5U);
            uint16_t k;
            float pk = 0.0f;
            for (k = start; k < (uint16_t)(start + 5U); k++)
            {
                if (s_ui_mag[k] > pk) pk = s_ui_mag[k];
            }
            if (pk > max_bin) max_bin = pk;
        }

        for (b = 0U; b < UI_BAR_COUNT; b++)
        {
            uint16_t start = (uint16_t)(2U + b * 5U);
            uint16_t k;
            float    pk = 0.0f;
            uint16_t h, old_h, old_pk, x;

            for (k = start; k < (uint16_t)(start + 5U); k++)
            {
                if (s_ui_mag[k] > pk) pk = s_ui_mag[k];
            }

            h = (uint16_t)((pk / max_bin) * (float)UI_BAR_MAX_H + 0.5f);
            if (h > UI_BAR_MAX_H) h = UI_BAR_MAX_H;

            x      = (uint16_t)(UI_SPEC_X0 + 2U + b * 3U);
            old_h  = s_prev_h[b];
            old_pk = s_peak_h[b];

            /* 擦除旧峰值保持点 */
            if (old_pk > 1U)
            {
                uint16_t py = (uint16_t)(UI_SPEC_BOTTOM - old_pk);
                lcd_fill_fast(x, py, (uint16_t)(x + 1U), (uint16_t)(py + 1U), UI_COL_BG);
            }

            /* 差分更新柱体：仅重绘变化区间或颜色跳变区 */
            if (h < old_h)
            {
                lcd_fill_fast(x, (uint16_t)(UI_SPEC_BOTTOM - old_h),
                              (uint16_t)(x + 1U), (uint16_t)(UI_SPEC_BOTTOM - h),
                              UI_COL_BG);
            }
            if (h > 0U)
            {
                uint16_t col = ui_bar_color(h);
                uint16_t y_top = (uint16_t)(UI_SPEC_BOTTOM - h);
                if (h > old_h)
                {
                    /* 若颜色区间变化则整柱重涂，否则只画新增顶部 */
                    if (col != ui_bar_color(old_h))
                    {
                        lcd_fill_fast(x, y_top, (uint16_t)(x + 1U), UI_SPEC_BOTTOM, col);
                    }
                    else
                    {
                        lcd_fill_fast(x, y_top, (uint16_t)(x + 1U),
                                      (uint16_t)(UI_SPEC_BOTTOM - old_h), col);
                    }
                }
                else if (col != ui_bar_color(old_h))
                {
                    lcd_fill_fast(x, y_top, (uint16_t)(x + 1U), UI_SPEC_BOTTOM, col);
                }
            }
            s_prev_h[b] = h;

            /* 峰值保持小红帽缓慢回落 */
            if (h >= old_pk)
            {
                s_peak_h[b] = h;
            }
            else if (old_pk > 2U)
            {
                s_peak_h[b] = (uint16_t)(old_pk - 2U);
            }
            else
            {
                s_peak_h[b] = 0U;
            }

            if (s_peak_h[b] > 1U)
            {
                uint16_t py = (uint16_t)(UI_SPEC_BOTTOM - s_peak_h[b]);
                lcd_fill_fast(x, py, (uint16_t)(x + 1U), (uint16_t)(py + 1U), WHITE);
            }
        }

        /* 4. 右侧遥测卡片数值刷新 (固定宽度直接覆盖，不闪烁) */
        g_back_color = UI_COL_CARD;

        snprintf(buf, sizeof(buf), "%6.1f Hz  ", r.peak_freq_hz);
        lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 70U, 142U, 16U, 16U, buf, CYAN);

        if (r.chan_mode != 0U)
        {
            snprintf(buf, sizeof(buf), "%6.1f mg  ", r.peak_amp_v);
        }
        else
        {
            snprintf(buf, sizeof(buf), "%6.3f V   ", r.peak_amp_v);
        }
        lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 110U, 142U, 16U, 16U, buf, WHITE);

        if (r.chan_mode != 0U)
        {
            snprintf(buf, sizeof(buf), "%5.2f (f%u) ", r.harm_800_amp_v, (unsigned int)r.dominant_feat);
        }
        else
        {
            snprintf(buf, sizeof(buf), "%5.3f V   ", r.harm_800_amp_v);
        }
        lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 150U, 142U, 16U, 16U, buf,
                        r.fault_flag ? RED : GREEN);

        if (t_ok != 0U)
        {
            int16_t abs_t = (t_x10 >= 0) ? t_x10 : (int16_t)(-t_x10);
            snprintf(buf, sizeof(buf), "%2d.%1d C/%2dC ",
                     (int)(t_x10 / 10), (int)(abs_t % 10), (int)(s_temp_th_x10 / 10));
        }
        else
        {
            snprintf(buf, sizeof(buf), " --.- C    ");
        }
        lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 190U, 142U, 16U, 16U, buf,
                        is_ot ? RED : GREEN);

        snprintf(buf, sizeof(buf), "%4.1f / %4.1f", r.kurtosis, r.crest_factor);
        lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 230U, 142U, 16U, 16U, buf, LGRAY);

        snprintf(buf, sizeof(buf), "%4luus O:%lu",
                 (unsigned long)r.fft_cost_us, (unsigned long)r.phy_ovr_cnt);
        lcd_show_string((uint16_t)(UI_PANEL_X0 + 8U), 266U, 142U, 16U, 12U, buf,
                        (r.phy_ovr_cnt > 0U) ? RED : YELLOW);

        /* 5. 底部信息栏 */
        g_back_color = 0x1082U;
        snprintf(buf, sizeof(buf), "LCD:0x%04X  Frames:%-6lu  CLI:USART1 115200",
                 (unsigned int)lcddev.id, (unsigned long)r.phy_frame_cnt);
        lcd_show_string(12U, 300U, 456U, 14U, 12U, buf, LGRAY);

        vTaskDelay(pdMS_TO_TICKS(100)); /* 10 FPS */
    }
}

void APP_SensorUI_Init(void)
{
    xTaskCreate(Task_Sensor, "Task_Sensor", 384, NULL, tskIDLE_PRIORITY + 22, NULL);
    xTaskCreate(Task_UI_LCD, "Task_UI_LCD", 512, NULL, tskIDLE_PRIORITY + 10, NULL);
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
