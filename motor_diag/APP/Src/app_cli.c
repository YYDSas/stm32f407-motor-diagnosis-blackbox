#include "app_cli.h"
#include "bsp_dwt.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "bsp_bkpsram.h"
#include "app_dsp.h"
#include "app_blackbox.h"
#include "bsp_stepper.h"
#include "bsp_adxl345.h"
#include "app_sensor_ui.h"
#include <stdlib.h>
#define CLI_RX_BUF_SIZE   128
#define LOG_TX_BUF_SIZE   256

static SemaphoreHandle_t xUartMutex    = NULL;
static TaskHandle_t      xCliTaskHandle = NULL;

static uint8_t  s_dma_rx_buf[CLI_RX_BUF_SIZE];
static char     s_cmd_buf[CLI_RX_BUF_SIZE];
static volatile uint16_t s_cmd_len = 0;
static char     s_task_list_buf[512];

/* 线程安全的串口日志打印 */
void Log_Printf(const char *fmt, ...)
{
    char tx_buf[LOG_TX_BUF_SIZE];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len >= (int)sizeof(tx_buf)) len = sizeof(tx_buf) - 1;

    if (xUartMutex != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        if (xSemaphoreTake(xUartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 100);
            xSemaphoreGive(xUartMutex);
        }
    }
    else
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 100);
    }
}

/* 启动一次 DMA + 空闲中断接收，并关闭半满中断防止误触发 */
static void CLI_StartDmaRx(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_dma_rx_buf, CLI_RX_BUF_SIZE - 1);
    if (huart1.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

/* HAL 空闲中断回调：零阻塞通知 Task_CLI */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        BaseType_t xHigherPrioWoken = pdFALSE;
        uint16_t copy_len = (Size < CLI_RX_BUF_SIZE - 1) ? Size : (CLI_RX_BUF_SIZE - 1);

        memcpy(s_cmd_buf, s_dma_rx_buf, copy_len);
        s_cmd_buf[copy_len] = '\0';
        s_cmd_len = copy_len;

        CLI_StartDmaRx();

        if (xCliTaskHandle != NULL)
        {
            vTaskNotifyGiveFromISR(xCliTaskHandle, &xHigherPrioWoken);
            portYIELD_FROM_ISR(xHigherPrioWoken);
        }
    }
}

/* 处理串口命令 */
static void CLI_HandleCommand(char *cmd)
{
    /* 去掉末尾的 \r \n 空格 */
    size_t n = strlen(cmd);
    while (n > 0 && (cmd[n - 1] == '\r' || cmd[n - 1] == '\n' || cmd[n - 1] == ' '))
    {
        cmd[--n] = '\0';
    }
    if (n == 0) return;

    if (strcmp(cmd, "help") == 0)
    {
        Log_Printf("\r\n[CLI] Commands: help | stat | dwt\r\n");
    }
    else if (strcmp(cmd, "stat") == 0)
    {
        vTaskList(s_task_list_buf);
        Log_Printf("\r\nTask            State   Prio    Stack   Num\r\n");
        Log_Printf("--------------------------------------------\r\n");
        Log_Printf("%s", s_task_list_buf);
        Log_Printf("Free Heap: %u B | Min Ever Free: %u B\r\n",
                   (unsigned int)xPortGetFreeHeapSize(),
                   (unsigned int)xPortGetMinimumEverFreeHeapSize());
    }
    else if (strcmp(cmd, "dwt") == 0)
    {
        uint32_t t0 = BSP_DWT_GetUs();
        BSP_DWT_DelayUs(1000);
        uint32_t dt = BSP_DWT_GetUs() - t0;
        Log_Printf("[DWT] Delay 1000us measured = %lu us\r\n", (unsigned long)dt);
    }
    else if (strcmp(cmd, "temp") == 0)
    {
        int16_t t = 0;
        uint8_t ok = APP_SensorUI_GetTempX10(&t);
        int16_t th = APP_SensorUI_GetTempThX10();
        Log_Printf("[TEMP] valid=%u cur=%.1f C | overtemp_th=%.1f C\r\n",
                   (unsigned int)ok, (float)t / 10.0f, (float)th / 10.0f);
    }
    else if (strncmp(cmd, "temp th ", 8) == 0)
    {
        int16_t th_c = (int16_t)atoi(&cmd[8]);
        APP_SensorUI_SetTempThX10((int16_t)(th_c * 10));
        Log_Printf("[TEMP] Overtemp threshold set to %d.0 C\r\n", (int)th_c);
    }
   else if (strcmp(cmd, "fft") == 0)
    {
        DSP_Result_t r;
        APP_DSP_GetResult(&r);
        if (r.chan_mode == 0U)
        {
            Log_Printf("[FFT HIL] Sig=%s | Peak=%.1f Hz (%.2f V) | 800Hz=%.2f V | Fault=%u | Cost=%lu us\r\n",
                       r.sig_mode ? "FAULT" : "NORM",
                       r.peak_freq_hz, r.peak_amp_v, r.harm_800_amp_v,
                       (unsigned int)r.fault_flag, (unsigned long)r.fft_cost_us);
        }
        else
        {
            const char *st = (r.base_state == 1U) ? "CAL" : ((r.base_state == 3U) ? "FAULT" : "NORM");
            Log_Printf("[FFT PHY] %s(%u/%u) | Peak=%.1fHz RMS=%.1fmg Kurt=%.2f | Z=%.2f(f%u) | OVR=%lu Frm=%lu Cost=%luus\r\n",
                       st, (unsigned int)r.cal_progress, (unsigned int)r.cal_target,
                       r.peak_freq_hz, r.peak_amp_v, r.kurtosis,
                       r.harm_800_amp_v, (unsigned int)r.dominant_feat,
                       (unsigned long)r.phy_ovr_cnt, (unsigned long)r.phy_frame_cnt,
                       (unsigned long)r.fft_cost_us);
        }
    }
    else if (strcmp(cmd, "mode hil") == 0)
    {
        APP_DSP_SetChannelMode(0U);
        Log_Printf("[MODE] Switched to HIL (DAC->ADC 10.24kHz)\r\n");
    }
    else if (strcmp(cmd, "mode phy") == 0)
    {
        uint8_t ok = APP_DSP_SetChannelMode(1U);
        Log_Printf("[MODE] Switched to PHY (ADXL345 3200Hz 50%%-overlap): %s (calibrating 30 frames...)\r\n",
                   ok ? "OK" : "FAIL");
    }
    else if (strcmp(cmd, "cal") == 0)
    {
        APP_DSP_TriggerCalibration(30U);
        Log_Printf("[BASE] Re-calibrating Welford baseline (30 frames ~4.8s)...\r\n");
    }
    else if (strcmp(cmd, "sig norm") == 0)
    {
        APP_DSP_SetSignalMode(0);
        Log_Printf("[SIG] Switched to 200Hz NORMAL wave\r\n");
    }
    else if (strcmp(cmd, "sig fault") == 0)
    {
        APP_DSP_SetSignalMode(1);
        Log_Printf("[SIG] Switched to 200Hz + 800Hz FAULT wave\r\n");
    }
    else if (strcmp(cmd, "bkp") == 0)
{
    const BkpsramArea_t *b = BSP_BkpSram_GetArea();
    Log_Printf("[BKP] magic=0x%08lX state=0x%02X seq=%lu frames=%u raw=%u valid=%u\r\n",
               (unsigned long)b->magic, (unsigned int)b->state,
               (unsigned long)b->seq, (unsigned int)b->frame_count,
               (unsigned int)b->raw_count, (unsigned int)BSP_BkpSram_IsFrozenValid());
}
else if (strcmp(cmd, "fault bkp") == 0)
{
    BSP_BkpSram_FillTestPatternAndFreeze();
    Log_Printf("[BKP] Test pattern written & FROZEN. Unplug power for 10 min to verify.\r\n");
}
else if (strncmp(cmd, "dump", 4) == 0)
    {
        uint32_t n = (cmd[4] == ' ') ? (uint32_t)atoi(&cmd[5]) : 20U;
        APP_Blackbox_PrintDump(n);
    }
    else if (strncmp(cmd, "snap ", 5) == 0)
    {
        APP_Blackbox_PrintSnapshot((uint16_t)atoi(&cmd[5]));
    }
    else if (strcmp(cmd, "clear") == 0)
    {
        APP_Blackbox_ClearAll();
    }
    else if (strcmp(cmd, "stress flash") == 0)
    {
        APP_Blackbox_SetStressMode(1U);
    }
    else if (strcmp(cmd, "stress stop") == 0)
    {
        APP_Blackbox_SetStressMode(0U);
    }
    else if (strncmp(cmd, "motor ", 6) == 0)
    {
        uint8_t m = (uint8_t)atoi(&cmd[6]);
        BSP_Stepper_SetMode(m);
        Log_Printf("[MOTOR] mode=%u step_hz=%u Hz\r\n",
                   (unsigned int)BSP_Stepper_GetMode(),
                   (unsigned int)BSP_Stepper_GetStepHz());
    }
    else if (strcmp(cmd, "adxl init") == 0 || strcmp(cmd, "adxl int2") == 0)
    {
        uint8_t use_int2 = (strcmp(cmd, "adxl int2") == 0) ? 1U : 0U;
        uint8_t id = BSP_ADXL345_ReadDevId();
        uint8_t ok = BSP_ADXL345_Init(use_int2);
        vTaskDelay(pdMS_TO_TICKS(20)); /* 静止等 20 ms，让 FIFO 攒满 16 点触发水位中断 */
Log_Printf("[ADXL] DEVID=0x%02X Init(%s)=%s EXTI_after_20ms=%lu\r\n",
           (unsigned int)id, use_int2 ? "INT2" : "INT1",
           ok ? "OK" : "FAIL", (unsigned long)BSP_ADXL345_GetExtiCount());
        Log_Printf("[ADXL] DEVID=0x%02X Init(%s)=%s\r\n",
                   (unsigned int)id, use_int2 ? "INT2" : "INT1", ok ? "OK" : "FAIL");
    }
    else if (strcmp(cmd, "adxl snr") == 0)
    {
        ADXL345_SnrStats_t st;
        if (BSP_ADXL345_RunSnrTest(1024U, &st) != 0U)
        {
            Log_Printf("[ADXL SNR] N=%lu (%lums) EXTI=%lu OVR=%lu | RMS(mg) X=%.1f Y=%.1f Z=%.1f | Z_p2p=%.1f Kurt=%.2f\r\n",
                       (unsigned long)st.samples_read, (unsigned long)st.cost_ms,
                       (unsigned long)st.exti_cnt, (unsigned long)st.overrun_cnt,
                       st.rms_x_mg, st.rms_y_mg, st.rms_z_mg, st.p2p_z_mg, st.kurt_z);
        }
        else
        {
            Log_Printf("[ADXL SNR] Timeout! Run 'adxl init' first.\r\n");
        }
    }
    else
    {
        Log_Printf("[CLI] Unknown cmd: %s (type 'help')\r\n", cmd);
    }
}

static void Task_CLI(void *argument)
{
    (void)argument;
    CLI_StartDmaRx();
    Log_Printf("\r\n=== STM32F407 Motor Diag CLI Ready (type 'help') ===\r\n");

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        CLI_HandleCommand(s_cmd_buf);
    }
}

void APP_CLI_Init(void)
{
    xUartMutex = xSemaphoreCreateMutex();
    xTaskCreate(Task_CLI, "Task_CLI", 512, NULL, tskIDLE_PRIORITY + 2, &xCliTaskHandle);
}
