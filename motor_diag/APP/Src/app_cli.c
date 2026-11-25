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
    else if (strcmp(cmd, "fft") == 0)
    {
        DSP_Result_t r;
        APP_DSP_GetResult(&r);
        Log_Printf("[FFT] Mode=%s | Peak=%.1f Hz (%.2f V) | 800Hz=%.2f V | Fault=%u | Cost=%lu us\r\n",
                   r.sig_mode ? "FAULT" : "NORM",
                   r.peak_freq_hz, r.peak_amp_v, r.harm_800_amp_v,
                   (unsigned int)r.fault_flag, (unsigned long)r.fft_cost_us);
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
