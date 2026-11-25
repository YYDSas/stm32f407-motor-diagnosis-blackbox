/**
 ****************************************************************************************************
 * @file        app_supervisor.c
 * @brief       Task_Supervisor (优先级 15): EventGroup 心跳聚合喂狗 + RCC->CSR 溯源落盘 + LED 状态指示
 ****************************************************************************************************
 */

#include "app_supervisor.h"
#include "app_dsp.h"
#include "app_blackbox.h"
#include "app_cli.h"
#include "bsp_bkpsram.h"
#include "iwdg.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include <string.h>

/* 守护参数 */
#define SUP_POLL_PERIOD_MS      (250U)  /* 250 ms 检查一次心跳与喂狗 */
#define SUP_HB_TIMEOUT_MS       (1600U) /* 单任务允许最大失联时间 (容忍 DS18B20 800ms 周期) */
#define SUP_BOOT_GRACE_MS       (2500U) /* 上电初始化宽限期，避免 LCD/Flash 初始化期间误饿狗 */
#define SUP_TASK_COUNT          (4U)

/* 探索者 F407 板载 LED (低电平点亮): PF9=LED0(红,告警), PF10=LED1(绿,心跳) */
#define SUP_LED_FAULT_PORT      GPIOF
#define SUP_LED_FAULT_PIN       GPIO_PIN_9
#define SUP_LED_HB_PORT         GPIOF
#define SUP_LED_HB_PIN          GPIO_PIN_10

static EventGroupHandle_t s_hb_eg           = NULL;
static volatile uint8_t   s_fft_deadlock    = 0U;
static SupResetReason_t   s_reset_reason    = SUP_RST_UNKNOWN;
static uint32_t           s_raw_csr         = 0U;
static uint32_t           s_feed_cnt        = 0U;
static uint32_t           s_last_seen[SUP_TASK_COUNT];
static uint8_t            s_starved_logged  = 0U;

static const char * const s_task_names[SUP_TASK_COUNT] = {
    "DSP", "SENSOR", "BLACKBOX", "UI_LCD"
};

static void sup_led_init(void)
{
    GPIO_InitTypeDef gpio;
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio.Pin   = SUP_LED_FAULT_PIN | SUP_LED_HB_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &gpio);

    /* 默认熄灭 (高电平灭) */
    HAL_GPIO_WritePin(SUP_LED_FAULT_PORT, SUP_LED_FAULT_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SUP_LED_HB_PORT,    SUP_LED_HB_PIN,    GPIO_PIN_SET);
}

static void sup_capture_reset_reason(void)
{
    uint32_t csr = RCC->CSR;
    s_raw_csr = csr;

    if ((csr & RCC_CSR_IWDGRSTF) != 0U)
    {
        s_reset_reason = SUP_RST_IWDG;
    }
    else if ((csr & RCC_CSR_WWDGRSTF) != 0U)
    {
        s_reset_reason = SUP_RST_WWDG;
    }
    else if ((csr & RCC_CSR_SFTRSTF) != 0U)
    {
        s_reset_reason = SUP_RST_SOFT;
    }
    else if ((csr & RCC_CSR_PORRSTF) != 0U)
    {
        s_reset_reason = SUP_RST_POR;
    }
    else if ((csr & RCC_CSR_BORRSTF) != 0U)
    {
        s_reset_reason = SUP_RST_BOR;
    }
    else if ((csr & RCC_CSR_PINRSTF) != 0U)
    {
        s_reset_reason = SUP_RST_PIN;
    }
    else
    {
        s_reset_reason = SUP_RST_UNKNOWN;
    }

    /* 必须清除复位标志，否则后续复位会残留旧标志 */
    RCC->CSR |= RCC_CSR_RMVF;
}

const char *APP_Supervisor_GetResetReasonStr(void)
{
    switch (s_reset_reason)
    {
        case SUP_RST_POR:  return "POR_POWERON";
        case SUP_RST_PIN:  return "NRST_PIN";
        case SUP_RST_SOFT: return "SOFT_RESET";
        case SUP_RST_IWDG: return "IWDG_WATCHDOG";
        case SUP_RST_WWDG: return "WWDG_WATCHDOG";
        case SUP_RST_BOR:  return "BOR_BROWNOUT";
        default:           return "UNKNOWN";
    }
}

SupResetReason_t APP_Supervisor_GetResetReason(void)
{
    return s_reset_reason;
}

static void Task_Supervisor(void *arg)
{
    uint32_t boot_tick = HAL_GetTick();
    uint32_t i;
    uint8_t  iwdg_logged = 0U;
    uint8_t  led_div = 0U;
    (void)arg;

    for (i = 0U; i < SUP_TASK_COUNT; i++)
    {
        s_last_seen[i] = boot_tick;
    }

    /* 先喂一次狗，确保启动阶段裕量充足 */
    HAL_IWDG_Refresh(&hiwdg);
    vTaskDelay(pdMS_TO_TICKS(600));
    HAL_IWDG_Refresh(&hiwdg);

    Log_Printf("[SUP] Boot ResetReason=%s (RCC_CSR=0x%08lX) | IWDG=2.0s active\r\n",
               APP_Supervisor_GetResetReasonStr(), (unsigned long)s_raw_csr);

    for (;;)
    {
        uint32_t now  = HAL_GetTick();
        EventBits_t got = xEventGroupClearBits(s_hb_eg, SUP_HB_ALL);
        uint32_t missing_mask = 0U;

        /* 若本次为 IWDG 看门狗自愈复位，待 Flash 挂载完成后自动落盘一条溯源记录 */
        if ((s_reset_reason == SUP_RST_IWDG) && (iwdg_logged == 0U) && ((now - boot_tick) >= 1200U))
        {
            BlackboxRecord_t rec;
            memset(&rec, 0, sizeof(rec));
            rec.timestamp_ms = now;
            rec.fault_type   = (uint8_t)BB_FAULT_IWDG_RESET;
            rec.reset_reason = (uint8_t)s_reset_reason;
            APP_Blackbox_PushFault(&rec, NULL, 0U, 0.0f);
            Log_Printf("[SUP] *** RECOVERED FROM IWDG WATCHDOG RESET! Logged to Blackbox ***\r\n");
            iwdg_logged = 1U;
        }

        /* 更新已打卡任务的最近活跃时间戳 */
        for (i = 0U; i < SUP_TASK_COUNT; i++)
        {
            if ((got & (1UL << i)) != 0U)
            {
                s_last_seen[i] = now;
            }
            if ((now - s_last_seen[i]) > SUP_HB_TIMEOUT_MS)
            {
                missing_mask |= (1UL << i);
            }
        }

        /* 上电宽限期内或全任务健康时正常喂狗 */
        if (((now - boot_tick) < SUP_BOOT_GRACE_MS) || (missing_mask == 0U))
        {
            HAL_IWDG_Refresh(&hiwdg);
            s_feed_cnt++;
            s_starved_logged = 0U;

            /* 绿灯 LED1 以 1 Hz 节拍闪烁 (每 500 ms 翻转) */
            led_div++;
            if ((led_div & 1U) == 0U)
            {
                HAL_GPIO_TogglePin(SUP_LED_HB_PORT, SUP_LED_HB_PIN);
            }
        }
        else
        {
            /* 有核心任务失联：停止喂狗，红灯常亮，等待硬件 IWDG 强制复位自愈 */
            HAL_GPIO_WritePin(SUP_LED_FAULT_PORT, SUP_LED_FAULT_PIN, GPIO_PIN_RESET);
            if (s_starved_logged == 0U)
            {
                Log_Printf("[SUP] WATCHDOG STARVATION! missing=0x%02lX -> halting feed for IWDG reset\r\n",
                           (unsigned long)missing_mask);
                s_starved_logged = 1U;
            }
        }

        /* 故障红灯 LED0 同步反映当前实时诊断状态 */
        if (missing_mask == 0U)
        {
            DSP_Result_t dr;
            APP_DSP_GetResult(&dr);
            HAL_GPIO_WritePin(SUP_LED_FAULT_PORT, SUP_LED_FAULT_PIN,
                              (dr.fault_flag != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }

        vTaskDelay(pdMS_TO_TICKS(SUP_POLL_PERIOD_MS));
    }
}

void APP_Supervisor_Init(void)
{
    sup_capture_reset_reason();
    sup_led_init();
    s_hb_eg = xEventGroupCreate();
    xTaskCreate(Task_Supervisor, "Task_Super", 384, NULL, tskIDLE_PRIORITY + 15, NULL);
}

void APP_Supervisor_Heartbeat(uint32_t hb_bit)
{
    if (s_hb_eg != NULL)
    {
        (void)xEventGroupSetBits(s_hb_eg, hb_bit);
    }
}

void APP_Supervisor_InjectFftDeadlock(void)
{
    Log_Printf("[SUP] Injecting deadlock into Task_DSP_FFT! Expect IWDG reset in ~2.0s...\r\n");
    s_fft_deadlock = 1U;
}

uint8_t APP_Supervisor_IsFftDeadlockInjected(void)
{
    return s_fft_deadlock;
}

void APP_Supervisor_PrintInfo(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t i;

    Log_Printf("[SUP] ResetReason=%s (RCC_CSR=0x%08lX) | IWDG_Feeds=%lu | Uptime=%lums\r\n",
               APP_Supervisor_GetResetReasonStr(),
               (unsigned long)s_raw_csr,
               (unsigned long)s_feed_cnt,
               (unsigned long)now);

    for (i = 0U; i < SUP_TASK_COUNT; i++)
    {
        Log_Printf("  HB[%lu] %-8s : last_seen=%lu ms ago\r\n",
                   (unsigned long)i,
                   s_task_names[i],
                   (unsigned long)(now - s_last_seen[i]));
    }
}
