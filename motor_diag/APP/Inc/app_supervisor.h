/**
 ****************************************************************************************************
 * @file        app_supervisor.h
 * @brief       系统健康守护模块 (W5): IWDG 2s 独立看门狗 + FreeRTOS EventGroup 多任务心跳聚合
 *              + RCC->CSR 复位原因溯源落盘 + CLI 死锁注入自愈测试
 ****************************************************************************************************
 */

#ifndef __APP_SUPERVISOR_H
#define __APP_SUPERVISOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 核心任务心跳位定义 (供 FreeRTOS EventGroup 聚合) */
#define SUP_HB_DSP          (1UL << 0)  /* Task_DSP_FFT   */
#define SUP_HB_SENSOR       (1UL << 1)  /* Task_Sensor    */
#define SUP_HB_BB           (1UL << 2)  /* Task_Blackbox  */
#define SUP_HB_UI           (1UL << 3)  /* Task_UI_LCD    */
#define SUP_HB_ALL          (SUP_HB_DSP | SUP_HB_SENSOR | SUP_HB_BB | SUP_HB_UI)

/* 复位原因编码 (存入黑匣子 reset_reason 字段) */
typedef enum {
    SUP_RST_UNKNOWN = 0U,
    SUP_RST_POR     = 1U,   /* 上电/掉电复位 */
    SUP_RST_PIN     = 2U,   /* 外部 NRST 按键复位 */
    SUP_RST_SOFT    = 3U,   /* 软件复位 (NVIC_SystemReset) */
    SUP_RST_IWDG    = 4U,   /* 独立看门狗超时自愈复位 */
    SUP_RST_WWDG    = 5U,   /* 窗口看门狗复位 */
    SUP_RST_BOR     = 6U    /* 欠压复位 */
} SupResetReason_t;

/**
 * @brief  创建 Task_Supervisor (优先级 15)，捕获并清除 RCC->CSR 复位标志，初始化状态 LED
 */
void APP_Supervisor_Init(void);

/**
 * @brief  各核心任务周期调用，向 EventGroup 上报存活心跳
 * @param  hb_bit  SUP_HB_DSP / SUP_HB_SENSOR / SUP_HB_BB / SUP_HB_UI
 */
void APP_Supervisor_Heartbeat(uint32_t hb_bit);

/**
 * @brief  CLI 'fault fft' 调用，向 Task_DSP_FFT 注入死锁以验证 IWDG 2s 硬件复位自愈
 */
void APP_Supervisor_InjectFftDeadlock(void);

/**
 * @brief  Task_DSP_FFT 主循环查询是否被注入了死锁测试
 */
uint8_t APP_Supervisor_IsFftDeadlockInjected(void);

/**
 * @brief  获取本次启动的复位原因编码及字符串
 */
SupResetReason_t APP_Supervisor_GetResetReason(void);
const char      *APP_Supervisor_GetResetReasonStr(void);

/**
 * @brief  CLI 'resetinfo' 命令打印复位溯源、IWDG 喂狗次数及各任务心跳状态
 */
void APP_Supervisor_PrintInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SUPERVISOR_H */
