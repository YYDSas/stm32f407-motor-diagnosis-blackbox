#ifndef __BSP_STEPPER_H
#define __BSP_STEPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  设置 28BYJ-48 步进电机档位（TIM7 硬件精确节拍，8 拍半步驱动）
 * @param  mode: 0=停止(四相断电防发热), 1=低速(300Hz), 2=中速(400Hz,推荐), 3=高速(500Hz)
 */
void BSP_Stepper_SetMode(uint8_t mode);

/**
 * @brief  获取当前步进激励拍频 (Hz): 0 / 300 / 400 / 500
 */
uint16_t BSP_Stepper_GetStepHz(void);

/**
 * @brief  获取当前档位 (0..3)
 */
uint8_t BSP_Stepper_GetMode(void);

/**
 * @brief  在 TIM7 更新中断回调 (HAL_TIM_PeriodElapsedCallback) 中调用
 *         注意：TIM7 优先级为 3 (高于 FreeRTOS 屏蔽阈值)，本函数绝不调用 RTOS API
 */
void BSP_Stepper_OnTimPeriodElapsed(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STEPPER_H */
