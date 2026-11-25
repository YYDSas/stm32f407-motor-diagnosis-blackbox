#ifndef __APP_SENSOR_UI_H
#define __APP_SENSOR_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  创建 Task_Sensor (DS18B20 过温监测, 优先级 22) 与 Task_UI_OLED (实时频谱显示, 优先级 10)
 */
void APP_SensorUI_Init(void);

/**
 * @brief  获取最新温度 (0.1℃ 单位) 及有效标志
 */
uint8_t APP_SensorUI_GetTempX10(int16_t *out_temp_x10);

/**
 * @brief  设置过温告警阈值 (0.1℃ 单位，默认 350 = 35.0℃，方便手指捏热演示)
 */
void APP_SensorUI_SetTempThX10(int16_t th_x10);

/**
 * @brief  获取当前过温告警阈值 (0.1℃ 单位)
 */
int16_t APP_SensorUI_GetTempThX10(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SENSOR_UI_H */
