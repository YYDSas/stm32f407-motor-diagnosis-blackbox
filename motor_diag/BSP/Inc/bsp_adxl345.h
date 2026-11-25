#ifndef __BSP_ADXL345_H
#define __BSP_ADXL345_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL345_DEVID_EXPECTED   (0xE5U)
#define ADXL345_MG_PER_LSB       (3.9f)   /* Full-Res +/-2g: 3.9 mg/LSB */
#define ADXL345_FIFO_WATERMARK   (16U)    /* 3200 Hz 下每 5 ms 触发一次中断 */

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ADXL345_Sample_t;

typedef struct {
    float    rms_x_mg;
    float    rms_y_mg;
    float    rms_z_mg;
    float    p2p_z_mg;
    float    kurt_z;
    uint32_t samples_read;
    uint32_t exti_cnt;
    uint32_t overrun_cnt;
    uint32_t cost_ms;
} ADXL345_SnrStats_t;

/**
 * @brief  读取 ADXL345 DEVID (0x00)，正常应返回 0xE5
 */
uint8_t BSP_ADXL345_ReadDevId(void);

/**
 * @brief  初始化 ADXL345 为 3200 Hz ODR + +/-2g Full-Res + FIFO Stream 水位 16
 * @param  use_int2: 0=水位中断映射至 INT1(默认); 1=映射至 INT2(百问网模块若走 INT2 时切换)
 * @retval 1: 初始化成功(DEVID==0xE5); 0: 失败
 */
uint8_t BSP_ADXL345_Init(uint8_t use_int2);

/**
 * @brief  批量读取 FIFO 中已就绪的加速度样本（调用前需持有 SPI2 互斥量或无并发）
 * @param  out          输出样本数组
 * @param  max_cnt      数组最大容量（推荐 32）
 * @param  out_overrun  可选：若发生 FIFO 溢出则置 1
 * @retval 实际读取到的样本数量 (0..max_cnt)
 */
uint8_t BSP_ADXL345_ReadFifoBatch(ADXL345_Sample_t *out, uint8_t max_cnt, uint8_t *out_overrun);

/**
 * @brief  在 PB0 EXTI0 中断回调 (HAL_GPIO_EXTI_Callback) 中调用，用于统计中断触发次数
 */
void BSP_ADXL345_OnExtiIrq(void);

/**
 * @brief  获取累计 EXTI 中断次数
 */
uint32_t BSP_ADXL345_GetExtiCount(void);

/**
 * @brief  W4 Day 1 实测工具：连续采集 sample_cnt 点（默认 1024 点 ≈ 320 ms），
 *         计算 X/Y/Z 三轴 RMS、Z 轴峰峰值/峭度及 EXTI0/溢出统计
 */
uint8_t BSP_ADXL345_RunSnrTest(uint32_t sample_cnt, ADXL345_SnrStats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADXL345_H */
