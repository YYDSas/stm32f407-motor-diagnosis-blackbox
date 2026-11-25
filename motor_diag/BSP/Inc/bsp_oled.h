#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH   (128U)
#define OLED_HEIGHT  (64U)
#define OLED_PAGES   (8U)

/**
 * @brief  初始化 SSD1306 128x64 SPI2 OLED（调用前需持有 SPI2 互斥量）
 */
void BSP_OLED_Init(void);

/**
 * @brief  清空内部 1 KB 显存（纯内存操作，不占 SPI2）
 */
void BSP_OLED_Clear(void);

/**
 * @brief  在指定页 (0..7) 与 X 坐标 (0..122) 绘制 ASCII 字符串（5x7 紧凑字体）
 */
void BSP_OLED_DrawStr(uint8_t page, uint8_t x, const char *str);

/**
 * @brief  在下半区 (y=16..63, 高度 48px) 绘制 64 柱实时频谱直方图（2px/柱）
 * @param  bars_h: 长度 64 的柱高数组，每项范围 0..48
 */
void BSP_OLED_DrawSpectrum64(const uint8_t bars_h[64]);

/**
 * @brief  将单页 (0..7, 128 字节) 显存刷写到 OLED（调用前需持有 SPI2 互斥量）
 *         分页短锁刷新可确保不阻塞高优先级 ADXL345 FIFO 读取
 */
void BSP_OLED_FlushPage(uint8_t page);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OLED_H */
