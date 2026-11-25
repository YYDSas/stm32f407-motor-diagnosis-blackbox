#ifndef __BSP_DS18B20_H
#define __BSP_DS18B20_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  检测 PG9 单总线上是否存在 DS18B20
 * @retval 1: 检测到应答脉冲; 0: 未检测到器件
 */
uint8_t BSP_DS18B20_Detect(void);

/**
 * @brief  发送温度转换指令 (0xCC + 0x44)，非阻塞（调用后需延时 >=750ms 再读取）
 * @retval 1: 成功; 0: 总线无应答
 */
uint8_t BSP_DS18B20_StartConvert(void);

/**
 * @brief  读取 9 字节暂存器并校验 CRC8，输出 0.1℃ 单位温度（如 265 = 26.5℃）
 * @retval 1: 读取且 CRC8 校验成功; 0: 无应答或 CRC 错误
 */
uint8_t BSP_DS18B20_ReadTempX10(int16_t *out_temp_x10);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DS18B20_H */
