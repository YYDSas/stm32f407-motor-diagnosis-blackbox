/**
 ****************************************************************************************************
 * @file        lcd.h
 * @brief       正点原子探索者 STM32F407 V3.5 TFTLCD (3.5寸等 MCU FSMC 屏) 驱动头文件
 *              基于 ALIENTEK F407 官方驱动 V1.1 移植至 CubeMX + FreeRTOS 工程
 ****************************************************************************************************
 */

#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************************/
/* 探索者 F407 34-pin TFTLCD FSMC 控制引脚定义 */
#define LCD_WR_GPIO_PORT                GPIOD
#define LCD_WR_GPIO_PIN                 GPIO_PIN_5
#define LCD_WR_GPIO_CLK_ENABLE()        do { __HAL_RCC_GPIOD_CLK_ENABLE(); } while (0)

#define LCD_RD_GPIO_PORT                GPIOD
#define LCD_RD_GPIO_PIN                 GPIO_PIN_4
#define LCD_RD_GPIO_CLK_ENABLE()        do { __HAL_RCC_GPIOD_CLK_ENABLE(); } while (0)

#define LCD_BL_GPIO_PORT                GPIOB
#define LCD_BL_GPIO_PIN                 GPIO_PIN_15
#define LCD_BL_GPIO_CLK_ENABLE()        do { __HAL_RCC_GPIOB_CLK_ENABLE(); } while (0)

#define LCD_CS_GPIO_PORT                GPIOG
#define LCD_CS_GPIO_PIN                 GPIO_PIN_12
#define LCD_CS_GPIO_CLK_ENABLE()        do { __HAL_RCC_GPIOG_CLK_ENABLE(); } while (0)

#define LCD_RS_GPIO_PORT                GPIOF
#define LCD_RS_GPIO_PIN                 GPIO_PIN_12
#define LCD_RS_GPIO_CLK_ENABLE()        do { __HAL_RCC_GPIOF_CLK_ENABLE(); } while (0)

/* FSMC Bank1 Sector4 (NE4 -> 0x6C000000), A6 -> offset 0x80 */
#define LCD_FSMC_NEX         4
#define LCD_FSMC_AX          6

#define LCD_FSMC_BCRX        FSMC_Bank1->BTCR[(LCD_FSMC_NEX - 1) * 2]
#define LCD_FSMC_BTRX        FSMC_Bank1->BTCR[(LCD_FSMC_NEX - 1) * 2 + 1]
#define LCD_FSMC_BWTRX       FSMC_Bank1E->BWTR[(LCD_FSMC_NEX - 1) * 2]

/******************************************************************************************/
/* LCD 参数结构体 */
typedef struct
{
    uint16_t width;     /* LCD 宽度 */
    uint16_t height;    /* LCD 高度 */
    uint16_t id;        /* LCD 驱动 IC ID */
    uint8_t  dir;       /* 0: 竖屏; 1: 横屏 */
    uint16_t wramcmd;   /* 开始写 GRAM 指令 */
    uint16_t setxcmd;   /* 设置 X 坐标指令 */
    uint16_t setycmd;   /* 设置 Y 坐标指令 */
} _lcd_dev;

extern _lcd_dev lcddev;
extern uint32_t g_point_color;
extern uint32_t g_back_color;

/* LCD 背光控制 */
#define LCD_BL(x)   do { \
                        HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, \
                                          (x) ? GPIO_PIN_SET : GPIO_PIN_RESET); \
                    } while (0)

/* LCD FSMC 地址结构体 */
typedef struct
{
    volatile uint16_t LCD_REG;
    volatile uint16_t LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE    (uint32_t)((0x60000000UL + (0x04000000UL * (LCD_FSMC_NEX - 1))) | (((1UL << LCD_FSMC_AX) * 2UL) - 2UL))
#define LCD         ((LCD_TypeDef *)LCD_BASE)

/******************************************************************************************/
/* 扫描方向定义 */
#define L2R_U2D         0
#define L2R_D2U         1
#define R2L_U2D         2
#define R2L_D2U         3
#define U2D_L2R         4
#define U2D_R2L         5
#define D2U_L2R         6
#define D2U_R2L         7
#define DFT_SCAN_DIR    L2R_U2D

/* RGB565 常用颜色 */
#define WHITE           0xFFFF
#define BLACK           0x0000
#define RED             0xF800
#define GREEN           0x07E0
#define BLUE            0x001F
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define CYAN            0x07FF
#define ORANGE          0xFD20
#define BROWN           0xBC40
#define BRRED           0xFC07
#define GRAY            0x8430
#define DARKGRAY        0x3186
#define DARKBLUE        0x01CF
#define LIGHTBLUE       0x7D7C
#define GRAYBLUE        0x5458
#define LIGHTGREEN      0x841F
#define LGRAY           0xC618
#define LGRAYBLUE       0xA651
#define LBBLUE          0x2B12

/* SSD1963 相关参数 */
#define SSD_HOR_RESOLUTION      800
#define SSD_VER_RESOLUTION      480
#define SSD_HOR_PULSE_WIDTH     1
#define SSD_HOR_BACK_PORCH      46
#define SSD_HOR_FRONT_PORCH     210
#define SSD_VER_PULSE_WIDTH     1
#define SSD_VER_BACK_PORCH      23
#define SSD_VER_FRONT_PORCH     22
#define SSD_HT                  (SSD_HOR_RESOLUTION + SSD_HOR_BACK_PORCH + SSD_HOR_FRONT_PORCH)
#define SSD_HPS                 (SSD_HOR_BACK_PORCH)
#define SSD_VT                  (SSD_VER_RESOLUTION + SSD_VER_BACK_PORCH + SSD_VER_FRONT_PORCH)
#define SSD_VPS                 (SSD_VER_BACK_PORCH)

/******************************************************************************************/
/* 驱动函数声明 */
void     lcd_wr_data(volatile uint16_t data);
void     lcd_wr_regno(volatile uint16_t regno);
void     lcd_write_reg(uint16_t regno, uint16_t data);

void     lcd_init(void);
void     lcd_display_on(void);
void     lcd_display_off(void);
void     lcd_scan_dir(uint8_t dir);
void     lcd_display_dir(uint8_t dir);
void     lcd_ssd_backlight_set(uint8_t pwm);

void     lcd_write_ram_prepare(void);
void     lcd_set_cursor(uint16_t x, uint16_t y);
uint32_t lcd_read_point(uint16_t x, uint16_t y);
void     lcd_draw_point(uint16_t x, uint16_t y, uint32_t color);

void     lcd_clear(uint16_t color);
void     lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void     lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void     lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color);
void     lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);
void     lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color);
void     lcd_fill_fast(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);
void     lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);
void     lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void     lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

void     lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color);
void     lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color);
void     lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color);
void     lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, const char *p, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */
