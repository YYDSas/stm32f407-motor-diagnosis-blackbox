/**
 ****************************************************************************************************
 * @file        lcd.c
 * @brief       正点原子探索者 STM32F407 V3.5 TFTLCD (MCU FSMC 屏) 驱动源文件
 *              支持 ILI9341/NT35310/NT35510/SSD1963/ST7789/ST7796/ILI9806 自动识别
 * @note        1. 已去除正点原子 SYSTEM 私有依赖，改用 HAL_Delay / BSP_DWT_DelayUs / Log_Printf
 *              2. 已移除 HAL_SRAM_MspInit 避免与 CubeMX 生成的 fsmc.c 冲突，改为内部初始化总线 GPIO
 *              3. 已修正 lcd_show_string 横屏 x >= 256 截断 bug，并新增开窗高速填充 lcd_fill_fast
 ****************************************************************************************************
 */

#include "lcd.h"
#include "lcdfont.h"
#include "bsp_dwt.h"
#include "app_cli.h"

/* lcd_ex.c 仅通过 include 编入本文件，切勿单独添加到 Keil 编译组！ */
#include "lcd_ex.c"

SRAM_HandleTypeDef g_sram_handle;

uint32_t g_point_color = 0xF800U;
uint32_t g_back_color  = 0xFFFFU;

_lcd_dev lcddev;

void lcd_wr_data(volatile uint16_t data)
{
    data = data;
    LCD->LCD_RAM = data;
}

void lcd_wr_regno(volatile uint16_t regno)
{
    regno = regno;
    LCD->LCD_REG = regno;
}

void lcd_write_reg(uint16_t regno, uint16_t data)
{
    LCD->LCD_REG = regno;
    LCD->LCD_RAM = data;
}

static void lcd_opt_delay(volatile uint32_t i)
{
    while (i--)
    {
        __NOP();
    }
}

static uint16_t lcd_rd_data(void)
{
    volatile uint16_t ram;
    lcd_opt_delay(2U);
    ram = LCD->LCD_RAM;
    return ram;
}

void lcd_write_ram_prepare(void)
{
    LCD->LCD_REG = lcddev.wramcmd;
}

uint32_t lcd_read_point(uint16_t x, uint16_t y)
{
    uint16_t r = 0U, g = 0U, b = 0U;

    if ((x >= lcddev.width) || (y >= lcddev.height))
    {
        return 0U;
    }

    lcd_set_cursor(x, y);

    if (lcddev.id == 0x5510U)
    {
        lcd_wr_regno(0x2E00U);
    }
    else
    {
        lcd_wr_regno(0x2EU);
    }

    r = lcd_rd_data(); /* dummy read */
    if (lcddev.id == 0x1963U)
    {
        return r;
    }

    r = lcd_rd_data();
    if (lcddev.id == 0x7796U)
    {
        return r;
    }

    b = lcd_rd_data();
    g = (uint16_t)((r & 0xFFU) << 8);

    return (uint32_t)(((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));
}

void lcd_display_on(void)
{
    if (lcddev.id == 0x5510U)
    {
        lcd_wr_regno(0x2900U);
    }
    else
    {
        lcd_wr_regno(0x29U);
    }
}

void lcd_display_off(void)
{
    if (lcddev.id == 0x5510U)
    {
        lcd_wr_regno(0x2800U);
    }
    else
    {
        lcd_wr_regno(0x28U);
    }
}

void lcd_set_cursor(uint16_t x, uint16_t y)
{
    if (lcddev.id == 0x1963U)
    {
        if (lcddev.dir == 0U)
        {
            x = (uint16_t)(lcddev.width - 1U - x);
            lcd_wr_regno(lcddev.setxcmd);
            lcd_wr_data(0U);
            lcd_wr_data(0U);
            lcd_wr_data((uint16_t)(x >> 8));
            lcd_wr_data((uint16_t)(x & 0xFFU));
        }
        else
        {
            lcd_wr_regno(lcddev.setxcmd);
            lcd_wr_data((uint16_t)(x >> 8));
            lcd_wr_data((uint16_t)(x & 0xFFU));
            lcd_wr_data((uint16_t)((lcddev.width - 1U) >> 8));
            lcd_wr_data((uint16_t)((lcddev.width - 1U) & 0xFFU));
        }

        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data((uint16_t)(y >> 8));
        lcd_wr_data((uint16_t)(y & 0xFFU));
        lcd_wr_data((uint16_t)((lcddev.height - 1U) >> 8));
        lcd_wr_data((uint16_t)((lcddev.height - 1U) & 0xFFU));
    }
    else if (lcddev.id == 0x5510U)
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data((uint16_t)(x >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setxcmd + 1U));
        lcd_wr_data((uint16_t)(x & 0xFFU));
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data((uint16_t)(y >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setycmd + 1U));
        lcd_wr_data((uint16_t)(y & 0xFFU));
    }
    else
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data((uint16_t)(x >> 8));
        lcd_wr_data((uint16_t)(x & 0xFFU));
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data((uint16_t)(y >> 8));
        lcd_wr_data((uint16_t)(y & 0xFFU));
    }
}

void lcd_scan_dir(uint8_t dir)
{
    uint16_t regval = 0U;
    uint16_t dirreg = 0U;
    uint16_t temp;

    if (((lcddev.dir == 1U) && (lcddev.id != 0x1963U)) ||
        ((lcddev.dir == 0U) && (lcddev.id == 0x1963U)))
    {
        switch (dir)
        {
            case 0: dir = 6; break;
            case 1: dir = 7; break;
            case 2: dir = 4; break;
            case 3: dir = 5; break;
            case 4: dir = 1; break;
            case 5: dir = 0; break;
            case 6: dir = 3; break;
            case 7: dir = 2; break;
            default: break;
        }
    }

    switch (dir)
    {
        case L2R_U2D: regval |= (0U << 7) | (0U << 6) | (0U << 5); break;
        case L2R_D2U: regval |= (1U << 7) | (0U << 6) | (0U << 5); break;
        case R2L_U2D: regval |= (0U << 7) | (1U << 6) | (0U << 5); break;
        case R2L_D2U: regval |= (1U << 7) | (1U << 6) | (0U << 5); break;
        case U2D_L2R: regval |= (0U << 7) | (0U << 6) | (1U << 5); break;
        case U2D_R2L: regval |= (0U << 7) | (1U << 6) | (1U << 5); break;
        case D2U_L2R: regval |= (1U << 7) | (0U << 6) | (1U << 5); break;
        case D2U_R2L: regval |= (1U << 7) | (1U << 6) | (1U << 5); break;
        default: break;
    }

    dirreg = (lcddev.id == 0x5510U) ? 0x3600U : 0x36U;

    if ((lcddev.id == 0x9341U) || (lcddev.id == 0x7789U) || (lcddev.id == 0x7796U))
    {
        regval |= 0x08U;
    }

    lcd_write_reg(dirreg, regval);

    if (lcddev.id != 0x1963U)
    {
        if (regval & 0x20U)
        {
            if (lcddev.width < lcddev.height)
            {
                temp = lcddev.width;
                lcddev.width = lcddev.height;
                lcddev.height = temp;
            }
        }
        else
        {
            if (lcddev.width > lcddev.height)
            {
                temp = lcddev.width;
                lcddev.width = lcddev.height;
                lcddev.height = temp;
            }
        }
    }

    if (lcddev.id == 0x5510U)
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(0U);
        lcd_wr_regno((uint16_t)(lcddev.setxcmd + 1U));
        lcd_wr_data(0U);
        lcd_wr_regno((uint16_t)(lcddev.setxcmd + 2U));
        lcd_wr_data((uint16_t)((lcddev.width - 1U) >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setxcmd + 3U));
        lcd_wr_data((uint16_t)((lcddev.width - 1U) & 0xFFU));
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(0U);
        lcd_wr_regno((uint16_t)(lcddev.setycmd + 1U));
        lcd_wr_data(0U);
        lcd_wr_regno((uint16_t)(lcddev.setycmd + 2U));
        lcd_wr_data((uint16_t)((lcddev.height - 1U) >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setycmd + 3U));
        lcd_wr_data((uint16_t)((lcddev.height - 1U) & 0xFFU));
    }
    else
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(0U);
        lcd_wr_data(0U);
        lcd_wr_data((uint16_t)((lcddev.width - 1U) >> 8));
        lcd_wr_data((uint16_t)((lcddev.width - 1U) & 0xFFU));
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(0U);
        lcd_wr_data(0U);
        lcd_wr_data((uint16_t)((lcddev.height - 1U) >> 8));
        lcd_wr_data((uint16_t)((lcddev.height - 1U) & 0xFFU));
    }
}

void lcd_draw_point(uint16_t x, uint16_t y, uint32_t color)
{
    lcd_set_cursor(x, y);
    lcd_write_ram_prepare();
    LCD->LCD_RAM = (uint16_t)color;
}

void lcd_ssd_backlight_set(uint8_t pwm)
{
    uint16_t duty = (uint16_t)(((uint32_t)pwm * 255U) / 100U);
    lcd_wr_regno(0xBEU);
    lcd_wr_data(0x05U);
    lcd_wr_data(duty);
    lcd_wr_data(0x01U);
    lcd_wr_data(0xFFU);
    lcd_wr_data(0x00U);
    lcd_wr_data(0x00U);
}

void lcd_display_dir(uint8_t dir)
{
    lcddev.dir = dir;

    if (dir == 0U)
    {
        lcddev.width  = 240U;
        lcddev.height = 320U;

        if (lcddev.id == 0x5510U)
        {
            lcddev.wramcmd = 0x2C00U;
            lcddev.setxcmd = 0x2A00U;
            lcddev.setycmd = 0x2B00U;
            lcddev.width   = 480U;
            lcddev.height  = 800U;
        }
        else if (lcddev.id == 0x1963U)
        {
            lcddev.wramcmd = 0x2CU;
            lcddev.setxcmd = 0x2BU;
            lcddev.setycmd = 0x2AU;
            lcddev.width   = 480U;
            lcddev.height  = 800U;
        }
        else
        {
            lcddev.wramcmd = 0x2CU;
            lcddev.setxcmd = 0x2AU;
            lcddev.setycmd = 0x2BU;
        }

        if ((lcddev.id == 0x5310U) || (lcddev.id == 0x7796U))
        {
            lcddev.width  = 320U;
            lcddev.height = 480U;
        }
        if (lcddev.id == 0x9806U)
        {
            lcddev.width  = 480U;
            lcddev.height = 800U;
        }
    }
    else
    {
        lcddev.width  = 320U;
        lcddev.height = 240U;

        if (lcddev.id == 0x5510U)
        {
            lcddev.wramcmd = 0x2C00U;
            lcddev.setxcmd = 0x2A00U;
            lcddev.setycmd = 0x2B00U;
            lcddev.width   = 800U;
            lcddev.height  = 480U;
        }
        else if ((lcddev.id == 0x1963U) || (lcddev.id == 0x9806U))
        {
            lcddev.wramcmd = 0x2CU;
            lcddev.setxcmd = 0x2AU;
            lcddev.setycmd = 0x2BU;
            lcddev.width   = 800U;
            lcddev.height  = 480U;
        }
        else
        {
            lcddev.wramcmd = 0x2CU;
            lcddev.setxcmd = 0x2AU;
            lcddev.setycmd = 0x2BU;
        }

        if ((lcddev.id == 0x5310U) || (lcddev.id == 0x7796U))
        {
            lcddev.width  = 480U;
            lcddev.height = 320U;
        }
    }

    lcd_scan_dir(DFT_SCAN_DIR);
}

void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    uint16_t twidth  = (uint16_t)(sx + width - 1U);
    uint16_t theight = (uint16_t)(sy + height - 1U);

    if ((lcddev.id == 0x1963U) && (lcddev.dir != 1U))
    {
        sx = (uint16_t)(lcddev.width - width - sx);
        theight = (uint16_t)(sy + height - 1U);
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data((uint16_t)(sx >> 8));
        lcd_wr_data((uint16_t)(sx & 0xFFU));
        lcd_wr_data((uint16_t)((sx + width - 1U) >> 8));
        lcd_wr_data((uint16_t)((sx + width - 1U) & 0xFFU));
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data((uint16_t)(sy >> 8));
        lcd_wr_data((uint16_t)(sy & 0xFFU));
        lcd_wr_data((uint16_t)(theight >> 8));
        lcd_wr_data((uint16_t)(theight & 0xFFU));
    }
    else if (lcddev.id == 0x5510U)
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data((uint16_t)(sx >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setxcmd + 1U));
        lcd_wr_data((uint16_t)(sx & 0xFFU));
        lcd_wr_regno((uint16_t)(lcddev.setxcmd + 2U));
        lcd_wr_data((uint16_t)(twidth >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setxcmd + 3U));
        lcd_wr_data((uint16_t)(twidth & 0xFFU));
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data((uint16_t)(sy >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setycmd + 1U));
        lcd_wr_data((uint16_t)(sy & 0xFFU));
        lcd_wr_regno((uint16_t)(lcddev.setycmd + 2U));
        lcd_wr_data((uint16_t)(theight >> 8));
        lcd_wr_regno((uint16_t)(lcddev.setycmd + 3U));
        lcd_wr_data((uint16_t)(theight & 0xFFU));
    }
    else
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data((uint16_t)(sx >> 8));
        lcd_wr_data((uint16_t)(sx & 0xFFU));
        lcd_wr_data((uint16_t)(twidth >> 8));
        lcd_wr_data((uint16_t)(twidth & 0xFFU));
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data((uint16_t)(sy >> 8));
        lcd_wr_data((uint16_t)(sy & 0xFFU));
        lcd_wr_data((uint16_t)(theight >> 8));
        lcd_wr_data((uint16_t)(theight & 0xFFU));
    }
}

/* 内部总线引脚初始化（不覆盖 CubeMX 生成的 HAL_SRAM_MspInit，避免重复定义） */
static void lcd_fsmc_bus_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_FSMC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 |
                                 GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull      = GPIO_PULLUP;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    gpio_init_struct.Pin       = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                                 GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);
}

void lcd_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FSMC_NORSRAM_TimingTypeDef fsmc_read_handle;
    FSMC_NORSRAM_TimingTypeDef fsmc_write_handle;

    LCD_CS_GPIO_CLK_ENABLE();
    LCD_WR_GPIO_CLK_ENABLE();
    LCD_RD_GPIO_CLK_ENABLE();
    LCD_RS_GPIO_CLK_ENABLE();
    LCD_BL_GPIO_CLK_ENABLE();

    lcd_fsmc_bus_gpio_init();

    gpio_init_struct.Pin       = LCD_CS_GPIO_PIN;
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull      = GPIO_PULLUP;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(LCD_CS_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = LCD_WR_GPIO_PIN;
    HAL_GPIO_Init(LCD_WR_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = LCD_RD_GPIO_PIN;
    HAL_GPIO_Init(LCD_RD_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = LCD_RS_GPIO_PIN;
    HAL_GPIO_Init(LCD_RS_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin       = LCD_BL_GPIO_PIN;
    gpio_init_struct.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull      = GPIO_PULLUP;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = 0U;
    HAL_GPIO_Init(LCD_BL_GPIO_PORT, &gpio_init_struct);

    g_sram_handle.Instance                  = FSMC_NORSRAM_DEVICE;
    g_sram_handle.Extended                  = FSMC_NORSRAM_EXTENDED_DEVICE;
    g_sram_handle.Init.NSBank               = FSMC_NORSRAM_BANK4;
    g_sram_handle.Init.DataAddressMux       = FSMC_DATA_ADDRESS_MUX_DISABLE;
    g_sram_handle.Init.MemoryType           = FSMC_MEMORY_TYPE_SRAM;
    g_sram_handle.Init.MemoryDataWidth      = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    g_sram_handle.Init.BurstAccessMode      = FSMC_BURST_ACCESS_MODE_DISABLE;
    g_sram_handle.Init.WaitSignalPolarity   = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    g_sram_handle.Init.WaitSignalActive     = FSMC_WAIT_TIMING_BEFORE_WS;
    g_sram_handle.Init.WriteOperation       = FSMC_WRITE_OPERATION_ENABLE;
    g_sram_handle.Init.WaitSignal           = FSMC_WAIT_SIGNAL_DISABLE;
    g_sram_handle.Init.ExtendedMode         = FSMC_EXTENDED_MODE_ENABLE;
    g_sram_handle.Init.AsynchronousWait     = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    g_sram_handle.Init.WriteBurst           = FSMC_WRITE_BURST_DISABLE;
    g_sram_handle.Init.PageSize             = FSMC_PAGE_SIZE_NONE;

    /* FSMC 读时序 (168 MHz: 1 HCLK = 5.95 ns) */
    fsmc_read_handle.AddressSetupTime      = 15U;
    fsmc_read_handle.AddressHoldTime       = 0U;
    fsmc_read_handle.DataSetupTime         = 60U;
    fsmc_read_handle.BusTurnAroundDuration = 0U;
    fsmc_read_handle.CLKDivision           = 0U;
    fsmc_read_handle.DataLatency           = 0U;
    fsmc_read_handle.AccessMode            = FSMC_ACCESS_MODE_A;

    /* FSMC 写时序 (初始化阶段保守值，识别 IC 后提速) */
    fsmc_write_handle.AddressSetupTime      = 9U;
    fsmc_write_handle.AddressHoldTime       = 0U;
    fsmc_write_handle.DataSetupTime         = 9U;
    fsmc_write_handle.BusTurnAroundDuration = 0U;
    fsmc_write_handle.CLKDivision           = 0U;
    fsmc_write_handle.DataLatency           = 0U;
    fsmc_write_handle.AccessMode            = FSMC_ACCESS_MODE_A;

    HAL_SRAM_Init(&g_sram_handle, &fsmc_read_handle, &fsmc_write_handle);
    HAL_Delay(50U);

    /* 自动识别 LCD 驱动 IC ID */
    lcd_wr_regno(0xD3U);
    lcddev.id = lcd_rd_data();
    lcddev.id = lcd_rd_data();
    lcddev.id = lcd_rd_data();
    lcddev.id <<= 8;
    lcddev.id |= lcd_rd_data();

    if (lcddev.id != 0x9341U)
    {
        lcd_wr_regno(0x04U);
        lcddev.id = lcd_rd_data();
        lcddev.id = lcd_rd_data();
        lcddev.id = lcd_rd_data();
        lcddev.id <<= 8;
        lcddev.id |= lcd_rd_data();

        if (lcddev.id == 0x8552U)
        {
            lcddev.id = 0x7789U;
        }

        if (lcddev.id != 0x7789U)
        {
            lcd_wr_regno(0xD4U);
            lcddev.id = lcd_rd_data();
            lcddev.id = lcd_rd_data();
            lcddev.id = lcd_rd_data();
            lcddev.id <<= 8;
            lcddev.id |= lcd_rd_data();

            if (lcddev.id != 0x5310U)
            {
                lcd_wr_regno(0xD3U);
                lcddev.id = lcd_rd_data();
                lcddev.id = lcd_rd_data();
                lcddev.id = lcd_rd_data();
                lcddev.id <<= 8;
                lcddev.id |= lcd_rd_data();

                if (lcddev.id != 0x7796U)
                {
                    lcd_write_reg(0xF000U, 0x0055U);
                    lcd_write_reg(0xF001U, 0x00AAU);
                    lcd_write_reg(0xF002U, 0x0052U);
                    lcd_write_reg(0xF003U, 0x0008U);
                    lcd_write_reg(0xF004U, 0x0001U);

                    lcd_wr_regno(0xC500U);
                    lcddev.id = lcd_rd_data();
                    lcddev.id <<= 8;

                    lcd_wr_regno(0xC501U);
                    lcddev.id |= lcd_rd_data();

                    HAL_Delay(5U);

                    if (lcddev.id != 0x5510U)
                    {
                        lcd_wr_regno(0xD3U);
                        lcddev.id = lcd_rd_data();
                        lcddev.id = lcd_rd_data();
                        lcddev.id = lcd_rd_data();
                        lcddev.id <<= 8;
                        lcddev.id |= lcd_rd_data();

                        if (lcddev.id != 0x9806U)
                        {
                            lcd_wr_regno(0xA1U);
                            lcddev.id = lcd_rd_data();
                            lcddev.id = lcd_rd_data();
                            lcddev.id <<= 8;
                            lcddev.id |= lcd_rd_data();

                            if (lcddev.id == 0x5761U)
                            {
                                lcddev.id = 0x1963U;
                            }
                        }
                    }
                }
            }
        }
    }

    if (lcddev.id == 0x7789U)
    {
        lcd_ex_st7789_reginit();
    }
    else if (lcddev.id == 0x9341U)
    {
        lcd_ex_ili9341_reginit();
    }
    else if (lcddev.id == 0x5310U)
    {
        lcd_ex_nt35310_reginit();
    }
    else if (lcddev.id == 0x7796U)
    {
        lcd_ex_st7796_reginit();
    }
    else if (lcddev.id == 0x5510U)
    {
        lcd_ex_nt35510_reginit();
    }
    else if (lcddev.id == 0x9806U)
    {
        lcd_ex_ili9806_reginit();
    }
    else if (lcddev.id == 0x1963U)
    {
        lcd_ex_ssd1963_reginit();
        lcd_ssd_backlight_set(100U);
    }

    /* 初始化完成后按 IC 提速 FSMC 写时序 */
    if (lcddev.id == 0x7789U)
    {
        fsmc_write_handle.AddressSetupTime = 3U;
        fsmc_write_handle.DataSetupTime    = 3U;
        FSMC_NORSRAM_Extended_Timing_Init(g_sram_handle.Extended, &fsmc_write_handle,
                                          g_sram_handle.Init.NSBank, g_sram_handle.Init.ExtendedMode);
    }
    else if ((lcddev.id == 0x9806U) || (lcddev.id == 0x9341U) || (lcddev.id == 0x5510U))
    {
        fsmc_write_handle.AddressSetupTime = 2U;
        fsmc_write_handle.DataSetupTime    = 2U;
        FSMC_NORSRAM_Extended_Timing_Init(g_sram_handle.Extended, &fsmc_write_handle,
                                          g_sram_handle.Init.NSBank, g_sram_handle.Init.ExtendedMode);
    }
    else if ((lcddev.id == 0x5310U) || (lcddev.id == 0x7796U) || (lcddev.id == 0x1963U))
    {
        fsmc_write_handle.AddressSetupTime = 1U;
        fsmc_write_handle.DataSetupTime    = 1U;
        FSMC_NORSRAM_Extended_Timing_Init(g_sram_handle.Extended, &fsmc_write_handle,
                                          g_sram_handle.Init.NSBank, g_sram_handle.Init.ExtendedMode);
    }

    lcd_display_dir(1U); /* 默认横屏 (3.5寸 480x320) */
    LCD_BL(1);
    lcd_clear(BLACK);

    Log_Printf("[LCD] Init OK: ID=0x%04X (%ux%u landscape)\r\n",
               (unsigned int)lcddev.id, (unsigned int)lcddev.width, (unsigned int)lcddev.height);
}

void lcd_clear(uint16_t color)
{
    uint32_t index;
    uint32_t totalpoint = (uint32_t)lcddev.width * (uint32_t)lcddev.height;

    lcd_set_window(0U, 0U, lcddev.width, lcddev.height);
    lcd_set_cursor(0U, 0U);
    lcd_write_ram_prepare();

    for (index = 0U; index < totalpoint; index++)
    {
        LCD->LCD_RAM = color;
    }
}

void lcd_fill_fast(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    uint16_t w, h;
    uint32_t total, i;

    if ((sx > ex) || (sy > ey) || (sx >= lcddev.width) || (sy >= lcddev.height))
    {
        return;
    }
    if (ex >= lcddev.width)  ex = (uint16_t)(lcddev.width - 1U);
    if (ey >= lcddev.height) ey = (uint16_t)(lcddev.height - 1U);

    w = (uint16_t)(ex - sx + 1U);
    h = (uint16_t)(ey - sy + 1U);
    total = (uint32_t)w * (uint32_t)h;

    lcd_set_window(sx, sy, w, h);
    lcd_write_ram_prepare();
    for (i = 0U; i < total; i++)
    {
        LCD->LCD_RAM = color;
    }
    /* 恢复全屏窗口，避免影响后续单点写 */
    lcd_set_window(0U, 0U, lcddev.width, lcddev.height);
}

void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
{
    lcd_fill_fast(sx, sy, ex, ey, (uint16_t)color);
}

void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    uint16_t height, width;
    uint16_t i, j;

    width  = (uint16_t)(ex - sx + 1U);
    height = (uint16_t)(ey - sy + 1U);

    for (i = 0U; i < height; i++)
    {
        lcd_set_cursor(sx, (uint16_t)(sy + i));
        lcd_write_ram_prepare();
        for (j = 0U; j < width; j++)
        {
            LCD->LCD_RAM = color[i * width + j];
        }
    }
}

void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;

    delta_x = (int)x2 - (int)x1;
    delta_y = (int)y2 - (int)y1;
    row = (int)x1;
    col = (int)y1;

    if (delta_x > 0)       incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }

    if (delta_y > 0)       incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }

    distance = (delta_x > delta_y) ? delta_x : delta_y;

    for (t = 0U; t <= (uint16_t)(distance + 1); t++)
    {
        lcd_draw_point((uint16_t)row, (uint16_t)col, color);
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color)
{
    if ((len == 0U) || (x >= lcddev.width) || (y >= lcddev.height))
    {
        return;
    }
    lcd_fill_fast(x, y, (uint16_t)(x + len - 1U), y, color);
}

void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    lcd_fill_fast(x1, y1, x2, y1, color);
    lcd_fill_fast(x1, y2, x2, y2, color);
    lcd_fill_fast(x1, y1, x1, y2, color);
    lcd_fill_fast(x2, y1, x2, y2, color);
}

void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a = 0;
    int b = (int)r;
    int di = 3 - (b << 1);

    while (a <= b)
    {
        lcd_draw_point((uint16_t)(x0 + a), (uint16_t)(y0 - b), color);
        lcd_draw_point((uint16_t)(x0 + b), (uint16_t)(y0 - a), color);
        lcd_draw_point((uint16_t)(x0 + b), (uint16_t)(y0 + a), color);
        lcd_draw_point((uint16_t)(x0 + a), (uint16_t)(y0 + b), color);
        lcd_draw_point((uint16_t)(x0 - a), (uint16_t)(y0 + b), color);
        lcd_draw_point((uint16_t)(x0 - b), (uint16_t)(y0 + a), color);
        lcd_draw_point((uint16_t)(x0 - a), (uint16_t)(y0 - b), color);
        lcd_draw_point((uint16_t)(x0 - b), (uint16_t)(y0 - a), color);
        a++;

        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    uint32_t i;
    uint32_t imax  = ((uint32_t)r * 707U) / 1000U + 1U;
    uint32_t sqmax = (uint32_t)r * (uint32_t)r + (uint32_t)r / 2U;
    uint32_t xr    = r;

    lcd_draw_hline((uint16_t)(x - r), y, (uint16_t)(2U * r), color);

    for (i = 1U; i <= imax; i++)
    {
        if ((i * i + xr * xr) > sqmax)
        {
            if (xr > imax)
            {
                lcd_draw_hline((uint16_t)(x - i + 1U), (uint16_t)(y + xr), (uint16_t)(2U * (i - 1U)), color);
                lcd_draw_hline((uint16_t)(x - i + 1U), (uint16_t)(y - xr), (uint16_t)(2U * (i - 1U)), color);
            }
            xr--;
        }
        lcd_draw_hline((uint16_t)(x - xr), (uint16_t)(y + i), (uint16_t)(2U * xr), color);
        lcd_draw_hline((uint16_t)(x - xr), (uint16_t)(y - i), (uint16_t)(2U * xr), color);
    }
}

void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t temp, t1, t;
    uint16_t y0 = y;
    uint8_t csize;
    const uint8_t *pfont;
    uint8_t idx;

    if ((chr < ' ') || (chr > '~'))
    {
        chr = ' ';
    }
    idx   = (uint8_t)(chr - ' ');
    csize = (uint8_t)(((size / 8U) + ((size % 8U) ? 1U : 0U)) * (size / 2U));

    switch (size)
    {
        case 12: pfont = asc2_1206[idx]; break;
        case 16: pfont = asc2_1608[idx]; break;
        case 24: pfont = asc2_2412[idx]; break;
        case 32: pfont = asc2_3216[idx]; break;
        default: return;
    }

    for (t = 0U; t < csize; t++)
    {
        temp = pfont[t];
        for (t1 = 0U; t1 < 8U; t1++)
        {
            if (temp & 0x80U)
            {
                lcd_draw_point(x, y, color);
            }
            else if (mode == 0U)
            {
                lcd_draw_point(x, y, g_back_color);
            }

            temp <<= 1;
            y++;

            if (y >= lcddev.height)
            {
                return;
            }
            if ((uint16_t)(y - y0) == (uint16_t)size)
            {
                y = y0;
                x++;
                if (x >= lcddev.width)
                {
                    return;
                }
                break;
            }
        }
    }
}

static uint32_t lcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1U;
    while (n--)
    {
        result *= m;
    }
    return result;
}

void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0U;

    for (t = 0U; t < len; t++)
    {
        temp = (uint8_t)((num / lcd_pow(10U, (uint8_t)(len - t - 1U))) % 10U);
        if ((enshow == 0U) && (t < (uint8_t)(len - 1U)))
        {
            if (temp == 0U)
            {
                lcd_show_char((uint16_t)(x + (size / 2U) * t), y, ' ', size, 0U, color);
                continue;
            }
            else
            {
                enshow = 1U;
            }
        }
        lcd_show_char((uint16_t)(x + (size / 2U) * t), y, (char)(temp + '0'), size, 0U, color);
    }
}

void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0U;

    for (t = 0U; t < len; t++)
    {
        temp = (uint8_t)((num / lcd_pow(10U, (uint8_t)(len - t - 1U))) % 10U);
        if ((enshow == 0U) && (t < (uint8_t)(len - 1U)))
        {
            if (temp == 0U)
            {
                char pad = (mode & 0x80U) ? '0' : ' ';
                lcd_show_char((uint16_t)(x + (size / 2U) * t), y, pad, size, (uint8_t)(mode & 0x01U), color);
                continue;
            }
            else
            {
                enshow = 1U;
            }
        }
        lcd_show_char((uint16_t)(x + (size / 2U) * t), y, (char)(temp + '0'), size, (uint8_t)(mode & 0x01U), color);
    }
}

void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, const char *p, uint16_t color)
{
    uint16_t x0 = x; /* 修正原版 uint8_t 截断导致横屏 x>=256 换行异常的问题 */

    width  = (uint16_t)(width + x);
    height = (uint16_t)(height + y);

    while ((*p <= '~') && (*p >= ' '))
    {
        if (x >= width)
        {
            x = x0;
            y = (uint16_t)(y + size);
        }
        if (y >= height)
        {
            break;
        }
        lcd_show_char(x, y, *p, size, 0U, color);
        x = (uint16_t)(x + size / 2U);
        p++;
    }
}
