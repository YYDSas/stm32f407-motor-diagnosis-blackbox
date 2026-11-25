#include "bsp_oled.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include <string.h>

/* 探索者 V3.5 引脚: OLED_CS=PE6, OLED_DC=PC13, ADXL_CS=PE5(保持高电平防冲突) */
#define OLED_CS_PORT   GPIOE
#define OLED_CS_PIN    GPIO_PIN_6
#define OLED_DC_PORT   GPIOC
#define OLED_DC_PIN    GPIO_PIN_13
#define ADXL_CS_PORT   GPIOE
#define ADXL_CS_PIN    GPIO_PIN_5

/* 128x64 静态显存 (1024 B 放 .bss) */
static uint8_t s_fb[OLED_PAGES][OLED_WIDTH];

/* 精简 5x7 ASCII 字模 (' ' 0x20 .. 'Z' 0x5A，覆盖数字/大写字母/标点) */
static const uint8_t s_font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' 32 */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' 33 */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' 34 */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' 35 */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' 36 */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' 37 */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' 38 */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' 39 */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' 40 */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' 41 */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' 42 */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' 43 */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' 44 */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' 45 */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' 46 */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' 47 */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' 48 */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' 49 */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' 50 */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' 51 */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' 52 */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' 53 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' 54 */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' 55 */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' 56 */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' 57 */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' 58 */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' 59 */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' 60 */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' 61 */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' 62 */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' 63 */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' 64 */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' 65 */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' 66 */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' 67 */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' 68 */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' 69 */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' 70 */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' 71 */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' 72 */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' 73 */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' 74 */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' 75 */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' 76 */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' 77 */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' 78 */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' 79 */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' 80 */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' 81 */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' 82 */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' 83 */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' 84 */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' 85 */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' 86 */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' 87 */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' 88 */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' 89 */
    {0x61,0x51,0x49,0x45,0x43}  /* 'Z' 90 */
};

static void oled_write_cmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(ADXL_CS_PORT, ADXL_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1U, 10U);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
}

static void oled_write_data(const uint8_t *data, uint16_t len)
{
    HAL_GPIO_WritePin(ADXL_CS_PORT, ADXL_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)data, len, 20U);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
}

void BSP_OLED_Init(void)
{
    static const uint8_t init_seq[] = {
        0xAEU,        /* Display OFF */
        0xD5U, 0x80U, /* Clock divide ratio */
        0xA8U, 0x3FU, /* Multiplex = 64 */
        0xD3U, 0x00U, /* Display offset = 0 */
        0x40U,        /* Start line = 0 */
        0x8DU, 0x14U, /* Charge pump ON */
        0x20U, 0x02U, /* Page addressing mode */
        0xA1U,        /* Segment remap */
        0xC8U,        /* COM scan direction */
        0xDAU, 0x12U, /* COM pins config */
        0x81U, 0xCFU, /* Contrast */
        0xD9U, 0xF1U, /* Pre-charge period */
        0xDBU, 0x40U, /* VCOMH deselect */
        0xA4U,        /* Entire display ON resume */
        0xA6U,        /* Normal display */
        0xAFU         /* Display ON */
    };
    uint32_t i;

    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
    for (i = 0U; i < (uint32_t)sizeof(init_seq); i++)
    {
        oled_write_cmd(init_seq[i]);
    }

    BSP_OLED_Clear();
    for (i = 0U; i < OLED_PAGES; i++)
    {
        BSP_OLED_FlushPage((uint8_t)i);
    }
}

void BSP_OLED_Clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void BSP_OLED_DrawStr(uint8_t page, uint8_t x, const char *str)
{
    if ((page >= OLED_PAGES) || (str == NULL))
    {
        return;
    }
    while ((*str != '\0') && ((uint16_t)x + 5U < OLED_WIDTH))
    {
        char ch = *str++;
        uint8_t idx;
        uint8_t col;

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch = (char)(ch - 'a' + 'A');
        }
        if ((ch < ' ') || (ch > 'Z'))
        {
            ch = ' ';
        }
        idx = (uint8_t)(ch - ' ');
        for (col = 0U; col < 5U; col++)
        {
            s_fb[page][x + col] = s_font5x7[idx][col];
        }
        s_fb[page][x + 5U] = 0x00U;
        x = (uint8_t)(x + 6U);
    }
}

void BSP_OLED_DrawSpectrum64(const uint8_t bars_h[64])
{
    uint8_t b;
    uint8_t y;

    if (bars_h == NULL)
    {
        return;
    }

    /* 清空下半区 Page 2..7 (y=16..63) */
    for (y = 2U; y < OLED_PAGES; y++)
    {
        memset(s_fb[y], 0, OLED_WIDTH);
    }

    /* 绘制底线 (y=63) 并填充每根 2px 宽的频谱柱（留 1px 间距更清晰） */
    for (b = 0U; b < 64U; b++)
    {
        uint8_t h = (bars_h[b] > 48U) ? 48U : bars_h[b];
        uint8_t x = (uint8_t)(b * 2U);
        uint8_t row;

        s_fb[7][x]      |= 0x80U;
        s_fb[7][x + 1U] |= 0x80U;

        for (row = 0U; row < h; row++)
        {
            uint8_t py   = (uint8_t)(63U - row);
            uint8_t page = (uint8_t)(py >> 3);
            uint8_t bit  = (uint8_t)(1U << (py & 0x07U));
            s_fb[page][x] |= bit;
        }
    }
}

void BSP_OLED_FlushPage(uint8_t page)
{
    if (page >= OLED_PAGES)
    {
        return;
    }
    oled_write_cmd((uint8_t)(0xB0U | page));
    oled_write_cmd(0x00U);
    oled_write_cmd(0x10U);
    oled_write_data(s_fb[page], OLED_WIDTH);
}
