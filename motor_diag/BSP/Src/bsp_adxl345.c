#include "bsp_adxl345.h"
#include "bsp_dwt.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include <math.h>
#include <string.h>

/* SPI2 片选定义: ADXL_CS=PE5, OLED_CS=PE6 (初始化前拉高防冲突) */
#define ADXL_CS_PORT     GPIOE
#define ADXL_CS_PIN      GPIO_PIN_5
#define OLED_CS_PORT     GPIOE
#define OLED_CS_PIN      GPIO_PIN_6

/* ADXL345 寄存器地址 */
#define REG_DEVID        0x00U
#define REG_BW_RATE      0x2CU
#define REG_POWER_CTL    0x2DU
#define REG_INT_ENABLE   0x2EU
#define REG_INT_MAP      0x2FU
#define REG_INT_SOURCE   0x30U
#define REG_DATA_FORMAT  0x31U
#define REG_DATAX0       0x32U
#define REG_FIFO_CTL     0x38U
#define REG_FIFO_STATUS  0x39U

static volatile uint32_t s_exti_cnt = 0U;

static void adxl_cs_low(void)
{
    HAL_GPIO_WritePin(ADXL_CS_PORT, ADXL_CS_PIN, GPIO_PIN_RESET);
}

static void adxl_cs_high(void)
{
    HAL_GPIO_WritePin(ADXL_CS_PORT, ADXL_CS_PIN, GPIO_PIN_SET);
}

static uint8_t adxl_read_reg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0U, 0U};

    tx[0] = (uint8_t)((reg & 0x3FU) | 0x80U);
    tx[1] = 0x00U;

    adxl_cs_low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 10U);
    adxl_cs_high();

    return rx[1];
}

static void adxl_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];

    tx[0] = (uint8_t)(reg & 0x3FU);
    tx[1] = val;

    adxl_cs_low();
    HAL_SPI_Transmit(&hspi2, tx, 2U, 10U);
    adxl_cs_high();
}

static void adxl_read_xyz_once(ADXL345_Sample_t *s)
{
    uint8_t tx[7] = {0U};
    uint8_t rx[7] = {0U};

    /* 0xC0 = Read(0x80) | Multi-Byte(0x40) */
    tx[0] = (uint8_t)(REG_DATAX0 | 0xC0U);

    adxl_cs_low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 7U, 10U);
    adxl_cs_high();

    s->x = (int16_t)((uint16_t)rx[1] | ((uint16_t)rx[2] << 8));
    s->y = (int16_t)((uint16_t)rx[3] | ((uint16_t)rx[4] << 8));
    s->z = (int16_t)((uint16_t)rx[5] | ((uint16_t)rx[6] << 8));
}

uint8_t BSP_ADXL345_ReadDevId(void)
{
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
    adxl_cs_high();
    return adxl_read_reg(REG_DEVID);
}

uint8_t BSP_ADXL345_Init(uint8_t use_int2)
{
    uint8_t devid;
    ADXL345_Sample_t dummy[32];

    /* 确保共用 SPI2 的 OLED 片选处于高电平释放状态 */
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
    adxl_cs_high();

    devid = adxl_read_reg(REG_DEVID);
    if (devid != ADXL345_DEVID_EXPECTED)
    {
        return 0U;
    }

    /* 1. 先进入待机模式再修改参数 */
    adxl_write_reg(REG_POWER_CTL, 0x00U);
    adxl_write_reg(REG_INT_ENABLE, 0x00U);

    /* 2. 3200 Hz ODR, +/-2g Full-Res (3.9 mg/LSB), 4 线 SPI, 高电平有效中断 */
    adxl_write_reg(REG_BW_RATE, 0x0FU);
    adxl_write_reg(REG_DATA_FORMAT, 0x08U);

    /* 3. FIFO Stream 模式 + 水位 16 (0x80 | 16 = 0x90) */
    adxl_write_reg(REG_FIFO_CTL, (uint8_t)(0x80U | (ADXL345_FIFO_WATERMARK & 0x1FU)));

    /* 4. 映射 Watermark 中断到 INT1 (0x00) 或 INT2 (0xFF) */
    adxl_write_reg(REG_INT_MAP, use_int2 ? 0xFFU : 0x00U);
    adxl_write_reg(REG_INT_ENABLE, 0x02U); /* Bit 1 = Watermark */

    /* 5. 启动测量并清空残留 FIFO，确保后续产生干净的上升沿 */
    s_exti_cnt = 0U;
    adxl_write_reg(REG_POWER_CTL, 0x08U);
    (void)adxl_read_reg(REG_INT_SOURCE);
    (void)BSP_ADXL345_ReadFifoBatch(dummy, 32U, NULL);

    return 1U;
}

uint8_t BSP_ADXL345_ReadFifoBatch(ADXL345_Sample_t *out, uint8_t max_cnt, uint8_t *out_overrun)
{
    uint8_t int_src;
    uint8_t fifo_cnt;
    uint8_t n;
    uint8_t i;

    int_src = adxl_read_reg(REG_INT_SOURCE);
    if (out_overrun != NULL)
    {
        *out_overrun = (uint8_t)(int_src & 0x01U);
    }

    fifo_cnt = (uint8_t)(adxl_read_reg(REG_FIFO_STATUS) & 0x3FU);
    if (fifo_cnt == 0U)
    {
        return 0U;
    }

    n = (fifo_cnt < max_cnt) ? fifo_cnt : max_cnt;
    for (i = 0U; i < n; i++)
    {
        adxl_read_xyz_once(&out[i]);
        /* ADXL345 手册要求连续 FIFO 读出间隔 >= 5 us 以便内部指针弹出 */
        BSP_DWT_DelayUs(6U);
    }
    return n;
}


uint32_t BSP_ADXL345_GetExtiCount(void)
{
    return s_exti_cnt;
}
void BSP_ADXL345_OnExtiIrq(void)
{
    s_exti_cnt++;
}
uint8_t BSP_ADXL345_RunSnrTest(uint32_t sample_cnt, ADXL345_SnrStats_t *out)
{
    /* 静态缓存防止任务栈溢出 */
    static ADXL345_Sample_t s_buf[1024];
    ADXL345_Sample_t batch[32];
    uint32_t collected = 0U;
    uint32_t ovr_cnt   = 0U;
    uint32_t exti_start;
    uint32_t t0;
    uint32_t i;
    double   mx = 0.0, my = 0.0, mz = 0.0;
    double   vx = 0.0, vy = 0.0, vz = 0.0, m4z = 0.0;
    float    z_min = 1e9f, z_max = -1e9f;

    if ((out == NULL) || (sample_cnt == 0U))
    {
        return 0U;
    }
    if (sample_cnt > 1024U)
    {
        sample_cnt = 1024U;
    }

    memset(out, 0, sizeof(ADXL345_SnrStats_t));
    /* 先清空旧 FIFO */
    (void)BSP_ADXL345_ReadFifoBatch(batch, 32U, NULL);

    exti_start = s_exti_cnt;
    t0 = HAL_GetTick();

    while (collected < sample_cnt)
    {
        uint8_t ovr = 0U;
        uint8_t n = BSP_ADXL345_ReadFifoBatch(batch, 32U, &ovr);
        if (ovr != 0U)
        {
            ovr_cnt++;
        }
        for (i = 0U; (i < (uint32_t)n) && (collected < sample_cnt); i++)
        {
            s_buf[collected++] = batch[i];
        }
        /* 超时保护 (2000 ms) */
        if ((HAL_GetTick() - t0) > 2000U)
        {
            return 0U;
        }
    }

    out->cost_ms      = HAL_GetTick() - t0;
    out->exti_cnt     = s_exti_cnt - exti_start;
    out->overrun_cnt  = ovr_cnt;
    out->samples_read = collected;

    for (i = 0U; i < collected; i++)
    {
        mx += (double)s_buf[i].x * (double)ADXL345_MG_PER_LSB;
        my += (double)s_buf[i].y * (double)ADXL345_MG_PER_LSB;
        mz += (double)s_buf[i].z * (double)ADXL345_MG_PER_LSB;
    }
    mx /= (double)collected;
    my /= (double)collected;
    mz /= (double)collected;

    for (i = 0U; i < collected; i++)
    {
        double dx = (double)s_buf[i].x * (double)ADXL345_MG_PER_LSB - mx;
        double dy = (double)s_buf[i].y * (double)ADXL345_MG_PER_LSB - my;
        double dz = (double)s_buf[i].z * (double)ADXL345_MG_PER_LSB - mz;
        double dz2 = dz * dz;

        vx  += dx * dx;
        vy  += dy * dy;
        vz  += dz2;
        m4z += dz2 * dz2;

        if ((float)dz < z_min) z_min = (float)dz;
        if ((float)dz > z_max) z_max = (float)dz;
    }

    vx  /= (double)collected;
    vy  /= (double)collected;
    vz  /= (double)collected;
    m4z /= (double)collected;

    out->rms_x_mg = (float)sqrt(vx);
    out->rms_y_mg = (float)sqrt(vy);
    out->rms_z_mg = (float)sqrt(vz);
    out->p2p_z_mg = z_max - z_min;
    out->kurt_z   = (vz > 1e-6) ? (float)(m4z / (vz * vz)) : 0.0f;

    return 1U;
}
