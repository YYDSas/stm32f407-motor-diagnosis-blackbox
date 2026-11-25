#include "bsp_stepper.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

/* Ì½Ë÷Õß V3.5 ¶ÀÁ¢ GPIO Ó³Éä: IN1=PB6, IN2=PB7, IN3=PC7, IN4=PF6 */
#define STEP_IN1_PORT  GPIOB
#define STEP_IN1_PIN   GPIO_PIN_6
#define STEP_IN2_PORT  GPIOB
#define STEP_IN2_PIN   GPIO_PIN_7
#define STEP_IN3_PORT  GPIOC
#define STEP_IN3_PIN   GPIO_PIN_7
#define STEP_IN4_PORT  GPIOF
#define STEP_IN4_PIN   GPIO_PIN_6

/* 28BYJ-48 4 Ïà 8 ÅÄ°ë²½¼¤Àø±í */
static const uint8_t s_half_step_lut[8][4] = {
    {1U, 0U, 0U, 0U},
    {1U, 1U, 0U, 0U},
    {0U, 1U, 0U, 0U},
    {0U, 1U, 1U, 0U},
    {0U, 0U, 1U, 0U},
    {0U, 0U, 1U, 1U},
    {0U, 0U, 0U, 1U},
    {1U, 0U, 0U, 1U}
};

static volatile uint8_t  s_phase_idx = 0U;
static volatile uint8_t  s_mode      = 0U;
static volatile uint16_t s_step_hz   = 0U;

static void stepper_write_coils(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    HAL_GPIO_WritePin(STEP_IN1_PORT, STEP_IN1_PIN, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEP_IN2_PORT, STEP_IN2_PIN, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEP_IN3_PORT, STEP_IN3_PIN, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEP_IN4_PORT, STEP_IN4_PIN, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_Stepper_SetMode(uint8_t mode)
{
    uint32_t arr = 2499U;

    HAL_TIM_Base_Stop_IT(&htim7);

    if (mode == 0U)
    {
        s_mode    = 0U;
        s_step_hz = 0U;
        stepper_write_coils(0U, 0U, 0U, 0U);
        return;
    }

    if (mode == 1U)
    {
        s_mode    = 1U;
        s_step_hz = 300U;
        arr       = 3332U; /* 1 MHz / 3333 = 300.03 Hz */
    }
    else if (mode == 3U)
    {
        s_mode    = 3U;
        s_step_hz = 500U;
        arr       = 1999U; /* 1 MHz / 2000 = 500.00 Hz */
    }
    else
    {
        s_mode    = 2U;
        s_step_hz = 400U;
        arr       = 2499U; /* 1 MHz / 2500 = 400.00 Hz */
    }

    __HAL_TIM_SET_AUTORELOAD(&htim7, arr);
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    HAL_TIM_Base_Start_IT(&htim7);
}

uint16_t BSP_Stepper_GetStepHz(void)
{
    return s_step_hz;
}

uint8_t BSP_Stepper_GetMode(void)
{
    return s_mode;
}

void BSP_Stepper_OnTimPeriodElapsed(void)
{
    uint8_t idx = s_phase_idx;
    stepper_write_coils(s_half_step_lut[idx][0],
                        s_half_step_lut[idx][1],
                        s_half_step_lut[idx][2],
                        s_half_step_lut[idx][3]);
    s_phase_idx = (uint8_t)((idx + 1U) & 0x07U);
}
