#include "app_baseline.h"
#include <math.h>
#include <stdio.h>

int main(void)
{
    BaselineModel_t model;
    BaselineEval_t  ev;
    float feat[BASELINE_FEAT_DIM];
    uint32_t i;

    printf("=== Welford Baseline Host Unit Tests ===\n");

    APP_Baseline_StartCal(&model, 30U);

    /* 1. 喂入 30 帧健康数据（带轻微波动） */
    for (i = 0U; i < 30U; i++)
    {
        float jitter = (i % 2U == 0U) ? 2.0f : -2.0f;
        feat[FEAT_RMS_MG]    = 250.0f + jitter;
        feat[FEAT_KURTOSIS]  = 3.20f  + 0.05f * jitter;
        feat[FEAT_CREST]     = 3.60f  + 0.05f * jitter;
        feat[FEAT_STEP_1X]   = 110.0f + jitter;
        feat[FEAT_STEP_2X]   = 45.0f  + 0.5f * jitter;
        feat[FEAT_STEP_3X]   = 30.0f  + 0.5f * jitter;
        feat[FEAT_LOW_BAND]  = 22.0f  + 0.5f * jitter;
        feat[FEAT_HIGH_BAND] = 35.0f  + 0.5f * jitter;
        APP_Baseline_Update(&model, feat, &ev);
    }

    if (ev.state != BASE_STATE_NORMAL)
    {
        printf("[FAIL] Expected NORMAL after 30 cal frames, got %u\n", (unsigned)ev.state);
        return 1;
    }
    if (fabsf(model.mean[FEAT_RMS_MG] - 250.0f) > 0.01f)
    {
        printf("[FAIL] Welford mean mismatch: %.4f\n", model.mean[FEAT_RMS_MG]);
        return 1;
    }
    printf("[PASS] Welford 30-frame calibration (mean_rms=%.2f, eff_std=%.2f)\n",
           model.mean[FEAT_RMS_MG], model.std[FEAT_RMS_MG]);

    /* 2. 连续 100 帧正常波动 -> 必须 0 误报 */
    for (i = 0U; i < 100U; i++)
    {
        float jitter = (float)((int)(i % 5U) - 2) * 3.0f;
        feat[FEAT_RMS_MG] = 250.0f + jitter;
        APP_Baseline_Update(&model, feat, &ev);
        if (ev.state != BASE_STATE_NORMAL)
        {
            printf("[FAIL] False positive on healthy frame %u (z=%.2f)\n", (unsigned)i, ev.max_z);
            return 1;
        }
    }
    printf("[PASS] 100 healthy frames zero false positives\n");

    /* 3. 注入 1 帧过载冲击 -> 必须立即触发 FAULT (code=2) */
    feat[FEAT_RMS_MG]  = 340.0f; /* +90 mg -> z = 90/12 = 7.5 >= 4.0 */
    feat[FEAT_STEP_3X] = 95.0f;
    APP_Baseline_Update(&model, feat, &ev);
    if ((ev.state != BASE_STATE_FAULT) || (ev.fault_code != 2U))
    {
        printf("[FAIL] Overload not detected (state=%u code=%u z=%.2f)\n",
               (unsigned)ev.state, (unsigned)ev.fault_code, ev.max_z);
        return 1;
    }
    printf("[PASS] Immediate 1-frame overload detection (z=%.2f, code=%u)\n",
           ev.max_z, (unsigned)ev.fault_code);

    /* 4. 滞回防抖测试：跌到 3.0 (介于 2.5..4.0 之间) 必须保持 FAULT */
    feat[FEAT_RMS_MG]  = 286.0f; /* z = 3.0 */
    feat[FEAT_STEP_3X] = 30.0f;
    for (i = 0U; i < 6U; i++)
    {
        APP_Baseline_Update(&model, feat, &ev);
        if (ev.state != BASE_STATE_FAULT)
        {
            printf("[FAIL] Hysteresis broken in deadband\n");
            return 1;
        }
    }

    /* 5. 连续 5 帧回归健康 (z < 2.5) -> 第 5 帧必须恢复 NORMAL */
    feat[FEAT_RMS_MG] = 252.0f;
    for (i = 0U; i < 4U; i++)
    {
        APP_Baseline_Update(&model, feat, &ev);
        if (ev.state != BASE_STATE_FAULT)
        {
            printf("[FAIL] Exited FAULT too early at frame %u\n", (unsigned)i + 1U);
            return 1;
        }
    }
    APP_Baseline_Update(&model, feat, &ev);
    if (ev.state != BASE_STATE_NORMAL)
    {
        printf("[FAIL] Did not recover to NORMAL after 5 healthy frames\n");
        return 1;
    }
    printf("[PASS] Hysteresis deadband & 5-frame recovery verified\n");
    printf("=== ALL BASELINE TESTS PASSED ===\n");
    return 0;
}
