#include "app_baseline.h"
#include <math.h>
#include <string.h>

/* 各维标准差下限保护：防止极平稳特征方差过小引发误报 */
static const float s_default_min_std[BASELINE_FEAT_DIM] = {
    12.0f,  /* RMS (mg) */
    2.5f,  /* Kurtosis */
    1.5f,  /* Crest Factor */
    8.0f,   /* Step 1x (mg) */
    6.0f,   /* Step 2x (mg) */
    6.0f,   /* Step 3x (mg) */
    8.0f,   /* Low band (mg) */
    8.0f    /* High band (mg) */
};

static const float s_default_weight[BASELINE_FEAT_DIM] = {
    1.0f, 0.6f, 0.8f, 1.0f, 1.0f, 1.0f, 1.0f, 0.9f
};

void APP_Baseline_StartCal(BaselineModel_t *m, uint16_t cal_frames)
{
    uint32_t i;
    if (m == NULL)
    {
        return;
    }

    memset(m, 0, sizeof(BaselineModel_t));
    m->state      = (uint8_t)BASE_STATE_CALIBRATING;
    m->cal_target = (cal_frames < 5U) ? BASELINE_DEFAULT_FRAMES : cal_frames;
    m->th_high    = BASELINE_TH_HIGH_DEFAULT;
    m->th_low     = BASELINE_TH_LOW_DEFAULT;

    for (i = 0U; i < BASELINE_FEAT_DIM; i++)
    {
        m->min_std[i] = s_default_min_std[i];
        m->weight[i]  = s_default_weight[i];
    }
}

void APP_Baseline_Update(BaselineModel_t *m,
                         const float feat[BASELINE_FEAT_DIM],
                         BaselineEval_t *out)
{
    uint32_t i;
    float max_z = 0.0f;
    uint8_t dom = 0U;

    if ((m == NULL) || (feat == NULL) || (out == NULL))
    {
        return;
    }

    memset(out, 0, sizeof(BaselineEval_t));

    if (m->state == (uint8_t)BASE_STATE_UNINIT)
    {
        APP_Baseline_StartCal(m, BASELINE_DEFAULT_FRAMES);
    }

    /* 阶段 1: Welford 在线单遍递推统计健康均值与方差 */
    if (m->state == (uint8_t)BASE_STATE_CALIBRATING)
    {
        m->cal_count++;
        for (i = 0U; i < BASELINE_FEAT_DIM; i++)
        {
            float delta  = feat[i] - m->mean[i];
            m->mean[i]  += delta / (float)m->cal_count;
            float delta2 = feat[i] - m->mean[i];
            m->m2[i]    += delta * delta2;
        }

        if (m->cal_count >= m->cal_target)
        {
            for (i = 0U; i < BASELINE_FEAT_DIM; i++)
            {
                float var = m->m2[i] / (float)(m->cal_count - 1U);
                float sd  = (var > 0.0f) ? sqrtf(var) : 0.0f;
                m->std[i] = (sd > m->min_std[i]) ? sd : m->min_std[i];
            }
            m->state       = (uint8_t)BASE_STATE_NORMAL;
            m->exit_consec = 0U;
        }

        out->state        = m->state;
        out->cal_progress = m->cal_count;
        out->cal_target   = m->cal_target;
        out->max_z        = 0.0f;
        out->fault_code   = 0U;
        return;
    }

    /* 阶段 2: 计算多维加权 z-score 异常得分 */
    for (i = 0U; i < BASELINE_FEAT_DIM; i++)
    {
        float sd = (m->std[i] > m->min_std[i]) ? m->std[i] : m->min_std[i];
        float z  = fabsf(feat[i] - m->mean[i]) / sd;
        float wz = z * m->weight[i];
        if (wz > max_z)
        {
            max_z = wz;
            dom   = (uint8_t)i;
        }
    }

    /* 阶段 3: 双阈值滞回防抖状态机 */
    if (m->state == (uint8_t)BASE_STATE_NORMAL)
    {
        if (max_z >= m->th_high)
        {
            m->state       = (uint8_t)BASE_STATE_FAULT;
            m->exit_consec = 0U;
        }
    }
    else if (m->state == (uint8_t)BASE_STATE_FAULT)
    {
        if (max_z <= m->th_low)
        {
            m->exit_consec++;
            if (m->exit_consec >= BASELINE_EXIT_CONSEC)
            {
                m->state       = (uint8_t)BASE_STATE_NORMAL;
                m->exit_consec = 0U;
            }
        }
        else
        {
            m->exit_consec = 0U;
        }
    }

    out->state         = m->state;
    out->dominant_feat = dom;
    out->cal_progress  = m->cal_count;
    out->cal_target    = m->cal_target;
    out->max_z         = max_z;

    if (m->state == (uint8_t)BASE_STATE_FAULT)
    {
        /* 低频宽带或整体 RMS 主导 -> 松动 (3); 步进谐波/冲击峭度主导 -> 过载失步 (2) */
        out->fault_code = (dom == (uint8_t)FEAT_LOW_BAND) ? 3U : 2U;
    }
    else
    {
        out->fault_code = 0U;
    }
}
