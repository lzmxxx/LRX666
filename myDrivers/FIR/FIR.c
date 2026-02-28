#include "fir.h"
#include <string.h>
#include <stdbool.h>
#include "arm_math.h"

/* -------------------- MATLAB fdacoefs 支持 -------------------- */
/* 直接强制包含 fdacoefs.h */
#include "FIR_SOS.h"

///* 校验 */
//#ifndef BL
//  #error "FIR_SOS.h 未定义 BL"
//#endif
//#ifndef B
//  #error "FIR_SOS.h 未定义 B[]"
//#endif

/* -------------------- 内部数据结构 -------------------- */
typedef struct {
    arm_fir_instance_f32 arm_inst;
    float coeffs[FIR_MAX_TAPS];
    float state[FIR_MAX_TAPS + 64]; // FIR 状态缓冲
    uint16_t num_taps;
} FIR_Filter_t;

static FIR_Filter_t fir_filters[FIR_MAX_CHANNELS];
static uint16_t current_num_channels = 0;

/* 可选调试输出函数 */
extern void Serial_Printf(const char *fmt, ...);

/* -------------------- 初始化函数 -------------------- */
static void fir_setup_channel(uint16_t ch, const float *coeffs, uint16_t num_taps)
{
    if (ch >= FIR_MAX_CHANNELS || num_taps > FIR_MAX_TAPS)
        return;

    memcpy(fir_filters[ch].coeffs, coeffs, num_taps * sizeof(float));
    memset(fir_filters[ch].state, 0, sizeof(fir_filters[ch].state));
    fir_filters[ch].num_taps = num_taps;

    arm_fir_init_f32(&fir_filters[ch].arm_inst, num_taps,
                     fir_filters[ch].coeffs, fir_filters[ch].state, 1);

    Serial_Printf("[FIR] Channel %u initialized, taps=%u\r\n", ch, num_taps);
}

/* -------------------- 从 fdacoefs.h 初始化 -------------------- */
int FIR_InitFromFdacoefs(uint16_t num_channels)
{
#ifdef HAVE_FDACOEFS_H
    if (num_channels == 0 || num_channels > FIR_MAX_CHANNELS)
        return -1;

    if (!(&B[0]))
        return -2;

    const uint16_t num_taps = (uint16_t)BL;
    for (uint16_t ch = 0; ch < num_channels; ch++)
        fir_setup_channel(ch, (const float *)B, num_taps);

    current_num_channels = num_channels;
    Serial_Printf("[FIR] Loaded from fdacoefs.h, taps=%d, channels=%d\r\n",
                  num_taps, num_channels);
    return 0;
#else
    (void)num_channels;
    return -3;
#endif
}

/* -------------------- 自定义初始化 -------------------- */
int FIR_InitCustom(const float *coeffs, uint16_t num_taps, uint16_t num_channels)
{
    if (!coeffs || num_taps == 0 || num_channels == 0)
        return -1;
    if (num_channels > FIR_MAX_CHANNELS || num_taps > FIR_MAX_TAPS)
        return -2;

    for (uint16_t ch = 0; ch < num_channels; ch++)
        fir_setup_channel(ch, coeffs, num_taps);

    current_num_channels = num_channels;
    return 0;
}

/* -------------------- 滤波处理 -------------------- */
float FIR_ProcessSample(uint16_t channel, float input)
{
    if (channel >= current_num_channels)
        return input;

    float out = 0.0f;
    arm_fir_f32(&fir_filters[channel].arm_inst, &input, &out, 1);
    return out;
}

void FIR_ProcessSamples(uint16_t channel, const float *input, float *output, uint32_t blockSize)
{
    if (channel >= current_num_channels || !input || !output || blockSize == 0)
        return;

    arm_fir_f32(&fir_filters[channel].arm_inst, (float32_t *)input, (float32_t *)output, blockSize);
}

/* -------------------- 状态控制 -------------------- */
void FIR_ResetChannel(uint16_t channel)
{
    if (channel >= current_num_channels)
        return;
    memset(fir_filters[channel].state, 0, sizeof(fir_filters[channel].state));
    arm_fir_init_f32(&fir_filters[channel].arm_inst,
                     fir_filters[channel].num_taps,
                     fir_filters[channel].coeffs,
                     fir_filters[channel].state,
                     1);
}

void FIR_ResetAll(void)
{
    for (uint16_t ch = 0; ch < current_num_channels; ch++)
        FIR_ResetChannel(ch);
}

uint16_t FIR_GetNumChannels(void)
{
    return current_num_channels;
}

float FIR_GetCoeff(uint16_t channel, uint16_t idx)
{
    if (channel >= current_num_channels)
        return 0.0f;
    if (idx >= fir_filters[channel].num_taps)
        return 0.0f;
    return fir_filters[channel].coeffs[idx];
}
