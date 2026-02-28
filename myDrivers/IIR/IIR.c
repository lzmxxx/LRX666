#include "iir.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* -------------------- CMSIS DSP -------------------- */
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#include "arm_math.h"

/* -------------------- MATLAB fdacoefs 支持 -------------------- */
#ifdef HAVE_FDACOEFS_H

#if defined(__has_include)
  #if __has_include("rtwtypes.h")
    #include "rtwtypes.h"
  #elif __has_include("tmwtypes.h")
    #include "tmwtypes.h"
  #endif
#endif

#if !defined(RTWTYPES_H) && !defined(TMWTYPES_H)
typedef int8_t    int8_T;
typedef uint8_t   uint8_T;
typedef int16_t   int16_T;
typedef uint16_t  uint16_T;
typedef int32_t   int32_T;
typedef uint32_t  uint32_T;
typedef float     real32_T;
typedef double    real_T;
typedef unsigned char boolean_T;
#endif

#include "fdacoefs.h"

#ifndef NUM_SECTIONS
  #ifdef MWSPT_NSEC
    #define NUM_SECTIONS MWSPT_NSEC
  #else
    #error "fdacoefs.h 未定义 NUM_SECTIONS 或 MWSPT_NSEC"
  #endif
#endif

#undef IIR_MAX_SECTIONS
#define IIR_MAX_SECTIONS NUM_SECTIONS

#endif /* HAVE_FDACOEFS_H */

/* -------------------- 内部结构 -------------------- */
typedef struct {
    const float (*sos)[5];
    const float *scalevalues;
    uint16_t num_sections;
    float *pState;
} IIR_FilterInstance_t;

static arm_biquad_cascade_df2T_instance_f32 arm_filters[IIR_MAX_CHANNELS];
static float filter_states[IIR_MAX_CHANNELS][4 * IIR_MAX_SECTIONS];
static IIR_FilterInstance_t filters[IIR_MAX_CHANNELS];
static uint16_t current_num_channels = 0;
static uint16_t current_num_sections = 0;

/* 供外部 printf 调试 */
extern void Serial_Printf(const char *fmt, ...);

/* -------------------- 辅助函数 -------------------- */
static void iir_setup_channel_from_sos(uint16_t ch,
                                       const float (*sos_ptr)[5],
                                       const float *scalevalues,
                                       uint16_t num_sections)
{
    filters[ch].sos = sos_ptr;
    filters[ch].scalevalues = scalevalues;
    filters[ch].num_sections = num_sections;
    filters[ch].pState = &filter_states[ch][0];
    memset(filters[ch].pState, 0, sizeof(float) * 4 * num_sections);

    arm_biquad_cascade_df2T_init_f32(&arm_filters[ch],
                                     (uint8_t)num_sections,
                                     (float32_t *)&sos_ptr[0][0],
                                     (float32_t *)filters[ch].pState);
}

/* -------------------- 初始化（从 fdacoefs.h） -------------------- */
int IIR_InitFromFdacoefs(uint16_t num_channels)
{
#ifdef HAVE_FDACOEFS_H
    if (num_channels == 0 || num_channels > IIR_MAX_CHANNELS) return -1;

    #ifdef MWSPT_NSEC
        extern const real32_T NUM[MWSPT_NSEC][3];
        extern const real32_T DEN[MWSPT_NSEC][3];

        static float local_sos[NUM_SECTIONS][5];
        for (uint16_t i = 0; i < NUM_SECTIONS; i++) {
            float b0 = (float)NUM[i][0];
            float b1 = (float)NUM[i][1];
            float b2 = (float)NUM[i][2];
            float a0 = (float)DEN[i][0];
            float a1 = (float)DEN[i][1];
            float a2 = (float)DEN[i][2];

            /* 归一化 a0 */
            if (a0 != 0.0f && a0 != 1.0f) {
                b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
            }

            /* CMSIS 需要负号 */
            local_sos[i][0] = b0;
            local_sos[i][1] = b1;
            local_sos[i][2] = b2;
            local_sos[i][3] = -a1;
            local_sos[i][4] = -a2;

            Serial_Printf("[SOS %02d] b0=%f b1=%f b2=%f a1=%f a2=%f\r\n",
                          i, local_sos[i][0], local_sos[i][1],
                          local_sos[i][2], local_sos[i][3],
                          local_sos[i][4]);
        }

    for (uint16_t ch = 0; ch < num_channels; ch++) {
        const float *scale_ptr = NULL;
    #ifdef scalevalues
        scale_ptr = scalevalues;
    #endif

        iir_setup_channel_from_sos(ch,
                                   (const float (*)[5])local_sos,
                                   scale_ptr,
                                   NUM_SECTIONS);
    }


    #else
        extern const float sos[][5];
        static float local_sos[NUM_SECTIONS][5];

        /* MATLAB sos: b0 b1 b2 a0 a1 a2 → CMSIS: b0 b1 b2 -a1 -a2 */
        for (uint16_t i = 0; i < NUM_SECTIONS; i++) {
            float b0 = sos[i][0];
            float b1 = sos[i][1];
            float b2 = sos[i][2];
            float a0 = sos[i][3];
            float a1 = sos[i][4];
            float a2 = sos[i][5];
            if (a0 != 0.0f && a0 != 1.0f) {
                b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
            }
            local_sos[i][0] = b0;
            local_sos[i][1] = b1;
            local_sos[i][2] = b2;
            local_sos[i][3] = -a1;
            local_sos[i][4] = -a2;

            Serial_Printf("[SOS %02d] b0=%f b1=%f b2=%f a1=%f a2=%f\r\n",
                          i, local_sos[i][0], local_sos[i][1],
                          local_sos[i][2], local_sos[i][3],
                          local_sos[i][4]);
        }

        for (uint16_t ch = 0; ch < num_channels; ch++) {
            iir_setup_channel_from_sos(ch,
                                       (const float (*)[5])local_sos,
                                       NULL,
                                       NUM_SECTIONS);
        }
    #endif

    current_num_channels = num_channels;
    current_num_sections = NUM_SECTIONS;
    return 0;

#else
    (void)num_channels;
    return -2;
#endif
}

/* -------------------- 自定义初始化 -------------------- */
int IIR_InitCustom(const float (*sos_in)[5],
                   uint16_t num_sections,
                   const float *scalevalues,
                   uint16_t num_channels)
{
    if (!sos_in) return -1;
    if (num_sections == 0 || num_sections > IIR_MAX_SECTIONS) return -2;
    if (num_channels == 0 || num_channels > IIR_MAX_CHANNELS) return -3;

    for (uint16_t ch = 0; ch < num_channels; ch++) {
        iir_setup_channel_from_sos(ch, sos_in, scalevalues, num_sections);
    }
    current_num_channels = num_channels;
    current_num_sections = num_sections;
    return 0;
}

/* -------------------- 滤波函数 -------------------- */
float IIR_ProcessSample(uint16_t channel, float input)
{
    if (channel >= current_num_channels) return input;
    IIR_FilterInstance_t *inst = &filters[channel];
    if (!inst->sos || inst->num_sections == 0) return input;

    float out = 0.0f;
    arm_biquad_cascade_df2T_f32(&arm_filters[channel], &input, &out, 1);

    if (inst->scalevalues) {
        float scl = inst->scalevalues[inst->num_sections];
        if (scl != 0.0f) out *= scl;
    }

    return out;
}

/* -------------------- 批量处理 -------------------- */
void IIR_ProcessSamples(uint16_t channel,
                        const float *input,
                        float *output,
                        uint32_t blockSize)
{
    if (channel >= current_num_channels || !input || !output || blockSize == 0) return;
    IIR_FilterInstance_t *inst = &filters[channel];
    if (!inst->sos || inst->num_sections == 0) {
        memcpy(output, input, blockSize * sizeof(float));
        return;
    }

    arm_biquad_cascade_df2T_f32(&arm_filters[channel],
                                (float32_t *)input,
                                (float32_t *)output,
                                blockSize);

    if (inst->scalevalues) {
        float scl = inst->scalevalues[inst->num_sections];
        if (scl != 0.0f) {
            for (uint32_t i = 0; i < blockSize; i++) output[i] *= scl;
        }
    }
}

/* -------------------- 重置 -------------------- */
void IIR_ResetChannel(uint16_t ch)
{
    if (ch >= current_num_channels) return;
    IIR_FilterInstance_t *inst = &filters[ch];
    if (!inst->pState) return;
    memset(inst->pState, 0, sizeof(float) * 4 * inst->num_sections);
    arm_biquad_cascade_df2T_init_f32(&arm_filters[ch],
                                     (uint8_t)inst->num_sections,
                                     (float32_t *)&inst->sos[0][0],
                                     (float32_t *)inst->pState);
}

void IIR_ResetAll(void)
{
    for (uint16_t ch = 0; ch < current_num_channels; ch++)
        IIR_ResetChannel(ch);
}

uint16_t IIR_GetNumChannels(void)
{
    return current_num_channels;
}

/* -------------------- 系数读取 -------------------- */
float IIR_GetCoeff(uint16_t channel, uint16_t stage, uint8_t idx)
{
    if (channel >= current_num_channels) return 0.0f;
    if (stage >= filters[channel].num_sections) return 0.0f;
    if (idx > 4) return 0.0f;
    return filters[channel].sos[stage][idx];
}
