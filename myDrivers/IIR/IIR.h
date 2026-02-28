#ifndef __IIR_H__
#define __IIR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 配置最大通道数、最大节数 */
#ifndef IIR_MAX_CHANNELS
#define IIR_MAX_CHANNELS 2
#endif

#ifndef IIR_MAX_SECTIONS
#define IIR_MAX_SECTIONS 200   /* 必须 >= fdacoefs.h 的 MWSPT_NSEC */
#endif

/* 每节的状态结构（Direct Form II Transposed）*/
typedef struct {
    float w1;
    float w2;
} IIR_SectionState_t;

/* 初始化函数：自动从 MATLAB 生成的 fdacoefs.h 读取参数 */
int IIR_InitFromFdacoefs(uint16_t num_channels);

/* 自定义滤波器初始化（手动传入 sos 系数与 scalevalues）*/
int IIR_InitCustom(const float (*sos_in)[5], uint16_t num_sections,
                   const float *scalevalues, uint16_t num_channels);

/* 处理单个样本（每通道独立）*/
float IIR_ProcessSample(uint16_t channel, float input);

/* 批量处理（更高效）：输入/输出 指向长度为 blockSize 的缓冲区 */
void IIR_ProcessSamples(uint16_t channel, const float *input, float *output, uint32_t blockSize);

/* 重置单通道滤波器状态 */
void IIR_ResetChannel(uint16_t channel);

/* 重置所有滤波器状态 */
void IIR_ResetAll(void);

/* 获取当前通道数 */
uint16_t IIR_GetNumChannels(void);

/* ---------------- Diagnostic / Debug helpers ---------------- */

/* 初始化成 identity (直通) 的 SOS，用于验证 DSP 库与流程正确性。
   - num_sections: 要创建的节数（>0）
   - num_channels: 通道数
   返回 0 成功 */
int IIR_InitDebugIdentity(uint16_t num_sections, uint16_t num_channels);

/* 读取某通道某节的系数（stage: 0..num_sections-1；idx: 0..4 对应 b0,b1,b2,a1,a2） */
float IIR_GetCoeff(uint16_t channel, uint16_t stage, uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __IIR_H__ */
