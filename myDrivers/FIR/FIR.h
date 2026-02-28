#ifndef __FIR_H__
#define __FIR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------- 可配置参数 -------------------- */
#ifndef FIR_MAX_CHANNELS
#define FIR_MAX_CHANNELS  2
#endif

#ifndef FIR_MAX_TAPS
#define FIR_MAX_TAPS      512
#endif

/* -------------------- 初始化接口 -------------------- */

/**
 * @brief 从 MATLAB 生成的 fdacoefs.h 初始化滤波器
 * @param num_channels 通道数（如 2 表示 IR + RED）
 * @return 0 成功；<0 表示错误
 */
int FIR_InitFromFdacoefs(uint16_t num_channels);

/**
 * @brief 自定义初始化滤波器（手动传入系数）
 * @param coeffs 滤波器系数数组
 * @param num_taps 系数数量（滤波器长度）
 * @param num_channels 通道数
 * @return 0 成功；<0 表示错误
 */
int FIR_InitCustom(const float *coeffs, uint16_t num_taps, uint16_t num_channels);

/* -------------------- 滤波处理接口 -------------------- */

/**
 * @brief 单样本处理（每通道独立）
 */
float FIR_ProcessSample(uint16_t channel, float input);

/**
 * @brief 批量处理输入数据
 */
void FIR_ProcessSamples(uint16_t channel, const float *input, float *output, uint32_t blockSize);

/**
 * @brief 重置指定通道状态
 */
void FIR_ResetChannel(uint16_t channel);

/**
 * @brief 重置所有通道状态
 */
void FIR_ResetAll(void);

/**
 * @brief 获取当前通道数
 */
uint16_t FIR_GetNumChannels(void);

/**
 * @brief 获取指定通道的第 i 个系数
 */
float FIR_GetCoeff(uint16_t channel, uint16_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __FIR_H__ */
