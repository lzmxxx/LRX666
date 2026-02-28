#ifndef __CALCULATE_H__
#define __CALCULATE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化函数
void Calculate_Init(void);

// 每次新数据输入时更新计算
void Calculate_Update(int32_t ir_raw, int32_t red_raw, int32_t ir_filt, int32_t red_filt);

// 获取当前 R 值
float Calculate_GetR(void);
float Calculate_GetPI(void) ;
// （可选）根据经验公式估算 SpO2
float Calculate_GetSpO2(void);
float Calculate_GetHeartRate(void) ;
#ifdef __cplusplus
}
#endif

#endif
