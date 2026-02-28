#include "MovingAvg.h"
#include <string.h>

void DualMovingAvg_Init(ChannelFilter *ir_filt, ChannelFilter *red_filt,ChannelFilter *SPO2_filt,ChannelFilter *HR_filt) {
    // 初始化IR滤波器
    memset(ir_filt->buffer, 0, sizeof(float) * WINDOW_SIZE);
    ir_filt->index = 0;
    ir_filt->sum = 0.0f;
    
    // 初始化RED滤波器
    memset(red_filt->buffer, 0, sizeof(float) * WINDOW_SIZE);
    red_filt->index = 0;
    red_filt->sum = 0.0f;
	
	  memset(SPO2_filt->buffer1, 0, sizeof(float) * WINDOW_SIZE1);
    SPO2_filt->index = 0;
    SPO2_filt->sum = 0.0f;
	
		  memset(HR_filt->buffer2, 0, sizeof(float) * WINDOW_SIZE2);
    SPO2_filt->index = 0;
    SPO2_filt->sum = 0.0f;
}

float DualMovingAvg_ProcessIR(ChannelFilter *filt, float new_sample) {
    filt->sum -= filt->buffer[filt->index];
    filt->buffer[filt->index] = new_sample;
    filt->sum += new_sample;
    filt->index = (filt->index + 1) % WINDOW_SIZE;  // 使用宏定义窗口大小
    return filt->sum / WINDOW_SIZE;                 // 使用宏定义分母
}

float DualMovingAvg_ProcessRED(ChannelFilter *filt, float new_sample) {
    filt->sum -= filt->buffer[filt->index];
    filt->buffer[filt->index] = new_sample;
    filt->sum += new_sample;
    filt->index = (filt->index + 1) % WINDOW_SIZE;
    return filt->sum / WINDOW_SIZE;
}

float DualMovingAvg_ProcessSPO2(ChannelFilter *filt, float new_sample) {
    filt->sum -= filt->buffer1[filt->index];
    filt->buffer1[filt->index] = new_sample;
    filt->sum += new_sample;
    filt->index = (filt->index + 1) % WINDOW_SIZE1;
    return filt->sum / WINDOW_SIZE1;
}

float DualMovingAvg_ProcessHR(ChannelFilter *filt, float new_sample) {
    filt->sum -= filt->buffer2[filt->index];
    filt->buffer2[filt->index] = new_sample;
    filt->sum += new_sample;
    filt->index = (filt->index + 1) % WINDOW_SIZE2;
    return filt->sum / WINDOW_SIZE2;
}