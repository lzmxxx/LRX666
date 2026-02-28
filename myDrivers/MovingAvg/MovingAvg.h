#ifndef DUALMOVINGAVG_H
#define DUALMOVINGAVG_H

// 定义窗口大小宏（方便全局修改）
#define WINDOW_SIZE 10
#define WINDOW_SIZE1 8
#define WINDOW_SIZE2 10

typedef struct {
    float buffer[WINDOW_SIZE];  // 数据缓冲区（由宏控制大小）
	   float buffer1[WINDOW_SIZE1];  // 数据缓冲区（由宏控制大小）
		   float buffer2[WINDOW_SIZE2];  // 数据缓冲区（由宏控制大小）

    int   index;                // 当前写入位置
    float sum;                  // 当前窗口总和
} ChannelFilter;

// 初始化双通道滤波器
void DualMovingAvg_Init(ChannelFilter *ir_filt, ChannelFilter *red_filt,ChannelFilter *SPO2_filt,ChannelFilter *HR_filt);
// 处理IR通道数据
float DualMovingAvg_ProcessIR(ChannelFilter *filt, float new_sample);

// 处理RED通道数据
float DualMovingAvg_ProcessRED(ChannelFilter *filt, float new_sample);

float DualMovingAvg_ProcessSPO2(ChannelFilter *filt, float new_sample);

float DualMovingAvg_ProcessHR(ChannelFilter *filt, float new_sample);

#endif // DUALMOVINGAVG_H