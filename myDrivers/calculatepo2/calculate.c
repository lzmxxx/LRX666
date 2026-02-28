#include "arm_math.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "calculate.h"

/* ---- 基础配置 ---- */
#define CALC_WINDOW_SIZE    256     // 血氧计算的基础窗口 (256点)
#define FFT_DATA_GROUPS     4       // 收集多少组基础窗口后进行 FFT (4组)
#define TOTAL_DATA_LEN      (CALC_WINDOW_SIZE * FFT_DATA_GROUPS) // 总数据长度 (1024点)
#define FFT_SIZE            4096    // FFT 补零总点数
#define SAMPLING_RATE       125.0f  // 采样率 125Hz
#define PI                  3.14159265358979323846f

/* ---- 静态全局变量 (静态存储区) ---- */
// 扩大缓冲区以容纳 1024 点数据
static float32_t ir_raw_buf[TOTAL_DATA_LEN];
static float32_t red_raw_buf[TOTAL_DATA_LEN];
static float32_t ir_filt_buf[TOTAL_DATA_LEN];
static float32_t red_filt_buf[TOTAL_DATA_LEN];

// FFT 处理专用缓冲区
static float32_t fft_input_buf[FFT_SIZE];       
static float32_t fft_output_buf[FFT_SIZE];      
static float32_t fft_mag_buf[FFT_SIZE / 2];     
// 窗函数长度需要匹配总数据长度 (1024点)
static float32_t hann_window[TOTAL_DATA_LEN];  

static int buf_index = 0;           // 循环缓冲区当前写入索引
static int sample_count = 0;        // 已积累的样本总数

// CMSIS-DSP FFT 实例
static arm_rfft_fast_instance_f32 fft_inst;

/* ---- 全局输出变量 ---- */
float g_ir_dc = 0.0f, g_red_dc = 0.0f;
float g_ir_ac = 0.0f, g_red_ac = 0.0f;
static float R_value = 0.0f;
static float pi_val = 0.0f;

static float heart_rate = 0.0f;

/**
 * @brief 初始化计算模块
 */
void Calculate_Init(void)
{
    buf_index = 0;
    sample_count = 0;
    R_value = 0.0f;
    heart_rate = 0.0f;

    // 清空缓冲区
    memset(ir_raw_buf, 0, sizeof(ir_raw_buf));
    memset(red_raw_buf, 0, sizeof(red_raw_buf));
    memset(ir_filt_buf, 0, sizeof(ir_filt_buf));
    memset(red_filt_buf, 0, sizeof(red_filt_buf));

    // 预计算 1024 点的汉宁窗
    for (int i = 0; i < TOTAL_DATA_LEN; i++) {
        hann_window[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (TOTAL_DATA_LEN - 1)));
    }

    // 初始化 FFT 实例
    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
}

/**
 * @brief 内部函数：执行 FFT 计算心率
 * 处理 1024 点数据并补零至 4096 点
 */
static void Internal_Execute_FFT(void)
{
    // 1. 数据线性化：从循环缓冲区提取 1024 点并加窗
    for (int i = 0; i < TOTAL_DATA_LEN; i++) {
        // 计算时间顺序索引
        int idx = (buf_index + i) % TOTAL_DATA_LEN;
        fft_input_buf[i] = ir_filt_buf[idx] * hann_window[i];
    }

    // 2. 补零：清除从 1024 到 4096 的空间
    memset(&fft_input_buf[TOTAL_DATA_LEN], 0, sizeof(float32_t) * (FFT_SIZE - TOTAL_DATA_LEN));

    // 3. 执行实数 FFT
    arm_rfft_fast_f32(&fft_inst, fft_input_buf, fft_output_buf, 0);

    // 4. 计算幅值
    arm_cmplx_mag_f32(fft_output_buf, fft_mag_buf, FFT_SIZE / 2);

    // 5. 搜索心率峰值 [45 BPM ~ 220 BPM]
    int min_bin = (int)(0.75f * FFT_SIZE / SAMPLING_RATE); 
    int max_bin = (int)(5.0f * FFT_SIZE / SAMPLING_RATE);
    
    if (max_bin >= FFT_SIZE / 2) max_bin = (FFT_SIZE / 2) - 1;

    float32_t max_val;
    uint32_t max_index;
    
    arm_max_f32(&fft_mag_buf[min_bin], (max_bin - min_bin), &max_val, &max_index);
    max_index += min_bin;

    // 6. 换算为心率结果 (BPM)
    heart_rate = (float32_t)max_index * SAMPLING_RATE * 60.0f / (float32_t)FFT_SIZE;
}

/**
 * @brief 核心更新函数：每次采样后调用一次
 */
void Calculate_Update(int32_t ir_raw, int32_t red_raw, int32_t ir_filt, int32_t red_filt)
{
    // A. 存入 1024 点的长缓冲区
    ir_raw_buf[buf_index]  = (float32_t)ir_raw;
    red_raw_buf[buf_index] = (float32_t)red_raw;
    ir_filt_buf[buf_index] = (float32_t)ir_filt;
    red_filt_buf[buf_index] = (float32_t)red_filt;

    buf_index = (buf_index + 1) % TOTAL_DATA_LEN;
    if (sample_count < TOTAL_DATA_LEN) {
        sample_count++;
    }

    /* ---- 1. 血氧计算 (保持基于最近的 CALC_WINDOW_SIZE) ---- */
    // 只有当至少有了 256 点数据时才算血氧
    if (sample_count >= CALC_WINDOW_SIZE) {
        float32_t ir_sum = 0, red_sum = 0;
        float32_t ir_max_v = -1e9, ir_min_v = 1e9;
        float32_t red_max_v = -1e9, red_min_v = 1e9;

        // 仅对最近的 256 个点进行分析
        for (int i = 0; i < CALC_WINDOW_SIZE; i++) {
            // 反向回溯最近的 256 点
            int back_idx = (buf_index - 1 - i + TOTAL_DATA_LEN) % TOTAL_DATA_LEN;
            
            float32_t i_val = ir_raw_buf[back_idx];
            float32_t r_val = red_raw_buf[back_idx];
            float32_t i_filt = ir_filt_buf[back_idx];
            float32_t r_filt = red_filt_buf[back_idx];

            ir_sum += i_val;
            red_sum += r_val;

            if (i_filt > ir_max_v) ir_max_v = i_filt;
            if (i_filt < ir_min_v) ir_min_v = i_filt;
            if (r_filt > red_max_v) red_max_v = r_filt;
            if (r_filt < red_min_v) red_min_v = r_filt;
        }

        g_ir_dc = ir_sum / CALC_WINDOW_SIZE;
        g_red_dc = red_sum / CALC_WINDOW_SIZE;
        g_ir_ac = (ir_max_v - ir_min_v);
        g_red_ac = (red_max_v - red_min_v);

        if (g_ir_dc > 0 && g_red_dc > 0 && g_ir_ac > 0) {
            float32_t ratio_ir = g_ir_ac / g_ir_dc;
            float32_t ratio_red = g_red_ac / g_red_dc;
            R_value = ratio_red / ratio_ir;
						pi_val = ratio_ir;

        }
    }

    /* ---- 2. 心率计算触发逻辑 (每收集 4 组 256 点数据，即 1024 点) ---- */
    static uint16_t group_counter = 0;
    
    // 每过 256 个采样点，计数器加 1
    if (++group_counter >= CALC_WINDOW_SIZE) {
        group_counter = 0;
        
        static uint8_t group_set_idx = 0;
        group_set_idx++;
        
        // 当收集够 4 组（即 1024 点）时，触发 FFT
        if (group_set_idx >= FFT_DATA_GROUPS) {
            if (sample_count >= TOTAL_DATA_LEN) {
                Internal_Execute_FFT();
            }
            group_set_idx = 0; // 重置组计数
        }
    }
}

/* ---- 外部获取接口 ---- */
float Calculate_GetR(void) { return R_value; }
float Calculate_GetPI(void) { return pi_val; }

float Calculate_GetHeartRate(void) { return heart_rate; }
float Calculate_GetSpO2(void) {
    float32_t spo2 = -45.060f * R_value * R_value + 30.354f * R_value + 94.845f;
//    if (spo2 > 100.0f) spo2 = 100.0f;
//    if (spo2 < 70.0f)  spo2 = 70.0f;
    return (float)spo2;
}