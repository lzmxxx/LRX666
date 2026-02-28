#ifndef __AFE4490_AGC_H
#define __AFE4490_AGC_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

/* ================= 系统参数 ================= */
#define SAMPLE_RATE_HZ      125.0f
#define FFT_SIZE            256
#define AVG_WIN_SIZE        128     /* 物理量计算的滑动窗口 */
#define DC_WIN_SIZE         64      /* DC值计算的滑动窗口 */
#define CALC_WINDOW_SIZE    125     /* 2秒滑动窗口 (125Hz * 2s) */

/* ================= 阈值定义 ================= */
#define ADC_POS_FS          2097151
#define SATURATION_THRES    (ADC_POS_FS - 0)
#define STABILITY_WIN       30       /* 突变检测窗口大小 */
/* 吸光系数判断 (空气范围: 9000~11000) */
#define COEFF_AIR_MIN       9910.0f
#define COEFF_AIR_MAX       10010.0f
#define DC_SAT_COUNT_THRESHOLD  7

/* DC 调节 P-Control 参数 */
#define DC_TARGET_ADC       1572864 
#define DC_TOL_ADC          209715  
#define DC_KP_MA_PER_ADC    (1.0f / 280000.0f)
#define LED_CURR_STEP_MA    10.0f    
#define LED_CURR_MIN_MA     1.0f
#define LED_CURR_MAX_MA    90.0f

/* 辅助计算参数 */
#define AMBDAC_RATIO        2.0f     
#define AMBDAC_MAX_UA       2.0f    
#define HR_FREQ_MIN         0.5f
#define HR_FREQ_MAX         4.0f
#define SNR_MIN_DB          0.0f     

/* ================= 枚举定义 ================= */
typedef enum {
    AGC_STATE_STANDBY = 0,  
    AGC_STATE_FLASH,        
    AGC_STATE_TUNING,       
    AGC_STATE_LOCKED        
} AgcState_t;

typedef enum {
    tune_IDLE = 0,
    tune_WAIT_STABLE0,
    tune_LED_DC_ADJUST,     
    tune_AMBDAC_CALC,       
    tune_WAIT_STABLE,       
    tune_FFT_COLLECT,       
    tune_SNR_ADJUST,
    tune_WAIT_STABLE1   
} TuningSubState_t;

typedef enum {
    PGA_1X = 0, PGA_1_5X, PGA_2X, PGA_3X, PGA_4X
} PgaGain_t;

/* ================= 控制结构体 ================= */
typedef struct {
    AgcState_t state;
    TuningSubState_t sub_state;

    /* 硬件参数 */
    float led_ir_ma;
    float led_red_ma;
    uint8_t ir_pga;
    uint8_t red_pga;
    uint8_t ambdac_code;

    /* DC 计算相关 (实时更新) */
    int32_t dc_ir_raw_buf[DC_WIN_SIZE]; /* 原始数据缓冲 */
    int32_t dc_red_raw_buf[DC_WIN_SIZE];
    uint16_t dc_buf_idx;
    bool     dc_buf_filled;
    int64_t dc_sum_ir;
    int64_t dc_sum_red;
    uint16_t dc_count;
    int32_t dc_ir;   /* 实时平均 DC 值 */
    int32_t dc_red;
    uint16_t dc_adjust_timeout; 

    /* DSP 滑动平均 (用于物理系数计算) */
    float32_t ir_history[AVG_WIN_SIZE];
    float32_t red_history[AVG_WIN_SIZE];
    uint16_t  buf_idx;
    bool      buf_filled;

    /* 2秒窗口 (R值与审计计算) */
    float32_t ir_raw_buf[CALC_WINDOW_SIZE];
    float32_t red_raw_buf[CALC_WINDOW_SIZE];
    uint16_t  calc_idx;
    bool      calc_filled;

    float32_t g_ir_dc;
    float32_t g_red_dc;
    float32_t g_ir_ac;
    float32_t g_red_ac;
    float32_t R_value;
    float32_t pi_val;

    float32_t r_history[CALC_WINDOW_SIZE];
    uint16_t  r_idx;
    bool      r_filled;

    /* 系数相关 */
    float current_ln2;      
    float final_ir_avg;     
    float final_red_avg;
    
    /* 稳定性检测缓冲区 (5点) */
    float stab_buf[STABILITY_WIN];
    uint8_t stab_idx;
    uint8_t finger_off;
    bool    stab_filled;
    
    /* FFT 原始数据缓冲 */
    int32_t ir_fft_buf[FFT_SIZE];
    int32_t red_fft_buf[FFT_SIZE];
    uint16_t fft_idx;
    uint16_t fft_count;

    /* 结果存储 */
    float snr_ir;
    float snr_red;
    float corr_ir;
    float corr_red;
    float locked_snr_ir;
    float locked_snr_red;
    bool  locked;

    /* 计数器 */
    uint16_t flash_counter;
    uint16_t wait_counter;
    int8_t wait_flag;
    
} AFE4490_AGC;

extern AFE4490_AGC agc;

/* ================= API ================= */
void agc_init(void);

/* 更新传入参数：前两个为净光强(用于物理计算)，后两个为原始光强(用于饱和度判断) */
void agc_push_sample(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw);

/* 外部必须实现的底层驱动 */
extern void afe_adjust(uint8_t stage1, uint8_t ir_pga, uint8_t stage2, uint8_t red_pga, uint8_t ir_led, uint8_t red_led, uint8_t ambdac);

#endif