#include "AFE4490_AGC.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 定义 PI */
#ifndef PI
#define PI 3.14159265358979f
#endif

/* 新增 R 值短期连续判断窗口大小 (64点 ≈ 0.5秒) */
#define R_CHECK_WIN 64

AFE4490_AGC agc;

/* ================= DSP 静态资源 ================= */
static arm_rfft_fast_instance_f32 fft_inst;
static float fft_in[FFT_SIZE];
static float fft_out[FFT_SIZE];
static float fft_mag[FFT_SIZE / 2];
static float hann_win[FFT_SIZE];

/* 内存复用优化：nsdf_buf 与 fft_in 共用内存 */
/* 因为 calc_snr 和 calc_corr_nsdf 不会同时运行，且每次运行都会重新赋值 */
#define nsdf_buf fft_in

/* ================= 全局增益缓存 ================= */
float g_pga_ir_val = 1.0f;
float g_pga_red_val = 1.0f;
float R_pga_ir_val = 0.1f; 
float R_pga_red_val = 0.1f;
float r_fluctuation;
uint8_t dc_count = 0;

/* ================= 内部函数声明 ================= */
static uint8_t calc_ambdac(int32_t dc_adc);
static float calc_snr(int32_t *buf, int32_t dc);
static float calc_corr_nsdf(int32_t *buf, int32_t dc);
static uint8_t led_ma_to_code(float ma);
static void init_hann_window(void);

static void Calculate_Physics_And_Coeff(int32_t ir_net, int32_t red_net);

static void State_Standby_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw);
static void State_Flash_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw);
static void State_Tuning_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw);
static void State_Locked_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw);

static void Reset_To_Standby(void);
static void Update_Hardware_Full(void);
static void Update_Gain_Params(void);

/* ================= 初始化 ================= */
void agc_init(void)
{
    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    init_hann_window();
    Reset_To_Standby();
}

static void init_hann_window(void)
{
    for (int i = 0; i < FFT_SIZE; i++)
    {
        hann_win[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
    }
}

static void Update_Hardware_Full(void)
{
    afe_adjust(1, agc.ir_pga, 1, agc.red_pga,
               led_ma_to_code(agc.led_ir_ma),
               led_ma_to_code(agc.led_red_ma),
               agc.ambdac_code);

    Update_Gain_Params();
}

static void Reset_To_Standby(void)
{
    agc.state = AGC_STATE_STANDBY;
    agc.sub_state = tune_IDLE;

    /* 3mA 常亮待机配置 */
    agc.led_ir_ma = 3.0f;
    agc.led_red_ma = 3.0f;
    agc.ir_pga = PGA_1X;
    agc.red_pga = PGA_1X;
    agc.ambdac_code = 1;

    /* 清空计算缓存 */
    memset(agc.ir_history, 0, sizeof(agc.ir_history));
    memset(agc.red_history, 0, sizeof(agc.red_history));
    agc.buf_idx = 0;
    agc.buf_filled = false;
    
    memset(agc.dc_ir_raw_buf, 0, sizeof(agc.dc_ir_raw_buf));
    memset(agc.dc_red_raw_buf, 0, sizeof(agc.dc_red_raw_buf));
    agc.dc_buf_idx = 0;
    agc.dc_buf_filled = false;
    agc.dc_ir = 0;
    agc.dc_red = 0;

    /* 清空稳定性检测缓冲 */
    memset(agc.stab_buf, 0, sizeof(agc.stab_buf));
    agc.stab_idx = 0;
    agc.stab_filled = false;
        
    /* 清空新增计算缓存 */
    memset(agc.ir_raw_buf, 0, sizeof(agc.ir_raw_buf));
    memset(agc.red_raw_buf, 0, sizeof(agc.red_raw_buf));
    agc.calc_idx = 0;
    agc.calc_filled = false;
    
    memset(agc.r_history, 0, sizeof(agc.r_history));
    agc.r_idx = 0;
    agc.r_filled = false;

    Update_Hardware_Full();
}

static void Update_Gain_Params(void)
{
    switch(agc.ir_pga)
    {
    case PGA_1X:   g_pga_ir_val = 1.0f; R_pga_ir_val=0.1f; break;
    case PGA_1_5X: g_pga_ir_val = 1.5f; R_pga_ir_val=0.15f; break;
    case PGA_2X:   g_pga_ir_val = 2.0f; R_pga_ir_val=0.2f; break;
    case PGA_3X:   g_pga_ir_val = 3.0f; R_pga_ir_val=0.3f; break;
    case PGA_4X:   g_pga_ir_val = 4.0f; R_pga_ir_val=0.4f; break;
    default:       g_pga_ir_val = 1.0f; R_pga_ir_val=0.1f; break;
    }
    switch(agc.red_pga)
    {
    case PGA_1X:   g_pga_red_val = 1.0f; R_pga_red_val=0.1f; break;
    case PGA_1_5X: g_pga_red_val = 1.5f; R_pga_red_val=0.15f; break;
    case PGA_2X:   g_pga_red_val = 2.0f; R_pga_red_val=0.2f; break;
    case PGA_3X:   g_pga_red_val = 3.0f; R_pga_red_val=0.3f; break;
    case PGA_4X:   g_pga_red_val = 4.0f; R_pga_red_val=0.4f; break;
    default:       g_pga_red_val = 1.0f; R_pga_red_val=0.1f; break;
    }
}

/* ================= 核心逻辑：采样点驱动 ================= */
void agc_push_sample(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw)
{
    /* 1. 独立 R 值计算更新与 2s 滑动窗口数据填充 (严格使用净光强) */
    agc.ir_raw_buf[agc.calc_idx] = (float32_t)ir_net;
    agc.red_raw_buf[agc.calc_idx] = (float32_t)red_net;
    
    agc.calc_idx++;
    if (agc.calc_idx >= CALC_WINDOW_SIZE) {
        agc.calc_idx = 0;
        agc.calc_filled = true;
    }

    if (agc.calc_filled) {
        float32_t ir_sum = 0, red_sum = 0;
        float32_t ir_max_v = -1e9, ir_min_v = 1e9;
        float32_t red_max_v = -1e9, red_min_v = 1e9;

        for (int i = 0; i < CALC_WINDOW_SIZE; i++) {
            float32_t i_val = agc.ir_raw_buf[i];
            float32_t r_val = agc.red_raw_buf[i];

            ir_sum += i_val;
            red_sum += r_val;

            if (i_val > ir_max_v) ir_max_v = i_val;
            if (i_val < ir_min_v) ir_min_v = i_val;
            if (r_val > red_max_v) red_max_v = r_val;
            if (r_val < red_min_v) red_min_v = r_val;
        }

        agc.g_ir_dc = ir_sum / CALC_WINDOW_SIZE;
        agc.g_red_dc = red_sum / CALC_WINDOW_SIZE;
        agc.g_ir_ac = (ir_max_v - ir_min_v);
        agc.g_red_ac = (red_max_v - red_min_v);

        if (agc.g_ir_dc > 0 && agc.g_red_dc > 0 && agc.g_ir_ac > 0) {
            float32_t ratio_ir = agc.g_ir_ac / agc.g_ir_dc;
            float32_t ratio_red = agc.g_red_ac / agc.g_red_dc;
            agc.R_value = ratio_red / ratio_ir;
            agc.pi_val = ratio_ir;
        }

        /* 记录 R 值的历史用于稳定性审计 (修改：缩短历史窗大小，加速通过) */
        agc.r_history[agc.r_idx] = agc.R_value;
        agc.r_idx++;
        if (agc.r_idx >= R_CHECK_WIN) {
            agc.r_idx = 0;
            agc.r_filled = true;
        }
    }

    /* 2. 原有物理量与DC计算 (严格使用净光强) */
    Calculate_Physics_And_Coeff(ir_net, red_net); 
    
    /* 3. 顶层状态机 */
    switch (agc.state)
    {
    case AGC_STATE_STANDBY:
        State_Standby_Handler(ir_net, red_net, ir_raw, red_raw);
        break;
    case AGC_STATE_FLASH:
        State_Flash_Handler(ir_net, red_net, ir_raw, red_raw);
        break;
    case AGC_STATE_TUNING:
        State_Tuning_Handler(ir_net, red_net, ir_raw, red_raw);
        break;
    case AGC_STATE_LOCKED:
        State_Locked_Handler(ir_net, red_net, ir_raw, red_raw);
        break;
    }
}

/* ================= 物理量与系数计算 ================= */
static void Calculate_Physics_And_Coeff(int32_t ir_net, int32_t red_net)
{
    Update_Gain_Params();

    /* 1. 还原真实 ADC 值 */
    int32_t ir_true  = (int32_t)ir_net; 
    int32_t red_true = (int32_t)red_net; 

    /* 2. 转换为纳安 (nA) */
    float Iir_true_nA  = ((float)ir_true * 0.000000572f) / (2.0f * g_pga_ir_val * 250000.0f) * 1e9f;
    float Ired_true_nA = ((float)red_true * 0.000000572f) / (2.0f * g_pga_red_val * 250000.0f) * 1e9f;

/* ============================================================
 * 1. 滑动平均计算 (使用 DSP 库，修复局部变量清零导致的负数 BUG)
 * ============================================================ */

/* 更新缓冲区 */
agc.ir_history[agc.buf_idx] = Iir_true_nA;
agc.red_history[agc.buf_idx] = Ired_true_nA;

agc.buf_idx++;
if (agc.buf_idx >= AVG_WIN_SIZE) {
    agc.buf_idx = 0;
    agc.buf_filled = true;
}

/* 计算有效长度 */
uint32_t len = agc.buf_filled ? AVG_WIN_SIZE : agc.buf_idx;
if (len == 0) len = 1;

/* 使用 arm_mean_f32 替代手写累加，避免负数和漂移 */
arm_mean_f32(agc.ir_history, len, &agc.final_ir_avg);
arm_mean_f32(agc.red_history, len, &agc.final_red_avg);


/* ============================================================
 * 2. 吸光系数计算 (增加 NaN 防护)
 * ============================================================ */
float a = 4;
float b = 4;
// 防止除以0 (电流不应为0)
if (a < 0.1f) a = 0.1f; 
if (b < 0.1f) b = 0.1f;

// 归一化计算
float Ir1 = agc.final_ir_avg / (a * 1000000.0f);
float RED1 = agc.final_red_avg / (b * 1000000.0f);

/* --- 关键防护：防止 log 输入为负或零 --- */
// 物理上光强不可能为负，设定一个极小正数作为下限
const float MIN_VAL = 1e-6f; 
if (Ir1 < MIN_VAL) Ir1 = MIN_VAL;
if (RED1 < MIN_VAL) RED1 = MIN_VAL;

/* 计算对数 */
float ln_ir, ln_red;
arm_vlog_f32(&Ir1, &ln_ir, 1);
arm_vlog_f32(&RED1, &ln_red, 1);

/* --- 关键防护：防止分母为零 --- */
// 如果 ln_ir 太接近 0 (即 Ir1 接近 1.0)，除法会爆
if (fabsf(ln_ir) < 1e-6f) {
    // 根据符号给予一个极小值，或者直接处理为 0
    ln_ir = (ln_ir >= 0) ? 1e-6f : -1e-6f;
}

/* 最终结果 */
agc.current_ln2 = (ln_red / ln_ir) * 10000.0f;

/* ============================================================
 * 3. DC计算 (严格使用净光强)
 * ============================================================ */
    agc.dc_sum_ir  += ir_net;
    agc.dc_sum_red += red_net;
    agc.dc_count++;

    if (agc.dc_count >= DC_WIN_SIZE) {
        agc.dc_ir  = (int32_t)(agc.dc_sum_ir  / DC_WIN_SIZE);
        agc.dc_red = (int32_t)(agc.dc_sum_red / DC_WIN_SIZE);
        agc.dc_sum_ir = 0; agc.dc_sum_red = 0;  
    }
}

/* ================= 状态机逻辑 ================= */

static void State_Standby_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw)
{
        agc.wait_counter=0;
        /* 初始化参数 */
        agc.ir_pga = PGA_1X;
        agc.red_pga = PGA_1X;
        agc.ambdac_code = 0;
        agc.led_ir_ma = 3.0f;
        agc.led_red_ma = 3.0f;
        agc.dc_adjust_timeout = 0;
        Update_Hardware_Full();
    /* 使用原始光强进行饱和判断 */
    bool ir_sat = (ir_raw >= SATURATION_THRES);
    bool red_sat = (red_raw >= SATURATION_THRES);

    /* 一、 全饱和 (Full Saturation) -> 进 Flash 模式 */
    if (ir_sat && red_sat)
    {
        agc.finger_off = true;
        agc.state = AGC_STATE_FLASH;
        agc.flash_counter = 0;
        agc.ir_pga = PGA_1X;
        agc.red_pga = PGA_1X;
        agc.ambdac_code = 0;

        afe_adjust(1, agc.ir_pga, 1, agc.red_pga, 0, 0, agc.ambdac_code);
        return;
    }

    /* 二、 半饱和 (Half Saturation) -> 异常佩戴，重置脱指 */
    if (ir_sat || red_sat) {         
        agc.finger_off = true;
        Reset_To_Standby(); /* 饱和表明数据无效，必须重置 */
        return;
    }

    /* 三、 不饱和 (Non-Saturation) -> 信号连续滑动审计门槛 */
    if (agc.calc_filled && agc.r_filled)
    {
        /* 审计条件1：净光强不足 */
        if (agc.g_ir_dc < 3000.0f || agc.g_red_dc < 3000.0f) {
            agc.finger_off = true;
            /* 关键修改：取消 Reset_To_Standby，保持传送带滑动，拒绝固定窗口的漫长等待 */
            return;
        }

        /* 审计条件2：短期连续滑动 R 值稳定性检测 */
        float max_r = -1e9f, min_r = 1e9f;
        for (int i = 0; i < R_CHECK_WIN; i++) {
            if (agc.r_history[i] > max_r) max_r = agc.r_history[i];
            if (agc.r_history[i] < min_r) min_r = agc.r_history[i];
        }

        float avg_r = (max_r + min_r) / 2.0f;
         r_fluctuation = (avg_r > 0) ? ((max_r - min_r) / avg_r) : 1.0f;

        /* R值不稳，上下浮动 > 30% -> 直接 return 让下一次循环继续判断 */
        if (r_fluctuation > 0.10f) {
            agc.finger_off = true;
            /* 关键修改：取消 Reset_To_Standby，允许每一帧数据实时滑动审查 */
            return;
        }

        /* 通过严苛连续滑动准入门槛，瞬间启动 AGC */
        agc.finger_off = false;
        agc.sub_state = tune_WAIT_STABLE0;
        agc.state = AGC_STATE_TUNING;

    }
}

static void State_Flash_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw)
{
    if (agc.flash_counter == 0)
    {
        /* 第0点: 开灯 (3mA) */
        afe_adjust(1, PGA_1X, 1, PGA_1X, led_ma_to_code(4.0f), led_ma_to_code(4.0f), 1);
    }
    else if (agc.flash_counter == 4)
    {
        /* 第4点: 采样判定 (使用原始光强) */
        bool sat = (ir_raw >= SATURATION_THRES) && (red_raw >= SATURATION_THRES);
        afe_adjust(1, PGA_1X, 1, PGA_1X, 0, 0, 1); /* 立即关灯 */

        if (!sat)
        {
            Reset_To_Standby();
            return;
        }
    }

    agc.flash_counter++;
    if (agc.flash_counter >= (uint16_t)SAMPLE_RATE_HZ)
    {
        agc.flash_counter = 0;
    }
}

static void State_Tuning_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw)
{
    /* 保护: 调节过程饱和直接复位 */
    /* 可选: 暂时注释掉，避免调节初期波动导致误判 */
    // if (ir_raw >= SATURATION_THRES || red_raw >= SATURATION_THRES) { Reset_To_Standby(); return; }

    /* 注意：这里不再需要手动累加 DC，因为 Calculate_DC_Average 已经自动更新了 agc.dc_ir */
if(agc.wait_flag ==1) agc.wait_counter++;
    switch (agc.sub_state)
    {
        case tune_WAIT_STABLE0:
    {
            agc.wait_flag = 1;
        if (agc.wait_counter > 64)
        {
                  agc.wait_flag = 0;
                    agc.wait_counter=0;
            agc.sub_state = tune_LED_DC_ADJUST;
        }
        break;
    }
    /* 1. DC 调节 (P控制) */
    case tune_LED_DC_ADJUST:
    {
            
            /* 阶段 2: 数据统计与脱落实时监控 */
   if(agc.dc_count>=DC_WIN_SIZE){
                    agc.dc_count = 0;
        /* 直接使用实时更新的 DC 平均值 */
            int32_t err_ir  = DC_TARGET_ADC - agc.dc_ir;
            int32_t err_red = DC_TARGET_ADC - agc.dc_red;
            float d_ir  = fminf(fmaxf(DC_KP_MA_PER_ADC * (float)err_ir, -LED_CURR_STEP_MA), LED_CURR_STEP_MA);
            float d_red = fminf(fmaxf(DC_KP_MA_PER_ADC * (float)err_red, -LED_CURR_STEP_MA), LED_CURR_STEP_MA);

            agc.led_ir_ma  = fminf(fmaxf(agc.led_ir_ma + d_ir, LED_CURR_MIN_MA), LED_CURR_MAX_MA);
            agc.led_red_ma = fminf(fmaxf(agc.led_red_ma + d_red, LED_CURR_MIN_MA), LED_CURR_MAX_MA);
        /* 限幅绝对值 */


        Update_Hardware_Full();
                dc_count++;
                if(dc_count>=DC_SAT_COUNT_THRESHOLD) {agc.sub_state = tune_AMBDAC_CALC;dc_count=0;}
        bool converged = (abs(err_ir) < DC_TOL_ADC) && (abs(err_red) < DC_TOL_ADC);
        bool railed = (agc.led_ir_ma >= LED_CURR_MAX_MA) || (agc.led_red_ma >= LED_CURR_MAX_MA);


        if (converged||railed )
        {
            agc.sub_state = tune_AMBDAC_CALC;
        }
            }
        break;
            
    }
    /* 2. AmbDAC 计算 */
    case tune_AMBDAC_CALC:
    {
        /* 使用稳定的 DC 平均值计算 */
        agc.ambdac_code = calc_ambdac((agc.dc_ir + agc.dc_red) / 2);
        Update_Hardware_Full();

        agc.wait_counter = 0;
        agc.sub_state = tune_WAIT_STABLE;
        break;
    }

    /* 3. 等待硬件稳定 */
    case tune_WAIT_STABLE:
    {
                        agc.wait_flag = 1;

        if (agc.wait_counter > 32)
        {
                  agc.wait_flag = 0;
                    agc.wait_counter=0;
            agc.sub_state = tune_FFT_COLLECT;
        }
        break;
    }

    /* 4. FFT 数据采集 (使用净光强) */
    case tune_FFT_COLLECT:
    {
        agc.ir_fft_buf[agc.fft_idx]  = ir_net;
        agc.red_fft_buf[agc.fft_idx] = red_net;
        agc.fft_idx++;
        if (agc.fft_idx >= FFT_SIZE) agc.fft_idx = 0;

        agc.fft_count++;
        if (agc.fft_count >= FFT_SIZE)
        {
            agc.sub_state = tune_SNR_ADJUST;
                     agc.fft_count=0;
        }
        break;
    }

    /* 5. SNR 计算与决策 */
    case tune_SNR_ADJUST:
    {
        agc.snr_ir  = calc_snr(agc.ir_fft_buf,  (int32_t)DC_TARGET_ADC);
        agc.snr_red = calc_snr(agc.red_fft_buf, (int32_t)DC_TARGET_ADC);
        agc.corr_ir  = calc_corr_nsdf(agc.ir_fft_buf,  (int32_t)DC_TARGET_ADC);
        agc.corr_red = calc_corr_nsdf(agc.red_fft_buf, (int32_t)DC_TARGET_ADC);

        if (agc.snr_ir < SNR_MIN_DB || agc.snr_red < SNR_MIN_DB)
        {
                      agc.ir_pga = PGA_4X;
                        agc.red_pga = PGA_4X;
                      agc.locked = true;
            Update_Hardware_Full();
            agc.sub_state = tune_WAIT_STABLE1;
            agc.locked_snr_ir = agc.snr_ir;
            agc.locked_snr_red = agc.snr_red;
                      agc.buf_idx = 0;
            agc.buf_filled = false;
                    }
            else
            {
                /* 强制锁定 */
                agc.locked = true;
                agc.sub_state = tune_WAIT_STABLE1;
                agc.locked_snr_ir = agc.snr_ir;
                agc.locked_snr_red = agc.snr_red;
                
                agc.buf_idx = 0; 
                agc.buf_filled = false;
            }
        

        break;
    }
            /* 6. 等待硬件稳定 */
    case tune_WAIT_STABLE1:
    {
            agc.wait_flag = 1;
        if (agc.wait_counter > 125)
        {
                  agc.wait_flag = 0;
                    agc.wait_counter=0;
            agc.state = AGC_STATE_LOCKED;

        }
        break;
    }
    default:
        break;
    }
}

static void State_Locked_Handler(int32_t ir_net, int32_t red_net, int32_t ir_raw, int32_t red_raw)
{
    /* 1. 饱和监测 (使用原始光强) */
    if (ir_raw >= SATURATION_THRES || red_raw >= SATURATION_THRES)
    {
        Reset_To_Standby();
        return;
    }
        if (abs(agc.g_ir_dc) < 10000.0f || abs(agc.g_red_dc) < 10000.0f) {
        Reset_To_Standby();
            return;
        }
    /* 2. 等待物理计算与 R 计算缓冲区填满 */
    if (!agc.buf_filled || !agc.calc_filled) return;

    /* 3. 动态持续监测: 2秒滑动窗口判定 DC 波动率 */
    float dc_variance = 0.0f;
    if (agc.g_ir_dc > 0.1f) {
        /* g_ir_ac 代表 2 秒窗口内的 (Max - Min)，刚好可以作为 Delta DC */
        dc_variance = agc.g_ir_ac / agc.g_ir_dc;
    }

    /* 一旦变动（解锁）：如果2秒内DC的变化率超过 20% -> 强制返回 3mA重新探测 */
    if (dc_variance > 0.50f)
    {
        Reset_To_Standby(); /* 解锁状态硬件参数会改变，必须重置缓冲以防杂讯 */
        return;
    }


}

/* ================= 辅助函数实现 ================= */

static uint8_t led_ma_to_code(float ma)
{
    if (ma < 0) ma = 0;
    return (uint8_t)((ma / 100.0f) * 255.0f + 0.5f);
}

uint8_t calc_ambdac(int32_t dc_adc)
{
  float i_pd  = ((float)dc_adc / ADC_POS_FS) * 4.8f;
  float i_amb = i_pd * AMBDAC_RATIO;
  if (i_amb > AMBDAC_MAX_UA) i_amb = AMBDAC_MAX_UA;
  if (i_amb < 0) i_amb = 0;
  return (uint8_t)(i_amb + 0.5f);
}

static float calc_snr(int32_t *buf, int32_t dc)
{
    for (int i = 0; i < FFT_SIZE; i++)
        fft_in[i] = (float)(buf[i] - dc) * hann_win[i];

    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);
    arm_cmplx_mag_f32(fft_out, fft_mag, FFT_SIZE / 2);

    int bmin = (int)(HR_FREQ_MIN * (float)FFT_SIZE / SAMPLE_RATE_HZ);
    int bmax = (int)(HR_FREQ_MAX * (float)FFT_SIZE / SAMPLE_RATE_HZ);

    float sig = 0, noise = 0;
    for (int i = 1; i < FFT_SIZE / 2; i++)
    {
        if (i >= bmin && i <= bmax)
            sig += fft_mag[i];
        else
            noise += fft_mag[i];
    }

    if (noise < 1e-6f) noise = 1e-6f;
    return 20.0f * log10f(sig / noise);
}

static float calc_corr_nsdf(int32_t *buf, int32_t dc)
{
    for (int i = 0; i < FFT_SIZE; i++)
        nsdf_buf[i] = (float)(buf[i] - dc);

    int lag_min = (int)(SAMPLE_RATE_HZ / HR_FREQ_MAX);
    int lag_max = (int)(SAMPLE_RATE_HZ / HR_FREQ_MIN);

    float best = 0.0f;
    for (int lag = lag_min; lag <= lag_max; lag++)
    {
        float num = 0, den = 0;
        for (int i = 0; i < FFT_SIZE - lag; i++)
        {
            float x1 = nsdf_buf[i];
            float x2 = nsdf_buf[i + lag];
            num += x1 * x2;
            den += x1 * x1 + x2 * x2;
        }
        if (den > 1e-6f)
        {
            float v = 2.0f * num / den;
            if (v > best) best = v;
        }
    }
    return best;
}