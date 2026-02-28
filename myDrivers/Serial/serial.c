#include "serial.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <afe4490.h>
#include <afe4490_agc.h>

extern UART_HandleTypeDef huart1;

uint8_t IR_strength = 0;
uint8_t RED_strength = 0;
extern bool agc_locked;
extern float best_error;
extern uint8_t no_improve_count;

// 接收缓冲区
volatile uint8_t rx_buffer[256];
volatile uint16_t rx_index = 0;
extern uint8_t led_IR_idx, led_RED_idx;

// 全局变量，存储ID
uint8_t ID[4];

// 启动串口接收中断
void Serial_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_index], 1);
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart1, Array, Length, HAL_MAX_DELAY);
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)String, strlen(String), HAL_MAX_DELAY);
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;    // 设置结果初值为1
    while (Y--)             // 执行Y次
    {
        Result *= X;        // 将X累乘到结果
    }
    return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)    // 根据数字长度遍历数字的每一位
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0'); // 发送每位数字
    }
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);    // 将printf的底层重定向到自己的发送字节函数
    return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
    char String[100];               // 定义字符数组
    va_list arg;                    // 定义可变参数列表数据类型的变量arg
    va_start(arg, format);          // 从format开始，接收参数列表到arg变量
    vsprintf(String, format, arg);   // 使用vsprintf打印格式化字符串和参数列表到字符数组中
    va_end(arg);                    // 结束变量arg
    Serial_SendString(String);      // 串口发送字符数组（字符串）
}

void send_plot_data(int32_t ir_data, int32_t red_data, int32_t ir_amb, int32_t red_amb) 
{
    uint8_t buffer[27];  // 完整帧长度27字节
    // 发送红外数据帧 (ID=0x01)
    buffer[0] = 0xFA;   // 起始符
    buffer[1] = 0x1B;   // 帧长度
    buffer[2] = 0x03;   // 参数类型
    buffer[3] = 0x04;   // 数据包类型
    buffer[4] = 0x84;   // 数据包ID
    buffer[5] = ID[0];
    buffer[6] = ID[1];
    buffer[7] = ID[2];
    buffer[8] = ID[3];
    buffer[9] = (ir_data >> 24) & 0xFF;
    buffer[10] = (ir_data >> 16) & 0xFF;
    buffer[11] = (ir_data >> 8)  & 0xFF;
    buffer[12] = ir_data & 0xFF;
    buffer[13] = (red_data >> 24) & 0xFF;
    buffer[14] = (red_data >> 16) & 0xFF;
    buffer[15] = (red_data >> 8)  & 0xFF;
    buffer[16] = red_data & 0xFF;
    buffer[17] = (ir_amb >> 24) & 0xFF;  // 数据高位在前（大端序）
    buffer[18] = (ir_amb >> 16) & 0xFF;
    buffer[19] = (ir_amb >> 8)  & 0xFF;
    buffer[20] = ir_amb & 0xFF;
    buffer[21] = (red_amb >> 24) & 0xFF;  // 数据高位在前（大端序）
    buffer[22] = (red_amb >> 16) & 0xFF;
    buffer[23] = (red_amb >> 8)  & 0xFF;
    buffer[24] = red_amb & 0xFF;
    buffer[25] = 0;
    for(int i = 0; i < 25; i++) {
        buffer[25] += buffer[i];  // 累加校验
    }
    buffer[26] = 0x0A;   // 帧尾
    Serial_SendArray(buffer, 27);  // 发送27字节
}
void send_plot_data_BLE(int32_t ir_data, int32_t red_data) 
{
    uint8_t buffer[11];  // 完整帧长度27字节
    // 发送红外数据帧 (ID=0x01)
    buffer[0] = 0xFA;   // 起始符

    buffer[1] = (ir_data >> 24) & 0xFF;
    buffer[2] = (ir_data >> 16) & 0xFF;
    buffer[3] = (ir_data >> 8)  & 0xFF;
    buffer[4] = ir_data & 0xFF;
    buffer[5] = (red_data >> 24) & 0xFF;
    buffer[6] = (red_data >> 16) & 0xFF;
    buffer[7] = (red_data >> 8)  & 0xFF;
    buffer[8] = red_data & 0xFF;
    buffer[9] = 0;
    for(int i = 0; i < 9; i++) {
        buffer[9] += buffer[i];  // 累加校验
    }
    buffer[10] = 0x0A;   // 帧尾
    Serial_SendArray(buffer, 11);  // 发送27字节
}
void send_gain_data(uint8_t IR_gain1, uint8_t IR_gain2, uint8_t RED_gain1, uint8_t RED_gain2) 
{
    uint8_t buffer[15];  // 完整帧长度15字节
    buffer[0] = 0xFA;   // 起始符
    buffer[1] = 0x0F;   // 帧长度
    buffer[2] = 0x03;   // 参数类型
    buffer[3] = 0x04;   // 数据包类型
    buffer[4] = 0x89;   // 数据包ID
    buffer[5] = ID[0];
    buffer[6] = ID[1];
    buffer[7] = ID[2];
    buffer[8] = ID[3];
    buffer[9] = IR_gain1;
    buffer[10] = IR_gain2;
    buffer[11] = RED_gain1;
    buffer[12] = RED_gain2;
    buffer[13] = 0;
    for(int i = 0; i < 13; i++) {
        buffer[13] += buffer[i];  // 累加校验
    }
    buffer[14] = 0x0A;   // 帧尾
    Serial_SendArray(buffer, 15);  // 发送15字节
}

void send_strength_data(uint8_t IR_strength, uint8_t RED_strength) 
{
    uint8_t buffer[13];  // 完整帧长度13字节
    buffer[0] = 0xFA;   // 起始符
    buffer[1] = 0x0D;   // 帧长度
    buffer[2] = 0x03;   // 参数类型
    buffer[3] = 0x04;   // 数据包类型
    buffer[4] = 0x89;   // 数据包ID
    buffer[5] = ID[0];
    buffer[6] = ID[1];
    buffer[7] = ID[2];
    buffer[8] = ID[3];
    buffer[9] = IR_strength;
    buffer[10] = RED_strength;
    buffer[11] = 0;
    for(int i = 0; i < 11; i++) {
        buffer[11] += buffer[i];  // 累加校验
    }
    buffer[12] = 0x0A;   // 帧尾
    Serial_SendArray(buffer, 13);  // 发送13字节
}

/**
  * 函数：发送通用应答包
  * 参数：state 应答状态
  * 返回：无
  */
void send_ack(uint8_t state) 
{
    uint8_t buffer[12];  // 应答包长度12字节
    buffer[0] = 0xFA;   // 起始符
    buffer[1] = 0x0C;   // 帧长度
    buffer[2] = 0x03;   // 参数类型
    buffer[3] = 0x03;   // 数据包类型
    buffer[4] = 0x80;   // 数据包ID
    buffer[5] = 0x00;   // 保留字节
    buffer[6] = 0x00;   // 保留字节
    buffer[7] = 0x00;   // 保留字节
    buffer[8] = 0x01;   // 保留字节
    buffer[9] = state;  // 应答状态
    buffer[10] = 0;     // 校验和

    // 计算校验和
    for(int i = 0; i < 10; i++) {
        buffer[10] += buffer[i];
    }

    buffer[11] = 0x0A;   // 帧尾
    Serial_SendArray(buffer, 12);  // 发送应答包
}

/**
  * 函数：解析接收的数据包
  * 参数：无
  * 返回：无
  */
//void Parse_Data(void) 
//{
//    if(rx_index >= 12) { // 至少需要12字节才能构成一个完整的数据包
//        // 检查起始符和结束符
//        if(rx_buffer[0] == 0xFA && rx_buffer[rx_index-1] == 0x0A) {
//            // 检查帧长度
//            uint8_t length = rx_buffer[1];
//            
//            if(rx_index == length) { // 帧长度匹配
//                // 检查数据包类型和ID
//                if(rx_buffer[3] == 0x01) { // 控制数据包
//                    if(rx_buffer[4] == 0x06) { // 增益控制数据包
//                        // 解析增益值
//                        agc.IR_gain1 = rx_buffer[9];
//                        agc.IR_gain2 = rx_buffer[10];
//                        agc.RED_gain1 = rx_buffer[11];
//                        agc.RED_gain2 = rx_buffer[12];

//                        // 校验和检查
//                        uint8_t checksum = 0;
//                        for(int i = 0; i < length - 2; i++) {
//                            checksum += rx_buffer[i];
//                        }
//                        if(checksum == rx_buffer[length - 2]) {
//                            // 发送成功应答
//                            afe_adjust(agc.IR_gain1, agc.IR_gain2, agc.RED_gain1, agc.RED_gain2,
//                                      led_table[agc.led_IR_idx], led_table[agc.led_RED_idx]);
//                            send_ack(0x07);
//                        } else {
//                            // 校验和错误，发送校验和错误应答
//                            send_ack(0x06);
//                        }
//                    }
//                    else if(rx_buffer[4] == 0x07) { // 光强控制数据包
//                        // 解析光强值
//                        IR_strength = rx_buffer[9];
//                        RED_strength = rx_buffer[10];

//                        int idx = -1;
//                        for (int i = 0; i < 6; i++) {
//                            if (led_table[i] == IR_strength) {
//                                agc.led_IR_idx = i;
//                            }
//                            if (led_table[i] == RED_strength) {
//                                agc.led_RED_idx = i;
//                            }
//                        }

//                        uint8_t checksum = 0;
//                        for(int i = 0; i < length - 2; i++) {
//                            checksum += rx_buffer[i];
//                        }
//                        if(checksum == rx_buffer[length - 2]) {
//                            // 发送成功应答
//                            afe_adjust(agc.IR_gain1, agc.IR_gain2, agc.RED_gain1, agc.RED_gain2,
//                                      led_table[agc.led_IR_idx], led_table[agc.led_RED_idx]);
//                            send_ack(0x07);
//                        } else {
//                            // 校验和错误，发送校验和错误应答
//                            send_ack(0x06);
//                        }
//                    }
//                    else if(rx_buffer[4] == 0x08) { // 重置AGC数据包
//                        // 校验和检查
//                        uint8_t checksum = 0;
//                        for(int i = 0; i < length - 2; i++) {
//                            checksum += rx_buffer[i];
//                        }
//                        if(checksum == rx_buffer[length - 2]) {
//                            agc_reset();
//                            // 发送成功应答
//                            send_ack(0x07);
//                        } else {
//                            // 校验和错误，发送校验和错误应答
//                            send_ack(0x06);
//                        }
//                    }
//                }
//            }
//        }
//    }
//    // 清空接收缓冲区
//    rx_index = 0;
//}

/**
  * 函数：串口接收中断回调函数
  * 参数：huart UART句柄指针
  * 返回：无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1) {
        // 检查缓冲区是否溢出
        if(rx_index < sizeof(rx_buffer) - 1) {
            // 检查是否收到结束符
            if(rx_buffer[rx_index] == 0x0A) {
                rx_index++; // 包含结束符
               // Parse_Data();
            } else {
                rx_index++; // 移动到下一个位置
            }
            
            // 重新启动接收中断
            HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_index], 1);
        } else {
            // 缓冲区溢出，重置索引
            rx_index = 0;
            // 重新启动接收中断
            HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_index], 1);
        }
    }
}

