#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <main.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
void send_plot_data(int32_t ir_data, int32_t red_data,int32_t irdiff,int32_t reddiff) ;
void send_strength_data(uint8_t IR_strength, uint8_t RED_strength);
void send_gain_data(uint8_t IR_gain1, uint8_t IR_gain2,uint8_t RED_gain1,uint8_t RED_gain2);

#endif