#include "afe4490.h"
#include "main.h"  // 包含HAL库和SPI/ GPIO配置

// 片选引脚定义（根据实际硬件连接修改）
#define AFE_CS_PIN   GPIO_PIN_14
#define AFE_CS_PORT  GPIOB

// SPI句柄（需在CubeMX中配置并生成）
extern SPI_HandleTypeDef hspi1;

// AFE44XX初始化
static void afe_init(void) {
    // 初始化片选GPIO
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(500);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(500);


    // 写入初始化寄存器配置
    afe_write(CONTROL0, 0x000000);
    afe_write(CONTROL0, 0x000008);
    afe_write(TIAGAIN, 0x00C000);       // TIA增益：CF=5pF, RF=500kΩ
    afe_write(TIA_AMB_GAIN, 0x004000);  // 环境光增益
    afe_write(LEDCNTRL, 0x002020);      // LED电流范围及强度
    afe_write(CONTROL2, 0x020000);      // LED范围100mA，LED电流50mA
    afe_write(CONTROL1, 0x000704);      // 定时器使能，平均5次采样
    afe_write(PRPCOUNT, 0x00000F9F);    // 脉冲重复周期
    // 以下为时序配置（根据数据手册调整具体值）
    afe_write(LED2STC, 0x00000BB8);
    afe_write(LED2ENDC, 0x00000F9E);
    afe_write(LED2LEDSTC, 0x00000BB8);
    afe_write(LED2LEDENDC, 0x000000F9F);
    afe_write(ALED2STC, 0x00000000);
    afe_write(ALED2ENDC, 0x000003E6);
    afe_write(LED2CONVST, 0x00000002);
    afe_write(LED2CONVEND, 0x000003E7);
    afe_write(ALED2CONVST, 0x000003EA);
    afe_write(ALED2CONVEND, 0x000007CF);
    afe_write(LED1STC, 0x000003E8);
    afe_write(LED1ENDC, 0x000007CE);
    afe_write(LED1LEDSTC, 0x000003E8);
    afe_write(LED1LEDENDC, 0x000007CF);
    afe_write(ALED1STC, 0x000007D0);
    afe_write(ALED1ENDC, 0x00000BB6);
    afe_write(LED1CONVST, 0x000007D2);
    afe_write(LED1CONVEND, 0x00000BB7);
    afe_write(ALED1CONVST, 0x00000BBA);
    afe_write(ALED1CONVEND, 0x000000F9F);
    // ADC复位配置
    afe_write(ADCRSTCNT0, 0x00000000);
    afe_write(ADCRSTENDCT0, 0x00000000);
    afe_write(ADCRSTCNT1, 0x000003E8);
    afe_write(ADCRSTENDCT1, 0x000003E8);
    afe_write(ADCRSTCNT2, 0x000007D0);
    afe_write(ADCRSTENDCT2, 0x000007D0);
    afe_write(ADCRSTCNT3, 0x00000BB8);
    afe_write(ADCRSTENDCT3, 0x00000BB8);

    HAL_Delay(1000); // 等待AFE稳定
}

// SPI写入寄存器（reg: 寄存器地址，value: 24位数据）
void afe_write(uint8_t reg, uint32_t value) {
    // 拉低片选使能SPI
    HAL_GPIO_WritePin(AFE_CS_PORT, AFE_CS_PIN, GPIO_PIN_RESET);

    // 构造发送数据：寄存器地址 + 24位数据（高位在前）
    uint8_t tx_buf[4] = {
        reg,                      // 寄存器地址
        (value >> 16) & 0xFF,     // 数据最高8位
        (value >> 8) & 0xFF,      // 数据中间8位
        value & 0xFF              // 数据最低8位
    };

    // 发送数据（忽略接收数据）
    HAL_SPI_Transmit(&hspi1, tx_buf, 4, HAL_MAX_DELAY);

    // 拉高片选禁用SPI
    HAL_GPIO_WritePin(AFE_CS_PORT, AFE_CS_PIN, GPIO_PIN_SET);
}


static void afe_read_data(uint8_t reg_addr, afe44xx_data *data1) {
    /**************************
     * 1. 写CONTROL0寄存器（原afe_write(CONTROL0, 0x000001)）
     **************************/
    uint8_t ctrl_reg_addr = CONTROL0;          // 控制寄存器地址
    uint8_t ctrl_data[3] = {                   // 24位数据拆分（高位在前）
        (0x000001 >> 16) & 0xFF,  // 数据最高8位（0x00）
        (0x000001 >> 8) & 0xFF,   // 数据中间8位（0x00）
        0x000001 & 0xFF           // 数据最低8位（0x01）
    };
    // 硬件NSS自动控制CS → 发送寄存器地址 → 发送数据
		    HAL_GPIO_WritePin(AFE_CS_PORT, AFE_CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, &ctrl_reg_addr, 1, HAL_MAX_DELAY); // 发寄存器地址
    HAL_SPI_Transmit(&hspi1, ctrl_data, 3, HAL_MAX_DELAY);      // 发24位数据

    HAL_GPIO_WritePin(AFE_CS_PORT, AFE_CS_PIN, GPIO_PIN_SET);

    /**************************
     * 2. 读目标寄存器（原afe_read(reg_addr)）
     **************************/
    uint8_t target_reg_addr = reg_addr;        // 目标寄存器地址
    uint8_t rx_buf[3] = {0};                   // 存储接收的24位数据
    // 硬件NSS自动控制CS → 发送目标寄存器地址 → 接收数据
				    HAL_GPIO_WritePin(AFE_CS_PORT, AFE_CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, &target_reg_addr, 1, HAL_MAX_DELAY); // 发目标寄存器地址
    HAL_SPI_Receive(&hspi1, rx_buf, 3, HAL_MAX_DELAY);            // 收24位数据
    HAL_GPIO_WritePin(AFE_CS_PORT, AFE_CS_PIN, GPIO_PIN_SET);


    /**************************
     * 3. 原数据解析与符号扩展（完全保留）
     **************************/
    // 组合24位原始数据
    uint32_t raw_data = ((uint32_t)rx_buf[0] << 16) | 
                        ((uint32_t)rx_buf[1] << 8) | 
                        rx_buf[2];
    raw_data &= 0x3FFFFF; // 提取低22位（AFE44XX寄存器数据位宽）

    // 22位补码转32位int32_t（符号扩展）
    int32_t data = (int32_t)raw_data;
    if (data & (1 << 21)) { // 检查符号位（第21位）
        data |= 0xFFC00000; // 扩展高位为1，保持补码
    }


    /**************************
     * 4. 原存储逻辑（完全保留）
     **************************/
    switch (reg_addr) {
    case LED2VAL:
        data1->RED_data = data;
        break;
    case ALED2VAL:
        data1->ARED_data = data;
        break;
    case LED1VAL:
        data1->IR_data = data;
        break;
    case ALED1VAL:
        data1->AIR_data = data;
        break;
    case LED1ABSVAL:
        data1->IRABSVAL_data = data;
        break;
    case LED2ABSVAL:
        data1->REDABSVAL_data = data;
        break;
    }
}

// 调整增益和LED强度
void afe_adjust(uint8_t IR_gain1, uint8_t IR_gain2, 
                uint8_t RED_gain1, uint8_t RED_gain2, 
                uint8_t IR_strength, uint8_t RED_strength,uint8_t ambdac_code) {
    afe_write(CONTROL0, 0x000000); // 退出读取模式
    // 设置TIA增益（IR通道）
    afe_write(TIAGAIN, 0x00C000 | ((IR_gain2 << 8) |(uint8_t)0  << 3) | IR_gain1);
    // 设置环境光增益（RED通道）
    afe_write(TIA_AMB_GAIN, ((uint8_t)ambdac_code << 16)|(1<<14)| ((RED_gain2 << 8) |(uint8_t)0  << 3|  RED_gain1));
    // 设置LED电流强度
    afe_write(LEDCNTRL, (IR_strength << 8) | RED_strength);
}

// 初始化句柄
AFE44XX_HandleTypeDef hAFE44xx = {
    .init = afe_init,
    .read_data = afe_read_data
};