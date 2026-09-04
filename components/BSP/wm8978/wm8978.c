#include "wm8978.h"
#include "wm8978_i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

/* WM8978：WM8978 音频编解码芯片寄存器驱动（逻辑层），
 * 移植自正点原子 STM32F407 录音机/音乐播放器参考实验的 wm8978.c。
 * I2C 硬件访问统一经由 wm8978_i2c 模块，删除了原实验中的 STM32 GPIO 配置。 */

static const char *TAG = "WM8978";

/* WM8978 寄存器默认值表（共 58 个寄存器 0~57，值宽 9 位）。
 * 由于 WM8978 的 I2C 接口只支持写而不支持读，写寄存器时必须同步更新
 * 该缓存表，读寄存器时直接返回缓存值。 */
static uint16_t WM8978_REGVAL_TBL[58] = {
    0X0000, 0X0000, 0X0000, 0X0000, 0X0050, 0X0000, 0X0140, 0X0000,
    0X0000, 0X0000, 0X0000, 0X00FF, 0X00FF, 0X0000, 0X0100, 0X00FF,
    0X00FF, 0X0000, 0X012C, 0X002C, 0X002C, 0X002C, 0X002C, 0X0000,
    0X0032, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000,
    0X0038, 0X000B, 0X0032, 0X0000, 0X0008, 0X000C, 0X0093, 0X00E9,
    0X0000, 0X0000, 0X0000, 0X0000, 0X0003, 0X0010, 0X0010, 0X0100,
    0X0100, 0X0002, 0X0001, 0X0001, 0X0039, 0X0039, 0X0039, 0X0039,
    0X0001, 0X0001
};

/* WM8978 初始化
 * 返回值：0，初始化成功；非 0，初始化失败（WM8978 异常或 I2C 通信失败） */
uint8_t WM8978_Init(void)
{
    /* 先初始化 I2C 底层（幂等） */
    if (wm8978_i2c_init() != ESP_OK) return 1;

    /* 软件复位 WM8978 并确认通信正常。上电初期(电源/复位未稳定)首帧可能失败，
     * 整体重试 3 次，每次间隔 50ms；I2C 底层另有逐帧重试。 */
    uint8_t ok = 0;
    for (int i = 0; i < 3; i++) {
        if (i > 0) esp_rom_delay_us(50 * 1000);
        if (WM8978_Write_Reg(0, 0) == 0) { ok = 1; break; }
    }
    if (!ok) {
        ESP_LOGE(TAG, "WM8978 软件复位失败，请检查 I2C 接线/供电");
        return 1;
    }

    WM8978_Write_Reg(1, 0X1B);   /* R1：MICEN=1(MIC 使能)，BIASEN=1(模拟偏置使能)，VMIDSEL=11(5K) */
    WM8978_Write_Reg(2, 0X1B0);  /* R2：ROUT1/LOUT1 使能(耳机功放)，BOOSTENR/BOOSTENL 使能 */
    WM8978_Write_Reg(3, 0X6C);   /* R3：LOUT2/ROUT2 使能(喇叭功放)，RMIX/LMIX 使能 */
    WM8978_Write_Reg(6, 0);      /* R6：MCLK 由外部(ESP32 I2S0)提供 */
    WM8978_Write_Reg(43, 1 << 4);   /* R43：INVROUT2 置位，输出反相 */
    WM8978_Write_Reg(47, 1 << 8);   /* R47：PGABOOSTL 使能，左通道 MIC 加 20dB */
    WM8978_Write_Reg(48, 1 << 8);   /* R48：PGABOOSTR 使能，右通道 MIC 加 20dB */
    WM8978_Write_Reg(49, 1 << 1);   /* R49：TSDEN，开启过热保护 */
    WM8978_Write_Reg(10, 1 << 3);   /* R10：SOFTMUTE 关闭，128x 过采样，提升 SNR */
    WM8978_Write_Reg(14, 1 << 3);   /* R14：ADC 128x 过采样 */
    return 0;
}

/* WM8978 写寄存器
 * reg：寄存器地址；val：寄存器值(9 位)
 * 返回值：0，成功；非 0，失败 */
uint8_t WM8978_Write_Reg(uint8_t reg, uint16_t val)
{
    if (wm8978_i2c_write_reg(reg, val) != ESP_OK) {
        return 1;   /* 错误详情已在 wm8978_i2c 底层打印 */
    }
    WM8978_REGVAL_TBL[reg] = val;   /* 同步更新软件缓存表 */
    return 0;
}

/* WM8978 读寄存器（读取软件缓存表中的对应值）
 * reg：寄存器地址；返回值：寄存器值 */
uint16_t WM8978_Read_Reg(uint8_t reg)
{
    if (reg >= 58) return 0;
    return WM8978_REGVAL_TBL[reg];
}

/* WM8978 DAC/ADC 通路使能
 * adcen：ADC 使能(1)/关闭(0)；dacen：DAC 使能(1)/关闭(0) */
void WM8978_ADDA_Cfg(uint8_t dacen, uint8_t adcen)
{
    uint16_t regval;
    regval = WM8978_Read_Reg(3);            /* 读 R3 */
    if (dacen) regval |= 3 << 0;            /* R3 低 2 位置 1，使能 DACR & DACL */
    else regval &= ~(3 << 0);               /* R3 低 2 位清 0，关闭 DACR & DACL */
    WM8978_Write_Reg(3, regval);            /* 写回 R3 */
    regval = WM8978_Read_Reg(2);            /* 读 R2 */
    if (adcen) regval |= 3 << 0;            /* R2 低 2 位置 1，使能 ADCR & ADCL */
    else regval &= ~(3 << 0);               /* R2 低 2 位清 0，关闭 ADCR & ADCL */
    WM8978_Write_Reg(2, regval);            /* 写回 R2 */
}

/* WM8978 输入通道配置
 * micen：MIC 使能(1)/关闭(0)
 * lineinen：Line In 使能(1)/关闭(0)
 * auxen：aux 使能(1)/关闭(0) */
void WM8978_Input_Cfg(uint8_t micen, uint8_t lineinen, uint8_t auxen)
{
    uint16_t regval;
    regval = WM8978_Read_Reg(2);            /* 读 R2 */
    if (micen) regval |= 3 << 2;            /* 使能 INPPGAENR/INPPGAENL(MIC 的 PGA 放大) */
    else regval &= ~(3 << 2);               /* 关闭 INPPGAENR/INPPGAENL */
    WM8978_Write_Reg(2, regval);            /* 写回 R2 */

    regval = WM8978_Read_Reg(44);           /* 读 R44 */
    if (micen) regval |= (3 << 4) | (3 << 0);   /* 使能 LIN2INPPGA/LIP2INPGA/RIN2INPPGA/RIP2INPGA */
    else regval &= ~((3 << 4) | (3 << 0));      /* 关闭上述输入通路 */
    WM8978_Write_Reg(44, regval);           /* 写回 R44 */

    if (lineinen) WM8978_LINEIN_Gain(5);    /* LINE IN 0dB 增益 */
    else WM8978_LINEIN_Gain(0);             /* 关闭 LINE IN */
    if (auxen) WM8978_AUX_Gain(7);          /* AUX 6dB 增益 */
    else WM8978_AUX_Gain(0);                /* 关闭 AUX 输入 */
}

/* WM8978 输出通道配置
 * dacen：DAC 输出(功放前级)使能(1)/关闭(0)
 * bpsen：Bypass 直通(监听 MIC/LINE IN/AUX 等)使能(1)/关闭(0) */
void WM8978_Output_Cfg(uint8_t dacen, uint8_t bpsen)
{
    uint16_t regval = 0;
    if (dacen) regval |= 1 << 0;            /* DAC 输出使能 */
    if (bpsen) {
        regval |= 1 << 1;                   /* BYPASS 使能 */
        regval |= 5 << 2;                   /* 0dB 增益 */
    }
    WM8978_Write_Reg(50, regval);           /* R50 左声道输出配置 */
    WM8978_Write_Reg(51, regval);           /* R51 右声道输出配置 */
}

/* WM8978 MIC 输入增益(含 BOOST 20dB，MIC-->ADC 输入部分增益)
 * gain：0~63，对应 -12dB ~ +35.25dB，0.75dB/步 */
void WM8978_MIC_Gain(uint8_t gain)
{
    gain &= 0X3F;
    WM8978_Write_Reg(45, gain);             /* R45 左声道 PGA 增益 */
    WM8978_Write_Reg(46, gain | (1 << 8));  /* R46 右声道 PGA 增益 */
}

/* WM8978 LINE IN 输入增益(L2/R2-->ADC 输入部分增益)
 * gain：0~7，0 表示通路截止，1~7 对应 -12dB ~ +6dB，3dB/步 */
void WM8978_LINEIN_Gain(uint8_t gain)
{
    uint16_t regval;
    gain &= 0X07;
    regval = WM8978_Read_Reg(47);           /* 读 R47 */
    regval &= ~(7 << 4);                    /* 清原左声道增益位 */
    WM8978_Write_Reg(47, regval | (gain << 4)); /* 写回 R47 */
    regval = WM8978_Read_Reg(48);           /* 读 R48 */
    regval &= ~(7 << 4);                    /* 清原右声道增益位 */
    WM8978_Write_Reg(48, regval | (gain << 4)); /* 写回 R48 */
}

/* WM8978 AUX 输入增益(AUXR/AUXL-->ADC 输入部分增益)
 * gain：0~7，0 表示通路截止，1~7 对应 -12dB ~ +6dB，3dB/步 */
void WM8978_AUX_Gain(uint8_t gain)
{
    uint16_t regval;
    gain &= 0X07;
    regval = WM8978_Read_Reg(47);           /* 读 R47 */
    regval &= ~(7 << 0);                    /* 清原左声道增益位 */
    WM8978_Write_Reg(47, regval | gain);    /* 写回 R47 */
    regval = WM8978_Read_Reg(48);           /* 读 R48 */
    regval &= ~(7 << 0);                    /* 清原右声道增益位 */
    WM8978_Write_Reg(48, regval | gain);    /* 写回 R48 */
}

/* 配置 I2S 音频格式（需与 wm8978_i2s 模块保持一致）
 * fmt：0,LSB(右对齐)；1,MSB(左对齐)；2,飞利浦标准 I2S；3,PCM/DSP
 * len：0,16 位；1,20 位；2,24 位；3,32 位 */
void WM8978_I2S_Cfg(uint8_t fmt, uint8_t len)
{
    fmt &= 0X03;
    len &= 0X03;                                        /* 限定范围 */
    WM8978_Write_Reg(4, (fmt << 3) | (len << 5));       /* R4，WM8978 音频格式配置 */
}

/* 耳机输出音量设置
 * voll：左声道音量(0~63)；volr：右声道音量(0~63) */
void WM8978_HPvol_Set(uint8_t voll, uint8_t volr)
{
    voll &= 0X3F;
    volr &= 0X3F;                                       /* 限定范围 */
    if (voll == 0) voll |= 1 << 6;                      /* 音量为 0 时直接 mute */
    if (volr == 0) volr |= 1 << 6;                      /* 音量为 0 时直接 mute */
    WM8978_Write_Reg(52, voll);                         /* R52 左声道耳机音量 */
    WM8978_Write_Reg(53, volr | (1 << 8));              /* R53 右声道耳机音量，同步加载(HPVU=1) */
}

/* 喇叭输出音量设置
 * volx：音量(0~63) */
void WM8978_SPKvol_Set(uint8_t volx)
{
    volx &= 0X3F;                                       /* 限定范围 */
    if (volx == 0) volx |= 1 << 6;                      /* 音量为 0 时直接 mute */
    WM8978_Write_Reg(54, volx);                         /* R54 左声道喇叭音量 */
    WM8978_Write_Reg(55, volx | (1 << 8));              /* R55 右声道喇叭音量，同步加载(SPKVU=1) */
}

/* 设置 3D 音效强度
 * depth：0~15(0 关闭，15 最强) */
void WM8978_3D_Set(uint8_t depth)
{
    depth &= 0XF;                                       /* 限定范围 */
    WM8978_Write_Reg(41, depth);                        /* R41 3D 效果配置 */
}

/* 设置 EQ/3D 作用方向
 * dir：0，作用于 ADC 通路；1，作用于 DAC 通路(默认) */
void WM8978_EQ_3D_Dir(uint8_t dir)
{
    uint16_t regval;
    regval = WM8978_Read_Reg(0X12);
    if (dir) regval |= 1 << 8;
    else regval &= ~(1 << 8);
    WM8978_Write_Reg(18, regval);                       /* R18，EQ1 的第 9 位配置 EQ/3D 方向 */
}

/* 设置 EQ1 频段
 * cfreq：截止频率，0~3，分别对应 80/105/135/175Hz
 * gain：增益，0~24，对应 -12dB ~ +12dB */
void WM8978_EQ1_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval;
    cfreq &= 0X3;                                       /* 限定范围 */
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval = WM8978_Read_Reg(18);
    regval &= 0X100;
    regval |= cfreq << 5;                               /* 设置截止频率 */
    regval |= gain;                                     /* 设置增益 */
    WM8978_Write_Reg(18, regval);                       /* R18，EQ1 配置 */
}

/* 设置 EQ2 频段
 * cfreq：截止频率，0~3，分别对应 230/300/385/500Hz
 * gain：增益，0~24，对应 -12dB ~ +12dB */
void WM8978_EQ2_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;                               /* 设置截止频率 */
    regval |= gain;                                     /* 设置增益 */
    WM8978_Write_Reg(19, regval);                       /* R19，EQ2 配置 */
}

/* 设置 EQ3 频段
 * cfreq：截止频率，0~3，分别对应 650/850/1100/1400Hz
 * gain：增益，0~24，对应 -12dB ~ +12dB */
void WM8978_EQ3_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(20, regval);                       /* R20，EQ3 配置 */
}

/* 设置 EQ4 频段
 * cfreq：截止频率，0~3，分别对应 1800/2400/3200/4100Hz
 * gain：增益，0~24，对应 -12dB ~ +12dB */
void WM8978_EQ4_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(21, regval);                       /* R21，EQ4 配置 */
}

/* 设置 EQ5 频段
 * cfreq：截止频率，0~3，分别对应 5300/6900/9000/11700Hz
 * gain：增益，0~24，对应 -12dB ~ +12dB */
void WM8978_EQ5_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(22, regval);                       /* R22，EQ5 配置 */
}
