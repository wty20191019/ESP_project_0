# ESP32-S3 FT8/FT4 业余无线电数字模式终端

基于 ESP32-S3 和 WM8978 音频编解码器的 FT8/FT4 数字通信电台固件，支持完整的 FT8/FT4 协议收发。

## 项目概述

本项目实现了一个完整的 FT8/FT4 弱信号数字模式终端，可用于业余无线电通信。系统通过 WM8978 音频芯片进行 12kHz 全双工音频采集和播放，使用 8-GFSK 调制方式实现 FT8/FT4 协议的编码发射和解码接收。

### 主要特性

- **双协议支持**：FT8（15s 时隙）和 FT4（7.5s 时隙）
- **全双工收发**：RX 和 TX 任务分别运行在不同核心，互不阻塞
- **实时解码**：持续音频采集，每时隙自动解码
- **精确时序**：严格对齐 UTC 时隙边界，支持奇偶时隙交替
- **GFSK 调制**：高斯成形 8-GFSK 波形，占用带宽约 50Hz
- **硬件加速**：PSRAM 波形预生成，优化的 FFT 解码

## 硬件要求

### 主控芯片
- ESP32-S3（支持 PSRAM）

### 音频编解码器
- WM8978（I2C 地址：0x1A）

### GPIO 接线定义

| 功能 | GPIO | 说明 |
|------|------|------|
| I2C SCL | GPIO3 | WM8978 I2C 时钟线 |
| I2C SDA | GPIO21 | WM8978 I2C 数据线 |
| I2S MCLK | GPIO12 | I2S 主时钟 |
| I2S BCLK | GPIO13 | I2S 位时钟 |
| I2S LRCK | GPIO14 | I2S 声道时钟 |
| I2S DOUT | GPIO15 | I2S 数据输出（MCU→CODEC） |
| I2S DIN | GPIO16 | I2S 数据输入（CODEC→MCU） |
| RGB LED | GPIO48 | WS2812 状态指示灯 |

### 音频规格
- 采样率：12kHz
- 位深：16 位
- 声道：双声道交织格式（L=R）
- I2S 模式：飞利浦标准

## 项目结构

```
ESP_project_0/
├── CMakeLists.txt                 # ESP-IDF 顶层项目文件
├── partitions-16MiB.csv           # 16MiB Flash 分区表
├── dependencies.lock              # 组件依赖锁定文件
├── main/                          # 应用主模块
│   ├── main.c                     # 入口：app_main()，配置并启动 ft8_app
│   ├── ft8_app.c                  # 核心应用控制器：RX/TX 双任务调度
│   ├── ft8_app.h                  # ft8_app 接口定义
│   ├── CMakeLists.txt             # main 组件构建文件
│   └── idf_component.yml          # 组件依赖声明
├── components/
│   ├── BSP/                       # 板级支持包
│   │   ├── wm8978/                # WM8978 音频编解码器驱动
│   │   │   ├── wm8978.c/.h        # 寄存器逻辑层（软件缓存表）
│   │   │   ├── wm8978_i2c.c/.h    # I2C 总线驱动
│   │   │   └── wm8978_i2s.c/.h    # I2S 全双工驱动
│   │   └── led/                   # RGB LED 驱动
│   │       └── led.c/.h           # WS2812 LED 控制
│   └── ft8_lib/                   # FT8/FT4 协议库
│       ├── ft8/                   # 协议核心
│       │   ├── encode.c/.h        # 编码：消息→payload→tone 序列
│       │   ├── decode.c/.h        # 解码：瀑布→候选→LDPC 解码
│       │   ├── message.c/.h       # 消息文本编解码
│       │   ├── ldpc.c/.h          # LDPC 纠错码
│       │   ├── crc.c/.h           # CRC 校验
│       │   ├── text.c/.h          # 文本处理
│       │   └── constants.c/.h     # 协议常量和查找表
│       ├── fft/                   # FFT 库
│       │   ├── kiss_fft.c/.h      # 基础 FFT
│       │   └── kiss_fftr.c/.h     # 实数 FFT
│       └── common/                # 公共模块
│           ├── monitor.c/.h       # 音频→瀑布图处理
│           └── common.h           # 公共定义
└── .devcontainer/                 # 开发容器配置
    ├── Dockerfile
    └── devcontainer.json
```

## 核心模块详解

### 1. 应用控制器 (`main/ft8_app.c`)

系统的核心，创建两个 FreeRTOS 任务：

**RX 任务** (`ft8_rx_task`)
- 运行核心：Core 1
- 栈大小：48KB
- 功能：持续读取 I2S 音频数据，每符号周期(160ms/48ms)做 STFT 分析，每时隙边界触发一次完整解码
- 输出：解码的 FT8/FT4 消息文本

**TX 任务** (`ft8_tx_task`)
- 运行核心：Core 0
- 栈大小：8KB
- 功能：启动时预生成 GFSK 波形存入 PSRAM，按奇偶时隙对齐 `esp_timer`，在目标时刻写出 I2S 数据
- 精度：忙等控制，起始误差 <1ms

### 2. FT8/FT4 协议库 (`components/ft8_lib/`)

移植自开源 [ft8_lib](https://github.com/kgoba/ft8_lib)，纯 C 实现：

- **编码流程**：消息文本 → payload(91 位) → LDPC 编码(174 位) → Gray 映射 → tone 序列(79/105 符号)
- **解码流程**：音频 STFT → 瀑布数据 → 候选搜索 → LDPC 置信传播解码 → 消息还原
- **关键参数**：
  - FT8：79 符号，符号周期 0.16s，时隙 15s
  - FT4：105 符号，符号周期 0.048s，时隙 7.5s
  - LDPC：(174,91) 码，83 个校验位

### 3. 音频驱动 (`components/BSP/wm8978/`)

三层架构：

1. **I2C 层** (`wm8978_i2c.c`)：I2C0 总线驱动，400kHz，支持写重试
2. **逻辑层** (`wm8978.c`)：维护 58 个寄存器的软件缓存表，配置 ADC/DAC/EQ/音量
3. **I2S 层** (`wm8978_i2s.c`)：I2S0 全双工驱动，DMA 双缓冲

## 数据流

### 接收链路

```
天线 → WM8978 ADC → I2S RX (12kHz, 16bit)
  → ft8_rx_task 读取音频样本
  → monitor_process() 做 STFT 累积到 waterfall
  → 时隙边界触发 rx_decode_slot()
    → ftx_find_candidates() 候选搜索
    → ftx_decode_candidate() LDPC 解码
    → ftx_message_decode() 还原呼号文本
  → ESP_LOGI 输出解码结果
```

### 发射链路

```
配置 callsign/grid/msg_mode
  → ft8_app_start() 初始化 WM8978 + I2S
  → ft8_tx_task 启动：
    → tx_encode_message() 文本→payload→tone 序列
    → tx_build_wave() GFSK 波形预生成(PSRAM)
    → 按奇偶时隙 + tx_delay_ms 等待目标时刻
    → wm8978_i2s_write() 写 I2S TX
    → WM8978 DAC → 天线
```

## 配置参数

在 `main/main.c` 的 `app_main()` 中配置 `ft8_app_config_t` 结构体：

```c
ft8_app_config_t cfg;
ft8_app_config_default(&cfg);

// 协议选择
cfg.protocol = FTX_PROTOCOL_FT8;    // 或 FTX_PROTOCOL_FT4

// 收发开关
cfg.tx_enable = true;               // 是否参与发射
cfg.rx_enable = true;               // 是否持续解码

// 时间配置
cfg.utc_enable = true;              // 对齐 UTC 时隙边界
cfg.tx_slot_parity = 0;             // 0=偶时隙发，1=奇时隙发
cfg.tx_delay_ms = 500;              // 时隙内延时发射(ms)

// 电台身份
snprintf(cfg.callsign, sizeof(cfg.callsign), "BG7ABC");
snprintf(cfg.grid, sizeof(cfg.grid), "JO70");

// 消息模式
cfg.msg_mode = FT8_APP_MSG_CQ;      // CQ 呼叫或呼叫指定台
cfg.cq_modifier[0] = '\0';          // CQ 修饰："DX"/"WW"/"TEST"

// 音频参数
cfg.audio_freq_hz = 1500.0f;        // 音频中心频率(Hz)
cfg.audio_level = 0.10f;            // 发射电平(0~1)

// 解码参数
cfg.rx_f_max = 3000.0f;             // 解码频率上限(Hz)
cfg.rx_time_osr = 2;                // 时间细分
cfg.rx_freq_osr = 2;                // 频率细分
cfg.max_candidates = 140;            // 每时隙最大候选数
cfg.ldpc_iterations = 25;           // LDPC 最大迭代次数
```

## 构建与烧录

### 环境要求

- ESP-IDF v5.0 或更高版本（测试版本：v6.1.0）
- ESP32-S3 开发板（支持 PSRAM）

### 构建命令

```bash
# 设置目标芯片
idf.py set-target esp32s3

# 构建项目
idf.py build

# 烧录固件
idf.py flash

# 监控串口输出
idf.py monitor
```

### 使用开发容器

项目提供了 `.devcontainer` 配置，支持 VS Code 开发容器：

1. 安装 VS Code 和 Remote - Containers 扩展
2. 打开项目文件夹
3. 按 F1 并选择 "Remote-Containers: Reopen in Container"

## 分区表

使用 16MiB Flash 分区表 (`partitions-16MiB.csv`)：

| 名称 | 类型 | 偏移 | 大小 | 说明 |
|------|------|------|------|------|
| nvs | data | 0x9000 | 24KB | NVS 存储 |
| phy_init | data | 0xF000 | 4KB | PHY 初始化数据 |
| factory | app | 0x10000 | 1.9MB | 应用固件 |
| vfs | data | 0x200000 | 10MB | FAT 文件系统 |
| storage | data | 0xC00000 | 4MB | SPIFFS 存储 |

## FT8/FT4 协议说明

### FT8 协议
- 时隙：15 秒
- 符号数：79（7 个同步 + 58 个数据 + 14 个校验）
- 符号周期：0.16 秒
- 调制：8-GFSK（8 种频率偏移）
- 带宽：约 50Hz
- 编码：LDPC(174,91) + CRC-14

### FT4 协议
- 时隙：7.5 秒
- 符号数：105（2 个 ramp + 16 个同步 + 87 个数据）
- 符号周期：0.048 秒
- 调制：4-GFSK（4 种频率偏移）
- 带宽：约 90Hz

### 消息格式

标准 FT8/FT4 消息格式：
- CQ 呼叫：`CQ [修饰符] <呼号> <网格>`
- 点呼：`<目标呼号> <呼号> <网格>`
- 信号报告：`<呼号> <呼号> <报告>`

示例：`CQ BG7ABC JO70` 或 `BG5ABC BG7ABC JO70`

## 注意事项

1. **时钟精度**：UTC 对齐依赖 SNTP 校时，但亚秒级精度受限于 `time()` 只有秒级。实际相位靠 `esp_timer` 本地栅格保证。

2. **I2C 只写**：WM8978 的 I2C 接口只能写不能读，驱动维护一份软件寄存器缓存表。

3. **PSRAM 使用**：波形缓冲优先使用外部 PSRAM，仅在 PSRAM 不足时回退到内部 SRAM。

4. **发射电平**：建议 `audio_level ≤ 0.9` 以防止削波失真。

5. **I2C 上拉**：如果 WM8978 模块板未外接 I2C 上拉电阻，建议将 I2C 频率降至 100kHz（修改 `wm8978_i2c.h` 中的 `WM8978_I2C_FREQ_HZ`）。

6. **编译优化**：ft8_lib 使用 `-O2` 优化（解码是重浮点运算），BSP 使用 `-O3 -ffast-math`。

## 开发指南

### 修改硬件引脚

所有 GPIO 定义集中在头文件中：
- I2C 引脚：`components/BSP/wm8978/wm8978_i2c.h:8`
- I2S 引脚：`components/BSP/wm8978/wm8978_i2s.h:10-14`
- LED 引脚：`components/BSP/led/led.h:5`

### 调整发射参数

修改 `main/main.c:36-51` 的 `cfg` 结构体。

### 添加新功能

1. **添加新的解码后处理**：在 `ft8_rx_task` 的 `rx_decode_slot()` 函数中添加逻辑
2. **添加新的消息模式**：扩展 `ft8_app_msg_mode_t` 枚举和 `tx_encode_message()` 函数
3. **添加 UI 显示**：在 `main.c` 中添加显示任务，读取 `s_stat_decoded` 等统计变量

## 故障排除

### 编译错误

- 确保 ESP-IDF 环境正确设置
- 运行 `idf.py fullclean` 清理构建缓存
- 检查 `dependencies.lock` 中的组件版本

### 运行时问题

- **WM8978 初始化失败**：检查 I2C 接线和上拉电阻
- **无解码输出**：检查天线连接、音频电平、频率配置
- **发射无信号**：检查 I2S 接线、发射电平、时隙配置

### 性能优化

- 增加 `rx_time_osr` 和 `rx_freq_osr` 可提高解码灵敏度，但增加 CPU 负载
- 减少 `max_candidates` 可降低解码计算量
- 使用 PSRAM 可释放内部 SRAM 给栈和堆使用

## 参考资料

- [FT8 协议规范](https://wsjt.sourceforge.io/FT8_DXpedition.pdf)
- [ft8_lib 开源库](https://github.com/kgoba/ft8_lib)
- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [WM8978 数据手册](https://www.wolfsonmicro.com/products/WM8978)

## 许可证

本项目基于 MIT 许可证开源。详见 LICENSE 文件。

## 贡献

欢迎提交 Issue 和 Pull Request！

## 联系方式

如有问题或建议，请通过 GitHub Issues 联系。
