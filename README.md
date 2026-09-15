# ESP_project_0 — ESP32-S3 FT8/FT4 自动通联终端

基于 **ESP32-S3**（16MB Flash + 8MB PSRAM，ESP-IDF v6.x）的 FT8/FT4 业余无线电数字模式收发终端。

它通过 WM8978 音频编解码器采集/播放电台音频，实时解码 FT8/FT4 信号，在 128×160 小屏上画瀑布和解码列表，按 **UTC 时隙**自动收发，用状态机自动完成一次完整通联（QSO），并把结果以 **ADIF** 格式写入 FAT 分区（可经 USB 暴露成 U 盘给电脑读取）。

- 解码/编码内核：Karlis Goba 的 [`ft8_lib`](https://github.com/kgoba/ft8_lib)（已移植为 `components/ft8_lib`）
- 音频编解码：WM8978（I2C 控制 + I2S 全双工）
- 定位/授时：GNSS（NMEA + PPS 秒脉冲）
- 显示/交互：ST7735 TFT + 7 按键 + WS2812 状态灯
- 日志导出：FAT + Wear-Leveling + TinyUSB MSC

---

## 1. 硬件平台与接线

主控：`esp32 s3n16r8`（16MB Flash，8MB Octal PSRAM），开发环境 ESP-IDF 6.1。

| 外设 | 接口 | 引脚 | 说明 |
|---|---|---|---|
| **WM8978** | I2C0 | `SCL=GPIO3`，`SDA=GPIO21` | 从机地址 `0x1A`，默认 400kHz |
| **WM8978** | I2S0 | `MCLK=GPIO12`，`BCLK=GPIO13`，`LRCK=GPIO14` | ESP32-S3 为主机，WM8978 为从机 |
| **WM8978** | I2S0 | `DOUT=GPIO15`（MCU→CODEC），`DIN=GPIO16`（CODEC→MCU） | 16bit、双声道、采样率 **12000 Hz** |
| **GNSS** | UART1 | `TX=GPIO17`（S3→GNSS），`RX=GPIO18`（GNSS→S3） | 默认 9600 baud |
| **GNSS PPS** | GPIO 中断 | `PPS=GPIO9` | 上升沿中断，用于 UTC 时隙精对齐 |
| **LCD ST7735** | SPI2 | `SCK=GPIO10`，`MOSI=GPIO11`，`CS=GPIO42`，`DC=GPIO4`，`RST=GPIO5`，`BLK=GPIO6` | 1.8 寸 128×160，SPI 40MHz |
| **按键** | GPIO | `UP=2`，`DOWN=7`，`LEFT=38`，`RIGHT=39`，`MID=40`，`SET=41`，`RST=47` | 低电平有效 |
| **RGB 状态灯** | RMT | `LED=GPIO48` | WS2812/SK68xx |
| **原生 USB** | USB-OTG | `D-=GPIO19`，`D+=GPIO20` | TinyUSB MSC（U 盘） |
| **串口控制台** | UART0 | `TX=GPIO43`，`RX=GPIO44` | 接 CH340，115200 |

> 引脚宏定义位置：`components/BSP/wm8978/wm8978_i2c.h`、`wm8978_i2s.h`、`components/BSP/GPS/gps.h`、`components/BSP/LCD_st7735/lcd_init.h`、`components/BSP/key/key.h`、`components/BSP/led/led.h`。
> 根目录 `board_pins.txt` 是硬件备忘（含 GPIO 复用、Strapping 脚等注意事项），非代码。

---

## 2. 项目结构

```
ESP_project_0/
├── CMakeLists.txt              # 标准 IDF 工程入口
├── sdkconfig                   # 工程配置(含关键项, 见 §8)
├── partitions-16MiB.csv        # 分区表
├── board_pins.txt              # 引脚/硬件备忘
│
├── main/                       # 应用层
│   ├── main.c                  # 入口 app_main + LCD 界面 + 按键 + GPS 搬运 + LED
│   ├── ft8_app.c / .h          # 核心控制器: 收发任务 / 解码调度 / UTC 栅格 / 自动 QSO 引擎
│   ├── ft8_qso.h               # QSO 引擎数据结构(事件/状态机/上下文)
│   ├── qso_log.c / .h          # ADIF 日志 / FAT 存储 / cfg.txt 持久化 / USB MSC
│   └── idf_component.yml       # 依赖: espressif/led_strip, espressif/esp_tinyusb
│
├── components/
│   ├── BSP/                    # 板级支持包(单个 IDF 组件, 多 SRC_DIRS)
│   │   ├── wm8978/             # 音频编解码: 寄存器逻辑 + I2C 底层 + I2S 全双工
│   │   ├── GPS/                # GNSS: NMEA 解析 + PPS 中断
│   │   ├── LCD_st7735/         # ST7735 显示驱动
│   │   ├── key/                # 7 按键回调驱动
│   │   └── led/                # WS2812 驱动
│   │
│   └── ft8_lib/                # 移植的 FT8/FT4 编解码库
│       ├── ft8/                # encode/decode/message/ldpc/crc/text/constants
│       ├── fft/                # kiss_fft
│       └── common/             # monitor(瀑布 DSP, 仅保留 monitor.{c,h})
│
└── managed_components/         # IDF 组件管理器下载的第三方组件
    ├── espressif__esp_tinyusb/ # TinyUSB MSC 封装(含 storage_spiflash/tinyusb_msc)
    ├── espressif__tinyusb/     # TinyUSB 协议栈本体
    └── espressif__led_strip/   # WS2812 RMT 驱动
```

---

## 3. 系统架构与任务关系

### 3.1 任务一览

| 任务名 | 创建处 | 栈 | 优先级 | 核 | 职责 |
|---|---|---|---|---|---|
| `main` | IDF 框架 | — | 1 | 0 | `app_main`：初始化 + 装配所有任务后返回 |
| `ft8_tx` | `ft8_app.c` | 8KB | **7** | 0 | 按奇偶时隙精确起播 GFSK 波形 |
| `ft8_rx` | `ft8_app.c` | 48KB | 6 | 0 | I2S 采集 → 喂瀑布；时隙末交接解码 |
| `ft8_dec` | `ft8_app.c` | 24KB | 5 | **1** | 独立解码（与收发核隔离） |
| `ft8_qso` | `ft8_app.c` | 4KB | 4 | 0 | 自动 QSO 状态机 |
| `gps_utc` | `main.c` | 4KB | 5 | 1 | 把 GPS UTC/PPS 搬进 `cfg.gps` |
| `LCD` | `main.c` | 4KB | 1 | 0 | 5 页界面绘制 + 按键回调 |
| `rgb_led` | `main.c` | 2KB | 1 | 0 | 发射/接收状态灯 |
| `gps_parse` | `gps.c` | 4KB | 8 | — | NMEA 解析 |
| `gps_pps` | `gps.c` | 3KB | 10 | — | PPS 中断回调处理 |
| `tinyusb` | `esp_tinyusb` | — | — | — | USB MSC 协议栈 |

> 关键设计：**收发固定在 core0**（TX 优先级最高，忙等到点起播，误差 <1ms），**解码独占 core1**，因此解码再慢也不会挤占采集/发射。

### 3.2 模块依赖关系

```
┌──────────────────────────────────────────┐
                 │                app_main                   │
                 │  key_init / gps_init / cfg 默认值          │
                 │  qso_log_init → cfg_store_load → usb_start │
                 └───────────────┬──────────────────────────┘
                                 │ 传入 &cfg（引用，不拷贝）
        ┌────────────────────────┼─────────────────────────┐
        ▼                        ▼                         ▼
 ┌─────────────┐        ┌─────────────────┐       ┌────────────────┐
 │ ft8_app     │◄──────►│ qso_log         │       │ LCD_task       │
 │ (RX/TX/DEC/ │ 回调    │ (ADIF/FAT/USB)  │       │ gps_time_task  │
 │  QSO 引擎)  │        └─────────────────┘       │ rgb_led_task   │
 └──────┬──────┘                                  └────────────────┘
        │ 依赖
        ▼
 ┌──────────────┐   ┌──────────────┐
 │ components/  │   │ components/  │
 │ ft8_lib      │   │ BSP          │
 │ (编解码/DSP) │   │ (wm8978/GPS/ │
 └──────────────┘   │  LCD/key/led)│
                    └──────────────┘
```

---

## 4. 数据流

### 4.1 接收（RX）链路

```
电台音频 → WM8978 ADC → I2S0 RX (12000Hz, 16bit 双声道)
   │
   ▼  ft8_rx_task: wm8978_i2s_read() 每符号 1920 样本(FT8)
   │
   ├─► monitor_process() ──► 幅度瀑布(waterfall)
   │        │
   │        └─► wf_disp_add() ──► ft8_wf_snap_t(无锁环形) ──► LCD 主页瀑布绘制
   │
   └─► 时隙结束前 rx_handoff_slot(): 拷贝瀑布到 PSRAM 快照, 唤醒解码任务
            │
            ▼  ft8_dec_task(独立任务)
        ftx_find_candidates() → ftx_decode_candidate() → ftx_message_decode()
            │
            ├─► ft8_rx_log(无锁环形) ──► LCD 解码列表页
            └─► qso_rx_publish() ──► s_qso_q 队列 ──► ft8_qso_task 状态机
```

### 4.2 发射（TX）链路

```
ft8_tx_task(每轮读 cfg.tx, 支持运行中热切换)
   │
   ▼  tx_wave_refresh(): 文本 → ftx_message_encode → ftx_add_crc
   │        → encode174(LDPC) → ft8_encode/ft4_encode → 8-GFSK 波形(PSRAM)
   │
   ▼  找到下一个"本台奇偶时隙"的目标时刻(UTC 栅格)
   │        先 vTaskDelay 长睡, 再忙等到点(busy-wait, 误差 <1ms)
   ▼  wm8978_i2s_write() 整窗写入 I2S0 TX → WM8978 DAC → 电台
```

### 4.3 一次自动通联（QSO）

```
主叫模式: CQ ──► 等对方回答(点我方呼号+网格) ──► 我方 REPORT
             ──► 等对方 R 报告 ──► 我方 RR73 ──► 等对方 73 ──► 完成

应答模式: 听到陌生 CQ ──► 我方 CALL ──► 等对方 REPORT
             ──► 我方 R 报告 ──► 等对方 RR73/RRR ──► 我方 73 ──► 完成

引擎不直接操作音频: 它把每个阶段要发的内容写进 cfg.tx / tx_enable / tx_slot_parity,
由 ft8_tx_task 在下一个本台时隙自动发射(复用"热切换"机制)。
完成一次 QSO → 回调 qso_log_on_qso() → 写 ADIF。
```

### 4.4 FT8/FT4 时序参数

| | FT8 | FT4 |
|---|---|---|
| 时隙 | 15.0 s | 7.5 s |
| 符号数 | 79 | 105 |
| 符号周期 | 0.16 s | 0.048 s |
| 有效波形 | 12.64 s | 5.04 s |
| 调制 | 8-GFSK（tone 间距 6.25Hz） | 同 |

时隙栅格对齐到 UTC `:00/:15/:30/:45`（由 GPS PPS 校准）；无 GPS 时回退到以上电时刻为起点的本地栅格（仅供测试）。

---

## 5. 数据存储结构

### 5.1 分区表（`partitions-16MiB.csv`）

| Name | Type | Offset | Size | 用途 |
|---|---|---|---|---|
| `nvs` | data/nvs | 0x9000 | 24KB | 系统 NVS |
| `phy_init` | data/phy | 0xf000 | 4KB | RF 校准 |
| `factory` | app/factory | 0x10000 | ~2MB | 应用程序 |
| `vfs` | data/fat | 0x200000 | **10MB** | FAT 分区：`log.txt` + `cfg.txt`，经 USB 暴露 |
| `storage` | data/spiffs | 0xC00000 | 4MB | SPIFFS（预留） |

### 5.2 存储链路

```
vfs 分区 → wl_mount()(Wear-Leveling, 扇区 = CONFIG_WL_SECTOR_SIZE; USB 场景须为 512B)
        → tinyusb_msc_new_storage_spiflash()(FAT 挂到 /storage, 应用侧可读写)
        → tinyusb_driver_install()(原生 USB 枚举成 U 盘)

互斥: 主机挂载 U 盘时 MSC 把文件系统切给 USB, 应用侧 VFS 不可用;
      每次写文件用 open/append/close, 失败只告警不阻塞。
```

### 5.3 `/storage/log.txt` — ADIF 日志

每条 QSO 一行，标准 ADIF 文本，以 `<eor>\n` 结尾：

```
<call:6>BG5ABC <QSL_RCVD:1>N <QSL_MANUAL:1>N <gridsquare:4>PM01 <mode:3>FT8 \
<rst_sent:3>-12 <rst_rcvd:3>-10 <qso_date:8>20260915 <time_on:6>120000 \
<qso_date_off:8>20260915 <time_off:6>120130 <band:3>40m <freq:9>7.074000 \
<station_callsign:7>BG7ZJW <my_gridsquare:4>JO70 <comment:32>Distance: 1234 km, QSO by ESP32-FT8 <eor>
```

写入点：`main/qso_log.c` 的 `qso_log_on_qso()`（由 `ft8_app` 的 QSO 完成回调触发）。
读取：LCD 日志页用 `qso_log_tail()` 解析摘要 + 按偏移读原文。

### 5.4 `/storage/cfg.txt` — 持久化配置

自动生成，`key=value` 文本，仅持久化部分字段（`main/qso_log.c` 的 `cfg_store_save/load`）：

```ini
# FT8 cfg (auto generated)
callsign=BG7ZJW
grid=JO70
band=40m
qso_freq_mhz=7.074000
protocol=1              # 0=FT4, 1=FT8
usb_mount_enable=1      # 是否把日志分区作为 U 盘挂载
utc_enable=1            # 是否按 UTC 对齐时隙
gps_utc_enable=1        # 用 GPS 的 UTC 时间/日期
gps_use_pps=0           # 是否用 PPS 精对齐
tx_slot_parity=0        # 0=偶时隙发, 1=奇时隙发
tx_delay_ms=500
audio_level=0.80
hp_vol=50
rx_parse_ms=100
rx_time_osr=2
rx_freq_osr=2
```

> ⚠️ 擦除 `vfs` 分区会连带删掉 `cfg.txt`，`usb_mount_enable` 等会回落到 `main.c` 的默认值。

### 5.5 运行时配置结构体

`ft8_app_config_t`（`main/ft8_app.h`）是全模块共享的**唯一配置源**，定义为 `main.c` 里的 `static ft8_app_config_t cfg`，以**指针引用**传给 `ft8_app`（不拷贝）。

---

## 6. 参考 ft8_lib

`components/ft8_lib` 移植自 [Karlis Goba 的 ft8_lib](https://github.com/kgoba/ft8_lib)，是 WSJT-X FT8/FT4 的 C 语言实现。

**关键点**：解码器吃的不是 PCM，而是 **6.25Hz/符号分辨率的 waterfall 幅度谱**。`common/monitor` 负责把音频转成瀑布（STFT），输入是 12000Hz、200~3000Hz 频段的信号。

**已移植的文件**：

| 目录 | 文件 | 作用 |
|---|---|---|
| `ft8/` | `encode.c` | 文本 → 音调序列（含 CRC/LDPC/8-GFSK 映射） |
| | `decode.c` | `ftx_find_candidates` + `ftx_decode_candidate`（核心解码） |
| | `message.c` | 消息打包/解包（`ftx_message_encode/decode`） |
| | `ldpc.c` / `crc.c` | LDPC 纠错 / CRC 校验 |
| | `text.c` | 呼号/网格文本处理 |
| | `constants.c` | LDPC/奇偶校验表（已编译进代码，无需 `.dat`） |
| `fft/` | `kiss_fft.c` / `kiss_fftr.c` | 供 monitor 做 STFT |
| `common/` | `monitor.c` | 音频 → waterfall（DSP 前端） |

**典型调用**：

```c
// 接收
monitor_config_t mc = { .f_min=200, .f_max=3000, .sample_rate=12000,
                        .time_osr=2, .freq_osr=2, .protocol=FTX_PROTOCOL_FT8 };
monitor_init(&mon, &mc);
monitor_process(&mon, pcm_float_frame);      // 逐符号喂
// 一个时隙后:
ftx_find_candidates(&mon.wf, max_candidates, cand, &num);
for (i...) ftx_decode_candidate(&mon.wf, &cand[i], ldpc_iterations, &msg);

// 发射
ftx_message_encode(&msg, callsign, text);    // 文本 → 消息
// ... ftx_add_crc → encode174 → ft8_encode/ft4_encode → 8-GFSK 波形
```

**裁剪说明**：`common/audio.c`、`wave.c` 是 PC 端用的，已剔除；音频输入由 ESP32 I2S 提供。`message.c` 用到 `stpcpy`，编译期需要 `-D_GNU_SOURCE -DHAVE_STPCPY`（见 `components/ft8_lib/CMakeLists.txt`）。

---

## 7. 编译、烧录与调试

```bash
# 1. 激活 ESP-IDF 环境(或使用 .devcontainer 的 espressif/idf 镜像)
. $IDF_PATH/export.sh

# 2. 设置目标(首次)
idf.py set-target esp32s3

# 3. 编译 / 烧录 / 监视
idf.py build
idf.py -p COMxx flash monitor
```

**从干净状态重来**（例如改了 Wear-Leveling 扇区大小或分区）：

```bash
# 只擦日志分区(推荐, 不动 NVS/应用)
esptool.py erase_region 0x200000 0xA00000
# 或全擦
idf.py erase-flash
```

### 关键 sdkconfig 项

| 配置 | 值 | 说明 |
|---|---|---|
| `CONFIG_IDF_TARGET` | `esp32s3` | 目标芯片 |
| `CONFIG_ESPTOOLPY_FLASHSIZE` | `16MB` | Flash 容量 |
| `CONFIG_SPIRAM` | `y` | 启用 PSRAM（解码快照/波形缓存） |
| `CONFIG_WL_SECTOR_SIZE` | **`512`** | Wear-Leveling 逻辑扇区大小；IDF 默认是 `4096`，**本工程为 512**|
| `CONFIG_FATFS_SECTOR_512` | `y` | 与 WL 扇区保持一致（512） |
| `CONFIG_TINYUSB_MSC_ENABLED` | `y` | 启用 MSC |
| `CONFIG_TINYUSB_MSC_BUFSIZE` | `4096` | MSC FIFO，**必须 ≥ WL 扇区大小**，否则创建存储会失败 |
| `CONFIG_TINYUSB_MSC_MOUNT_PATH` | `/storage` | FAT 挂载点 |
| `CONFIG_ESP_CONSOLE_UART_DEFAULT` | `y` | 控制台走 UART0/CH340 |

---

## 8. 开发指南

### 8.1 想改哪里？

| 目标 | 位置 |
|---|---|
| 界面 / 按键交互 | `main/main.c` 的 `draw_page_*` + `my_key_callback` |
| 收发时序 / 解码参数 | `main/ft8_app.c` 的 `ft8_rx_task` / `ft8_tx_task` / `rx_decode_snapshot` |
| 自动通联逻辑 | `main/ft8_app.c` 的 QSO 引擎段 + `main/ft8_qso.h` |
| 日志 / 存储 / USB | `main/qso_log.c` |
| 音频编解码硬件 | `components/BSP/wm8978/` |
| GPS / 授时 | `components/BSP/GPS/` |
| 显示驱动 | `components/BSP/LCD_st7735/` |
| 编解码算法 | `components/ft8_lib/` |

### 8.2 如何新增一个配置项

需要**同步改 4 处**：

1. `main/ft8_app.h` — 在 `ft8_app_config_t` 里加字段。
2. `main/ft8_app.c` — 在 `ft8_app_config_default()` 里设默认值，并在使用处读取。
3. `main/main.c` — 在 `s_ci[]` 表里加一行（可选，决定它是否出现在设置页）。
4. `main/qso_log.c` — 在 `cfg_store_save()` / `cfg_store_load()` 里加对应 key（可选，决定是否持久化到 `cfg.txt`）。

### 8.3 约定与坑

- **`cfg` 是引用共享**：`ft8_app_start(&cfg)` 只保存指针，任务持续读它。运行中直接改 `cfg.tx.*` 即可热切换发射内容（下一个本台时隙生效）。
- **自动 QSO 引擎会接管 `cfg.tx / tx_enable / tx_slot_parity`**（`qso.enable=1` 时），手动设置会被覆盖。
- **瀑布/解码日志是无锁环形缓冲**（`ft8_wf_snap_t`、`ft8_rx_log_t`），读端可能看到正在写的一行，设计上接受。
- **两套时隙栅格**：GPS UTC 校准 vs 本地 `esp_timer`；`utc_enable`/`gps_utc_enable`/`gps_use_pps` 三个开关组合决定用哪套。
- **`usb_mount_enable` 门控 USB**：只有它为真时 `main.c` 才调 `qso_log_usb_start()`；该值可被 `cfg.txt` 覆盖，擦分区后回落到默认值。
- **解码任务栈很大（24KB/48KB）**：monitor FFT 需要，别随意调小。
- **不要用 `FM_SFD` 之外的假设**：FAT 由 FatFs 自动格式化（MBR + 单分区），一般无需手动格式化。
- **代码注释/文档以中文为主**，且每个跨文件数据接缝都有"数据流"示意图（`qso_log.c`、`ft8_app.c`、`ft8_app.h` 顶部），是最好的阅读入口。
- **编码**：源码为 UTF-8；Windows 终端若显示乱码属终端编码问题，不影响编译。

### 8.4 上电初始化顺序（`app_main`）

```
key_init() → gps_init()
ft8_app_config_default(&cfg) → 手工覆盖各项
ft8_app_set_qso_callback(qso_log_on_qso)
qso_log_init()            // 挂载 FAT(vfs) 到 /storage
  └ cfg_store_load(&cfg)  // 用 cfg.txt 覆盖持久化字段
  └ if (cfg.usb_mount_enable) qso_log_usb_start()   // 启动 USB MSC
xTaskCreate(gps_utc)
ft8_app_start(&cfg)       // WM8978+I2S 初始化, 建 RX/TX/DEC/QSO 任务
xTaskCreate(LCD) / xTaskCreate(rgb_led)
```

---

## 9. 参考

- FT8/FT4 协议与 `ft8_lib`：https://github.com/kgoba/ft8_lib
- ESP-IDF 编程指南：https://docs.espressif.com/projects/esp-idf/
- esp_tinyusb（MSC 存储）：`managed_components/espressif__esp_tinyusb/README.md`

