# AGENTS.md — AI 助手标准指引（ESP32-S3 FT8/FT4 终端固件）

> 本文件面向在本仓库中工作的 AI 助手与协作者。所有说明使用简体中文。
> 修改代码前先通读本文件与对应模块源码，遵循既有约定，尽量少改动。

## 一、项目总览

基于 **ESP32-S3（带 PSRAM）+ WM8978 音频编解码器** 的 FT8/FT4 业余无线电数字模式终端固件。

- 语言：纯 C（C11 风格），构建框架：ESP-IDF（目标 `esp32s3`）。
- 通信：WM8978 经 I2S 全双工 12kHz/16bit，8-GFSK 调制收发 FT8(15s 时隙)/FT4(7.5s)。
- 周边：1.8 寸 ST7735 TFT(128×160, SPI+DMA)、7 键 GPIO 扫描、WS2812 状态灯(GPIO48)。
- 顶层功能由 `main/ft8_app.c` 的双任务（RX/TX 分置双核）驱动，配置全部收敛到 `ft8_app_config_t` 结构体。
- 协议编解码移植自开源 [kgoba/ft8_lib](https://github.com/kgoba/ft8_lib)（第三方代码）。

### 目录职责速查

| 路径 | 职责 | 归属 |
|------|------|------|
| `main/` | 入口 `app_main`、应用控制器 `ft8_app`（RX/TX 任务调度） | 本仓自研 |
| `components/BSP/` | `wm8978`(I2C 逻辑层+I2S)、`key`、`led` 板级驱动 | 本仓自研 |
| `components/LCD_1.8_st7735/` | ST7735 SPI+DMA 底层与绘图 API | 移植自 STM32 例程，已 ESP 化 |
| `components/ft8_lib/` | `ft8/`(编解码/LDPC/CRC/message)、`fft/`(kiss_fft)、`common/`(monitor 瀑布) | 第三方移植，基本原样 |
| `partitions-16MiB.csv` | 16MB Flash 分区表 | 构建约束 |

## 二、架构与设计模式

1. **ESP-IDF 组件模型**：每个模块是独立 component，自带 `CMakeLists.txt`，通过 `REQUIRES` 声明依赖，头文件经 `INCLUDE_DIRS` 暴露。不要跨目录乱 `#include`。
2. **任务 = 并发单元**：FreeRTOS 任务固定双核（`xTaskCreatePinnedToCore`）。
   - RX 任务：Core 1，栈 48KB（monitor FFT 需大栈），持续读 I2S → 瀑布累积 → 每时隙整窗解码（去重）。
   - TX 任务：Core 0，栈 8KB，PSRAM 预生成 GFSK 波形，`esp_timer` 忙等对齐时隙起播。
3. **集中配置结构体**：`ft8_app_config_t`（`main/ft8_app.h`）＋ `ft8_app_config_default()` 填默认值，调用方逐字段覆写；配置在 `ft8_app_start()` 内被整体复制。
4. **事件回调注册**：底层驱动不感知业务，如按键 `key_init(callback, user_data)`，由上层注册回调消费事件。
5. **软件寄存器缓存表**：WM8978 的 I2C 只写不可读，驱动内维护 58 个寄存器软件缓存。
6. **状态前置下划线命名 + volatile 跨任务标记**：模块级全局用 `s_` 前缀；任务间共享状态加 `volatile`。

## 三、整洁架构分层

依赖方向保持**单向、自顶向下**，禁止反向与环依赖：

```
main（应用层：ft8_app 调度 / 组装配置）
   ↓ REQUIRES
components/BSP（板级驱动：wm8978 / key / led）   ← 只依赖 ESP-IDF driver
components/LCD_1.8_st7735（显示驱动）             ← 只依赖 ESP-IDF driver
components/ft8_lib（纯计算协议库）                ← 不依赖 BSP/LCD/任何业务
```

- `ft8_lib` 是**无外设的纯算法库**：输入输出只有内存缓冲，绝不允许在其中调用 I2S/I2C/GPIO/日志之外的业务。
- 驱动层（BSP/LCD）只暴露硬件能力，不写协议逻辑；业务逻辑只出现在 `main/`。
- `main.c` 只做：初始化各驱动 + 填写配置 + 启动任务 + 注册回调，不内联业务算法。

## 四、模块隔离约束

- **组件边界靠 CMake 而非习惯**：新增跨组件调用必须显式加到对方组件的 `REQUIRES`，并只 include 其公开头文件。
- 公开接口以头文件为准；`.c` 内的函数一律 `static`，不对外暴露。
- 移植代码（`ft8_lib/`、`LCD_1.8_st7735/` 底层）保持**原样可追溯**：
  - 不重排、不翻译、不重构第三方代码（含其英文/`///` 注释、`kiss_fft` 与 `ft8` 内部）。
  - 确实需要改动时，用中文注释标明"相对上游的改动点"，并控制到最小。
- 版本/引脚等平台事实只在一个头文件中定义一次，改动只动源头（详见第七节"坑点"）。
- 注释里的文档（如 README、模块头部注释）若与代码事实不符，改代码时顺手同步修正，避免遗留误导。

## 五、构建与平台参数

| 项 | 值 |
|----|----|
| 芯片 / 目标 | ESP32-S3（支持 PSRAM） |
| SDK | ESP-IDF v5.0+（实测 v6.1.0，`.vscode/settings.json` 指向 `D:\ESP\.espressif\v6.1\esp-idf`） |
| Flash | 16MiB，分区表 `partitions-16MiB.csv`（factory 1.9MB + vfs FAT 10MB + storage SPIFFS 4MB） |
| 烧录 | JTAG（`idf.portWin=COM19`），也可 `idf.py flash` |
| 组件依赖 | 外部仅 `espressif/led_strip ^3.0.3`（`main/idf_component.yml`） |
| 编译选项 | `ft8_lib`: `-O2`；`BSP`: `-O3 -ffast-math`；`main` 依赖 `esp_timer` |
| 常用命令 | `idf.py set-target esp32s3 && idf.py build && idf.py flash`；问题排查先 `idf.py fullclean` |

## 六、工程开发原则

**最高优先级约束：尽量少改动。**

1. **优先删除代码，而非新增代码**：修复/重构优先考虑删掉多余逻辑，而不是叠补丁。
2. **保守平淡优于炫技**：新代码要与周边风格一致，用最直接的写法，不引入新库/新抽象/宏魔法。
3. **尽量少改动文件**：一个需求先判断能否在局部解决；diff 越小越好。
4. **照抄现有范式**：新增一个驱动/模块时，先读同层已有代码（如 `key.c` 或 `led.c`），结构、命名、注释、错误处理保持一致。
5. **接口先行，文档同步**：公共头文件（如 `xxx.h`）先写好接口注释（`@brief/@param/@return`）再写实现。
6. **动手前确认**：模糊的交互行为（UI、按键、协议时序）先列方案与用户对齐，不要自行假设。

### 代码风格基线（从仓库现状归纳）

- 缩进 4 空格；花括号沿用**函数体大括号独立成行**、控制块多数另起一行的 Allman 风格；短 `for`/`if` 允许单行紧凑体，保持与所在文件一致。
- 命名：函数/变量小写蛇形；类型 `xxx_t`；枚举成员与宏全大写（如 `KEY_ID_UP`、`KEY_EVENT_CLICK`）；模块级静态加 `s_` 前缀；公共函数带模块前缀（`ft8_app_*`/`wm8978_*`/`key_*`）；LCD 沿用历史大写 `LCD_ShowString`。
- 头文件保护两者并存，新建文件就近选择同层风格（`#pragma once` 或 `#ifndef __XX_H__`），不要在同层制造第三种。
- 注释用中文（自研代码），`/* ... */` 块注释划分小节；关键业务逻辑必须有注释。
- 日志用 `ESP_LOGx`，本文件 `TAG` 为模块名小写（如 `ft8_app`/`key`/`main`）；打印 `int64_t/float` 注意显式转型（`(long long)`/`(double)`）并用对应格式符。
- 错误处理沿用 `esp_err_t` 返回值或 `ESP_ERROR_CHECK`，与所在模块一致。

## 七、Bug 修复准则

1. **先复现、定位根因，再改**：改动前确认现象（串口日志/按键/显示），追到最小根因；禁止"猜一个可能原因就改"。
2. **最小修复**：只改根因所在点，不加多余防御；优先删问题代码而非堆处理。
3. **修复后自查关联**：
   - 改时隙/时序逻辑 → 检查 RX 与 TX 双任务是否会互相干扰、DMA 残余是否被补零。
   - 改 WM8978 寄存器 → 牢记 I2C 只写，寄存器缓存表需同步。
   - 改跨任务共享变量 → 需要 `volatile`/同步手段，防止读脏。
   - 改任务栈/缓冲大小 → 对照现有 `#define`（如 `STACK_RX`、`MAX_SYMBOL_SAMPLES`）与 PSRAM 分配路径。
4. **不要动移植库的算法内核**：编解码/LDPC/FFT 行为异常优先怀疑调用参数、缓冲、时序与内存分配，不直接改算法。
5. **回归验证**：嵌入式无自动测试时，至少保证 `idf.py build` 编译通过、与现有默认配置下行为不回退；验证手段如实记录。

## 八、路线规划（已知缺口，供新功能定位）

- RX/TX 收发与解码主链路已通；下面多为"占位/未完成"点：
  - `main.c` 的 `LCD_task`/`rgb_led_task` 仍是演示用（刷色轮播/呼吸），不是真实界面。
  - 解码呼号 hash 回填为空（`ft8_app.c` 的 `hash_lookup`/`hash_save`），压缩呼号不可读，但影响解码成功与否，需再接表。
  - UTC 亚秒相位未做：`time()` 只有秒级，真正对齐 UTC `:00/:15/:30/:45` 需毫秒级墙钟（SNTP 只解决粗对齐）。
  - 无 PTT 控制、无 UI 菜单、无参数持久化（NVS/FAT 分区已留，未接入）。
- 各分支命名 `F_xxx` 合并进 `main_debug` 再并入 `main`，遵循已有工作流。

## 九、注意坑点（平台与代码红线）

1. **GPIO 硬件约束**（参考 `board_pins.txt`）：
   - GPIO0/3/45/46 为 Strapping 脚，复位瞬间决定启动模式，按键/输出别在复位期拉死。
   - GPIO43/44 默认接 UART0（CH340 调试口），复用会丢日志。
   - 若芯片是 N16R8**V**（VDD_SPI=1.8V），GPIO47/48 为 1.8V 电平，不能直接驱 3.3V 逻辑。
   - WM8978 板若未外接 I2C 上拉，需把 `WM8978_I2C_FREQ_HZ` 降到 100kHz。
2. **WM8978 的 I2C 只能写不能读**：必须靠软件寄存器缓存表，禁止假设回读结果。
3. **编译优化差异**：BSP 带 `-O3 -ffast-math`（fast-math 会放宽浮点语义）；ft8_lib 带 `-O2`。涉及严格 IEEE 浮点的代码注意所在组件的编译选项。
4. **任务栈敏感**：RX 48KB、TX 8KB 是实测调好的值；函数内避免超大栈上数组（如 `LCD_Fill` 内部约 320B 栈数组），新建任务先核对栈余量。
5. **内存分配路径**：大缓冲（TX 波形等）优先 `heap_caps_malloc(MALLOC_CAP_SPIRAM)`，不足回退内部 RAM；不要默认 `malloc` 大块。
6. **时隙对齐**：以 `esp_timer` 本地栅格为权威；所有"发射时刻"必须用忙等/`esp_timer` 而非 `vTaskDelay` 长睡（精度 <1ms）。
7. **I2S TX 静音**：DMA 环存在残余，发射结束后必须补一整块全零冲刷，否则残余会被循环重放。
8. **文档与代码可能脱节**：若干历史注释仍写"单任务"等旧描述，以 `main/ft8_app.c` 实际双任务实现为准；发现问题顺手改注释。
9. **不要全局动第三方代码**：`components/ft8_lib` 与 `LCD_1.8_st7735` 底层尽量保持与上游一致，改动需加中文标注。

## 十、AI Copilot 工作模式

1. 默认只输出代码变更；解释性文字放在对话聊天区，不写进代码注释以外的文档。
2. 需要新增文档时，先询问用户确认，不要自行新建。
3. **禁止自动创建任何 `.md` 文档**，除非用户明确要求；不要自动添加 README、教程、迁移文档。
4. diff 尽量小，聚焦当前实现需求，不做顺手重构。
5. 默认只做静态检查；**不会主动执行编译 / 测试命令**（`idf.py build`/`flash` 等），除非用户明确要求。
