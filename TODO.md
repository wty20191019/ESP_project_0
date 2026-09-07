# TODO 待办事项

本文档记录了项目中需要修复和优化的问题，按优先级分类。

---

## 🔴 高优先级 - 严重问题

### 1. GPS 模块数组访问线程安全问题

**位置**：`components/BSP/GPS/gps.c:481-490`

**问题描述**：
在 `handle_gsa()` 函数中，存在竞态条件风险：
```c
if (slot < 0 && g.sys_count < GPS_SYS_MAX) {
    slot = (int)g.sys_count++;
}
if (slot >= 0) {
    g.sys[slot].system_id = sysid;
    // ...
}
```

虽然在 `commit()` 内部有互斥锁保护，但 `handle_gsa()` 中直接修改快照，没有锁保护。在多线程环境下，`g.sys_count` 的递增和数组访问之间可能存在竞态条件。

**影响范围**：
- GPS 模块在处理 GSA 语句时可能出现数据不一致
- 可能导致数组越界访问
- 在高频 GPS 数据更新场景下问题更明显

**修复建议**：
```c
// 在 handle_gsa 中添加锁保护
xSemaphoreTake(s_mutex, portMAX_DELAY);
if (slot < 0 && g.sys_count < GPS_SYS_MAX) {
    slot = (int)g.sys_count++;
}
if (slot >= 0) {
    g.sys[slot].system_id = sysid;
    // ...
}
xSemaphoreGive(s_mutex);
```

**预计工作量**：1-2 小时

---

### 2. 时区转换函数边界条件处理不够清晰

**位置**：`main/main.c:91-95`

**问题描述**：
时区转换函数的跨日进位逻辑嵌套过深，不够清晰：
```c
if (carry > 0) {                 /* 跨日进位 */
    d++;
    if (d > days_in_month(y, m)) { d = 1; if (++m > 12) { m = 1; y++; } }
}
```

虽然嵌套的 `if (++m > 12) { m = 1; y++; }` 处理了年份跨越，但逻辑不够清晰，容易出错。

**影响范围**：
- 时区转换可能在特定边界条件下出错
- 代码维护困难，容易出现新的 bug

**修复建议**：
```c
if (carry > 0) {
    d++;
    if (d > days_in_month(y, m)) {
        d = 1;
        m++;
        if (m > 12) {
            m = 1;
            y++;
        }
    }
}
```

**预计工作量**：0.5 小时

---

## 🟡 中优先级 - 中等问题

### 3. LCD 帧缓冲内存分配无释放

**位置**：`components/BSP/LCD_st7735/lcd.c:20`

**问题描述**：
帧缓冲分配后永不释放：
```c
static uint8_t *s_fb = NULL;
```
在 `lcd_init()` 中分配，但没有对应的去初始化函数释放内存。

**影响范围**：
- 对于内存紧张的嵌入式系统（特别是 PSRAM 较小的版本），40KB 的内存占用可能有问题
- 如果需要重新初始化 LCD，会内存泄漏

**修复建议**：
1. 在 `LCD_Deinit()` 函数中添加释放代码：
```c
void LCD_Deinit(void) {
    if (s_fb != NULL) {
        free(s_fb);
        s_fb = NULL;
    }
    // 其他清理代码...
}
```

2. 或者在项目文档中明确说明此设计决策（LCD 永久驻留内存）

**预计工作量**：1 小时

---

### 4. 静态初始化标志重复使用

**位置**：`main/main.c:251-258, 281-287`

**问题描述**：
使用静态变量 `res` 作为初始化标志：
```c
static void LCD_task(void *arg) {
    static uint8_t res = 0;
    if (res == 0) { /* ... */ res = 1; }
}
```

如果任务被删除重建，静态变量不会被重置，导致初始化代码不执行。

**影响范围**：
- 任务重启后可能未正确初始化
- 系统状态不一致

**修复建议**：
使用全局初始化状态管理，或者在 `app_main` 中统一初始化：
```c
// 在 app_main 中统一初始化
void app_main(void) {
    LCD_Init();
    Key_Init(key_callback, NULL);
    LED_Init();

    // 创建任务
    xTaskCreate(LCD_task, "LCD", 4096, NULL, 5, NULL);
    xTaskCreate(Key_task, "KEY", 3072, NULL, 5, NULL);
    // ...
}
```

**预计工作量**：2-3 小时

---

### 5. 编译器优化过于激进

**位置**：`components/BSP/CMakeLists.txt:7`

**问题描述**：
```cmake
component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)
```

**问题分析**：
- `-O3` 优化可能导致不可预期的行为，特别是在涉及硬件寄存器访问和中断处理的代码中
- `-ffast-math` 可能影响浮点运算精度，需确认是否可接受
- 禁用了格式字符串警告（`-Wno-format`），可能隐藏潜在的格式化 bug

**影响范围**：
- 可能导致调试困难（优化后代码与源码对应关系复杂）
- 可能引入难以复现的 bug
- 浮点运算精度可能不符合预期

**修复建议**：
1. 对于关键驱动代码使用 `-O2` 代替 `-O3`
2. 评估 `-ffast-math` 的必要性，如不需要则移除
3. 重新启用格式字符串警告，修复真正的警告而非禁用
```cmake
# 建议配置
component_compile_options(-O2 -Wformat)
```

**预计工作量**：1 小时 + 测试时间

---

## 🟢 低优先级 - 轻微问题/建议

### 6. 组件依赖声明冗余

**位置**：`components/BSP/CMakeLists.txt:4`

**问题描述**：
```cmake
REQUIRES driver esp_driver_gpio led_strip esp_driver_i2c esp_driver_i2s esp_rom esp_driver_uart esp_timer freertos log esp_driver_spi
```

`driver` 已经包含了大部分 ESP 驱动，重复声明显得冗余。

**影响范围**：
- CMake 配置文件可读性降低
- 维护困难（需要同步修改多个地方）

**修复建议**：
精简依赖列表，只显式声明必要的组件：
```cmake
REQUIRES driver freertos log
```

**预计工作量**：0.5 小时

---

### 7. GPS 模块去初始化时未清理互斥锁

**位置**：`components/BSP/GPS/gps.c:793-816`

**问题描述**：
`gps_deinit()` 函数缺少互斥锁清理：
```c
esp_err_t gps_deinit(void) {
    // ... 其他清理 ...
    if (s_pps_q) {
        vQueueDelete(s_pps_q);
        s_pps_q = NULL;
    }
    // 缺少: if (s_mutex) { vSemaphoreDelete(s_mutex); s_mutex = NULL; }
}
```

**影响范围**：
- 资源泄漏
- 如果多次调用 `gps_init()`/`gps_deinit()`，会创建多个互斥锁

**修复建议**：
```c
esp_err_t gps_deinit(void) {
    // ... 其他清理 ...
    if (s_pps_q) {
        vQueueDelete(s_pps_q);
        s_pps_q = NULL;
    }
    if (s_mutex) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }
    return ESP_OK;
}
```

**预计工作量**：0.5 小时

---

### 8. 代码注释语言不统一

**位置**：多个文件

**问题描述**：
- 大部分代码使用中文注释
- 但有些地方使用英文注释（如 LCD 模块）

**影响范围**：
- 代码可读性降低
- 维护困难

**修复建议**：
统一使用中文注释以保持一致性。

**预计工作量**：2-3 小时

---

## 📋 测试相关

### 9. 添加单元测试

**问题描述**：
项目缺少自动化测试，主要依赖手动测试。

**建议添加的测试**：
- GPS 模块 NMEA 解析测试
- 时区转换函数边界条件测试
- FT8/FT4 编解码正确性测试
- LCD 绘图功能测试

**预计工作量**：1-2 天

---

## 🔧 代码质量改进

### 10. 添加代码风格检查

**建议**：
- 配置 `.clang-format` 统一代码风格
- 配置 `.clang-tidy` 进行静态分析
- 在 CI/CD 中集成检查

**预计工作量**：1 天

---

### 11. 添加内存安全检查

**建议**：
- 启用 GCC 的 AddressSanitizer（在调试模式下）
- 添加内存泄漏检测
- 添加栈溢出检测

**预计工作量**：0.5 天

---

## 📚 文档改进

### 12. 添加 API 文档

**建议**：
- 使用 Doxygen 生成 API 文档
- 为所有公共函数添加详细注释
- 添加使用示例

**预计工作量**：2-3 天

---

### 13. 添加架构设计文档

**建议**：
- 绘制系统架构图
- 添加模块依赖关系图
- 添加数据流图

**预计工作量**：1 天

---

## 🎯 性能优化

### 14. 优化 LCD 刷新性能

**建议**：
- 评估是否可以使用 DMA 传输优化
- 考虑局部刷新而非全屏刷新
- 添加双缓冲机制减少闪烁

**预计工作量**：1-2 天

---

### 15. 优化 GPS 解析性能

**建议**：
- 评估是否可以使用查找表优化解析
- 考虑使用更高效的字符串处理函数
- 添加解析结果缓存

**预计工作量**：1 天

---

## 📊 问题统计

| 优先级 | 数量 |
|--------|------|
| 🔴 高优先级 | 2 |
| 🟡 中优先级 | 3 |
| 🟢 低优先级 | 3 |
| 📋 测试相关 | 1 |
| 🔧 代码质量 | 2 |
| 📚 文档改进 | 2 |
| 🎯 性能优化 | 2 |
| **总计** | **15** |

---

## 📝 修复建议

### 优先修复顺序（按影响和风险）：

1. **立即修复**（影响功能稳定性）：
   - #1 GPS 模块数组访问线程安全问题
   - #2 时区转换函数边界条件处理

2. **短期内修复**（影响代码质量）：
   - #3 LCD 帧缓冲内存分配无释放
   - #4 静态初始化标志重复使用
   - #7 GPS 模块去初始化时未清理互斥锁

3. **中期优化**（提升代码质量）：
   - #5 编译器优化过于激进
   - #6 组件依赖声明冗余
   - #8 代码注释语言不统一

4. **长期改进**（提升项目可维护性）：
   - #9-15 测试、文档、性能优化

---

**最后更新时间**：2026-09-07
**审查版本**：基于当前未提交的更改
