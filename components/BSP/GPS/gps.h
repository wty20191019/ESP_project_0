#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* ============================================================
 * GPS / GNSS 模块 (UART + NMEA 解析 + PPS 秒脉冲)
 *
 * 硬件接线(默认):
 *   GNSS_TXD -> GPIO17   (ESP32-S3 UART TX -> GNSS RX)
 *   GNSS_RXD <- GPIO18   (ESP32-S3 UART RX <- GNSS TX)
 *   PPS_GPIO -> GPIO9    (GNSS 秒脉冲 PPS, 检测上升沿)
 *
 * 说明:
 *   1) NMEA 0183 异步解析(后台任务), 支持 GGA/RMC/GLL/GSA/VTG/ZDA/TXT,
 *      多星座机(GN/GP/GL 等前缀)通用。
 *   2) 解析结果合并到内部快照结构体 gps_info_t, 调用方可通过
 *      gps_get_info() 随时取一致性快照。
 *   3) PPS 上升沿走 GPIO 中断: 中断里记录 esp_timer 时间戳,
 *      再投递给专用任务调用注册回调, 用于本地时钟/时隙对齐。
 *
 * 使用示例:
 *   gps_set_pps_callback(my_pps_cb, NULL);
 *   gps_init();
 *   for (;;) { gps_info_t g; gps_get_info(&g); ... }
 * ============================================================ */

/* ---------- 引脚/串口定义(可先于本头文件覆盖) ---------- */
#ifndef GNSS_TXD
#define GNSS_TXD        GPIO_NUM_17   /* S3 TX -> GNSS RX */
#endif
#ifndef GNSS_RXD
#define GNSS_RXD        GPIO_NUM_18   /* S3 RX <- GNSS TX */
#endif
#ifndef PPS_GPIO
#define PPS_GPIO        GPIO_NUM_9    /* PPS 上升沿检测 */
#endif

#ifndef GPS_UART_NUM
#define GPS_UART_NUM    1             /* 使用 UART1 */
#endif
#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD   9600          /* 多数 GNSS 模块默认波特率 */
#endif

/* ---------- 辅助类型 ---------- */
#define GPS_SYS_MAX      6              /* 最多跟踪的星座(系统)数 */
#define GPS_SV_PER_SYS   12             /* 每个星座(GSA)最多卫星数 */

/* 天线状态(GPTXT 上报) */
typedef enum {
    GPS_ANT_UNKNOWN = 0,   /* 未知 / 尚未上报 */
    GPS_ANT_OK,            /* 天线正常 */
    GPS_ANT_OPEN,          /* 天线开路 (ANTENNA OPEN) */
    GPS_ANT_SHORT,         /* 天线短路 (ANTENNA SHORT) */
} gps_antenna_state_t;

/* 单个星座(GSA 一条)的参与卫星信息 */
typedef struct {
    uint8_t system_id;     /* 系统 ID: 1=GPS 2=GLONASS 3=Galileo 4=BDS 5=QZSS 6=SBAS... */
    uint8_t fix_type;      /* 该条解算类型: 0/1=无 2=2D 3=3D */
    uint8_t sv_count;      /* 参与定位卫星数量 */
    uint8_t sv[GPS_SV_PER_SYS];  /* 卫星 PRN 列表 */
} gps_sys_t;

/* ---------- 解析结果结构体 ---------- */
typedef struct {
    /* ---- 定位 ---- */
    bool     fix_valid;       /* 定位有效(RMC/GLL 状态 A / GGA quality>0) */
    uint8_t  fix_quality;     /* GGA 定位质量: 0=无效 1=GPS 2=DGPS 3=PPS... */
    uint8_t  fix_type;        /* GSA 定位类型: 0/1=无 2=2D 3=3D */
    uint8_t  satellites;      /* GGA 使用卫星总数 */
    char     mode_ind;        /* 模式指示符: A=自主 D=差分 E=估算 M=手动 N=无效 */
    double   latitude;        /* 纬度(十进制度, 北纬为正) */
    double   longitude;       /* 经度(十进制度, 东经为正) */
    float    pdop;            /* 位置精度因子(GSA) */
    float    hdop;            /* 水平精度因子 */
    float    vdop;            /* 垂直精度因子(GSA) */
    float    altitude_m;      /* 海拔(米, MSL) */

    /* ---- UTC 时间(GGA/RMC/ZDA 提供) ---- */
    bool     time_valid;      /* 时间字段解析成功 */
    bool     date_valid;      /* 日期字段解析成功(RMC/ZDA) */
    uint16_t year;            /* 公元年, 如 2026 */
    uint8_t  month;           /* 1~12 */
    uint8_t  day;             /* 1~31 */
    uint8_t  hour;            /* 0~23 */
    uint8_t  minute;          /* 0~59 */
    uint8_t  second;          /* 0~59 */
    uint16_t millisecond;     /* 0~999 */

    /* ---- 运动 ---- */
    float    speed_kmh;       /* 地面速度(km/h, RMC/VTG) */
    float    course_deg;      /* 对地航向(度, 0~359.9, RMC/VTG) */

    /* ---- 星座细分(GSA, 按系统 ID 去重) ---- */
    uint8_t  sys_count;       /* 实际有数据的星座数 */
    gps_sys_t sys[GPS_SYS_MAX];

    /* ---- 天线状态(GPTXT) ---- */
    gps_antenna_state_t antenna;       /* 天线状态 */
    char     antenna_text[24];         /* 原始文本, 如 "ANTENNA OPEN" */

    /* ---- PPS 秒脉冲(最近一次上升沿, 供对齐使用) ---- */
    uint32_t pps_seq;         /* PPS 上升沿计数(0=尚未收到) */
    int64_t  pps_edge_us;     /* 该上升沿对应的 esp_timer 时间(上电后微秒) */
} gps_info_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 GNSS: 配置 UART1、启动 NMEA 解析任务与 PPS 中断任务。
 *        幂等, 重复调用直接返回 ESP_OK。
 * @return ESP_OK 成功, 否则失败
 */
esp_err_t gps_init(void);

/**
 * @brief 反初始化: 停任务、关闭串口(不卸载共享的 GPIO ISR 服务)。
 * @return ESP_OK
 */
esp_err_t gps_deinit(void);

/**
 * @brief 取当前 GNSS 快照(线程安全, 一致性拷贝)。
 * @param out 输出结构体, 不可为 NULL
 */
void gps_get_info(gps_info_t *out);

/* ---- PPS 对齐回调 ----
 * 在 PPS 任务上下文回调(非中断), 可执行较耗时逻辑。
 * @param edge_boot_us PPS 上升沿时刻(esp_timer, 微秒)
 * @param seq          上升沿序号(从 1 开始)
 * @param gps          回调时刻的 GNSS 快照(含该沿 pps 字段)
 * @param arg          注册回调时传入的参数 */
typedef void (*gps_pps_callback_t)(int64_t edge_boot_us, uint32_t seq,
                                   const gps_info_t *gps, void *arg);

/**
 * @brief 注册 PPS 上升沿回调(用于时间/时隙对齐)。传 NULL 可注销。
 * @param cb  回调函数
 * @param arg 透传参数
 * @return ESP_OK 成功
 */
esp_err_t gps_set_pps_callback(gps_pps_callback_t cb, void *arg);

/* ---- 数据更新回调(可选) ----
 * 每成功解析一条 GGA/RMC 且在任务上下文调用一次。
 * 适用于驱动 UI 刷新等场景。 */
typedef void (*gps_data_callback_t)(const gps_info_t *gps, void *arg);

esp_err_t gps_set_data_callback(gps_data_callback_t cb, void *arg);

/**
 * @brief 打印当前 GNSS 状态到日志(调试用)。
 */
void gps_log_info(void);

#ifdef __cplusplus
}
#endif

#endif
