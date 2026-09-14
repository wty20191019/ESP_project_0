#pragma once

/* QSO 日志模块: 把完成的 QSO 以 ADIF 行追加到 FAT 分区上的 /storage/log.txt,
 * 同时通过 TinyUSB MSC 把该分区作为 U 盘暴露给 PC(挂载期间主机独占读写)。 */
#include "esp_err.h"
#include "ft8_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化日志存储: 挂载 vfs(FAT) 分区; 视 usb_mount 决定是否再把它
 *        通过 TinyUSB MSC 暴露成 U 盘。失败不致命(仅日志不可用), 会打印告警。
 * @param usb_mount true=启动 USB 大容量存储(U盘); false=只本地挂载写日志
 * @return ESP_OK 成功
 */
esp_err_t qso_log_init(bool usb_mount);

/** ft8_app QSO 完成回调: 追加一条 ADIF 记录(签名匹配 ft8_qso_callback_t) */
void qso_log_on_qso(const ft8_qso_record_t *rec, void *arg);

/* ---- 最近 QSO 摘要(RAM 环形, 供 LCD 日志页显示) ---- */
#define QSO_LOG_RAM_MAX 16
/** 取第 idx 条摘要(idx 0 = 最新); 越界返回 NULL */
const char *qso_log_line(int idx);
/** 当前缓存条数 */
int qso_log_lines(void);

/* ---- 直接读取 /storage/log.txt 尾部若干行(供日志页浏览文件内容) ---- */
#define QSO_TAIL_MAX 48

/* 从 ADIF 行解析出的摘要(呼号/网格/信号/频率) */
typedef struct {
    char call[16];
    char grid[8];
    char rst[8];
    char freq[12];
} qso_log_sum_t;

/**
 * @brief 读取 log.txt 最后 max_lines 行到内部缓冲(最多 QSO_TAIL_MAX 行), 并解析摘要。
 *        读失败(如被 USB 主机占用)时保留上次内容。
 * @return 当前缓冲行数
 */
int qso_log_tail(int max_lines);
/** 取已加载尾部行的第 idx 行原文(idx 0 = 最旧); 越界返回 NULL */
const char *qso_log_tail_line(int idx);
/** 取第 idx 行的解析摘要(idx 0 = 最旧); 越界返回 NULL */
const qso_log_sum_t *qso_log_tail_summary(int idx);
/** 已加载尾部行数 */
int qso_log_tail_count(void);

#ifdef __cplusplus
}
#endif
