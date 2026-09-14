#pragma once

/* QSO 日志模块: 把完成的 QSO 以 ADIF 行追加到 FAT 分区上的 /storage/log.txt,
 * 同时通过 TinyUSB MSC 把该分区作为 U 盘暴露给 PC(挂载期间主机独占读写)。 */
#include "esp_err.h"
#include "ft8_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化日志存储: 挂载 vfs(FAT) 分区到 /storage(供读写 log.txt/cfg.txt),
 *        但不启动 USB。启动 USB 请随后按配置调用 qso_log_usb_start()。
 * @return ESP_OK 成功
 */
esp_err_t qso_log_init(void);

/** 启动 USB 大容量存储(把 /storage 分区暴露为 U 盘); 幂等, 失败返回错误码 */
esp_err_t qso_log_usb_start(void);

/** 把存储从 USB 主机收回给应用侧(便于写文件; 会解除 PC 端挂载) */
void qso_log_usb_release(void);

/** ft8_app QSO 完成回调: 追加一条 ADIF 记录(签名匹配 ft8_qso_callback_t) */
void qso_log_on_qso(const ft8_qso_record_t *rec, void *arg);

/* ---- 最近 QSO 摘要(RAM 环形, 供 LCD 日志页显示) ---- */
#define QSO_LOG_RAM_MAX 16
/** 取第 idx 条摘要(idx 0 = 最新); 越界返回 NULL */
const char *qso_log_line(int idx);
/** 当前缓存条数 */
int qso_log_lines(void);

/* ---- 直接读取 /storage/log.txt 尾部行(供日志页浏览文件内容) ---- */
#define QSO_LINE_MAX 512      /* 单行 ADIF 最大长度 */
/* 只缓存解析摘要+文件偏移, 详情按需读文件; 数组按需在 PSRAM 动态增长, 不设条数上限 */

/* 从 ADIF 行解析出的摘要(呼号/网格/信号/频率/时间) */
typedef struct {
    char call[16];
    char grid[8];
    char rst[8];
    char freq[12];
    char time[8];      /* UTC 结束时间 HHMMSS */
} qso_log_sum_t;

/**
 * @brief 读取 log.txt 尾部若干行到内部缓冲, 并解析摘要(数组按需在 PSRAM 动态增长)。
 *        读失败(如被 USB 主机占用)时保留上次内容。
 * @param max_lines >0 只保留最后 max_lines 条; <=0 读取全部
 * @return 当前缓冲行数
 */
int qso_log_tail(int max_lines);
/** 取已加载尾部行的第 idx 行原文(idx 0 = 最旧); 越界返回 NULL */
const char *qso_log_tail_line(int idx);
/** 取第 idx 行的解析摘要(idx 0 = 最旧); 越界返回 NULL */
const qso_log_sum_t *qso_log_tail_summary(int idx);
/** 已加载尾部行数 */
int qso_log_tail_count(void);

/* ---- cfg.txt: 持久化部分配置(与 log.txt 同目录) ---- */
#define CFG_STORE_PATH "/storage/cfg.txt"
/**
 * @brief 从 cfg.txt 读取并覆盖 cfg 中"被持久化"的字段(文件不存在则保持原值)。
 *        需在 qso_log_init() 挂载存储之后调用。
 */
esp_err_t cfg_store_load(ft8_app_config_t *cfg);
/** 把 cfg 中"被持久化"的字段写入 cfg.txt(覆盖)。 */
esp_err_t cfg_store_save(const ft8_app_config_t *cfg);

#ifdef __cplusplus
}
#endif
