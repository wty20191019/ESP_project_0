#pragma once

/* QSO 日志模块: 把完成的 QSO 以 ADIF 行追加到 FAT 分区上的 /storage/log.txt,
 * 同时通过 TinyUSB MSC 把该分区作为 U 盘暴露给 PC(挂载期间主机独占读写)。 */
#include "esp_err.h"
#include "ft8_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化日志存储: 挂载 vfs(FAT) 分区 -> WL -> TinyUSB MSC -> 安装 USB 驱动。
 *        失败不致命(仅日志不可用), 会打印告警。
 * @return ESP_OK 成功
 */
esp_err_t qso_log_init(void);

/** ft8_app QSO 完成回调: 追加一条 ADIF 记录(签名匹配 ft8_qso_callback_t) */
void qso_log_on_qso(const ft8_qso_record_t *rec, void *arg);

#ifdef __cplusplus
}
#endif
