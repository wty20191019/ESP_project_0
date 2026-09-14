#pragma once

/* 自动 QSO 引擎内部数据结构(仅供 ft8_app.c 及其它需要观测引擎的任务使用)。
 * 数据流: RX 任务把解码消息构造成 qso_rx_t 投队列, 引擎消费后驱动状态机。 */
#include <stdint.h>
#include <stdbool.h>
#include "ft8/message.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 结构化解码消息(RX 任务 -> 引擎队列) */
typedef struct {
    ftx_message_type_t msg_type;
    char call_to[16];
    char call_de[16];
    char extra[12];
    ftx_field_t ftypes[FTX_MAX_MESSAGE_FIELDS];
    float freq_hz;
    float snr_db;
    int64_t slot;
} qso_rx_t;

/* 语义事件(已按"发射方视角"归一化) */
typedef enum {
    QSO_EVT_NONE,   /* 与本站无关 */
    QSO_EVT_CQ,     /* 对方呼叫 CQ(可应答) */
    QSO_EVT_ANSWER, /* 对方回答我方 CQ(点我方呼号 + 网格) */
    QSO_EVT_REPORT, /* 对方发我方信号报告(不带 R) */
    QSO_EVT_RREPORT,/* 对方发我方 R 报告 */
    QSO_EVT_RRR,    /* 对方 RRR */
    QSO_EVT_RR73,   /* 对方 RR73 */
    QSO_EVT_73,     /* 对方 73 */
} qso_evt_kind_t;

typedef struct {
    qso_evt_kind_t kind;
    char sender[16];   /* 发射方呼号 */
    char grid[8];      /* 网格(仅 CQ/ANSWER) */
    int  rst_db;       /* 报告 dB(仅报告类) */
    int  parity;       /* 收到该消息的时隙奇偶(= 对方发射相位) */
} qso_evt_t;

/* QSO 状态机阶段(每阶段对应一份我方要发的 cfg.tx 内容) */
typedef enum {
    QSO_ST_IDLE,        /* 空闲: 主叫模式=CQ, 应答模式=静默 */
    QSO_ST_REPORT,      /* 主叫: 已发 REPORT, 等对方 R 报告/结束 */
    QSO_ST_RR73,        /* 主叫: 已发 RR73, 等对方 73 */
    QSO_ST_CALL,        /* 应答: 已发 CALL, 等对方 REPORT */
    QSO_ST_RRPT,        /* 应答: 已发 R 报告, 等对方 RR73/RRR */
    QSO_ST_73,          /* 应答: 已决定发 73, 发完即完成 */
} qso_state_t;

/* 引擎任务运行时上下文 */
typedef struct {
    qso_state_t state;
    bool engaged;           /* 是否已锁定某台在通联中 */
    char peer[16];
    char peer_grid[8];
    int  peer_rst;          /* 对方报告给我们的 dB */
    int  my_rst;            /* 我方发出的 dB */
    int  tx_parity;         /* 我方发射时隙奇偶 */
    int  attempts;          /* 当前阶段我方已发射次数 */
    int64_t last_counted;   /* 已计数的我方时隙号 */
    bool have_start;        /* 已记录 QSO 起始 UTC 时间 */
    uint16_t year_on;  uint8_t month_on,  day_on,  hour_on,  minute_on,  second_on;
} qso_ctx_t;

/* 最近记录的呼号(完成=永久; 放弃=暂避 SKIP_AGE_SLOTS 个时隙) */
typedef struct {
    char call[16];
    int64_t slot;
    bool worked;
} qso_recent_t;

#ifdef __cplusplus
}
#endif
