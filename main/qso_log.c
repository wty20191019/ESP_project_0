/* ============================================================
 * QSO 日志模块 (ADIF -> /storage/log.txt + TinyUSB MSC U盘)
 *
 * 存储链路:
 *   partitions-16MiB.csv 的 "vfs" (data/fat, 10MB)
 *     -> wl_mount (磨损均衡, 扇区 4096)
 *     -> tinyusb_msc_new_storage_spiflash (FAT 挂到 /storage, 可被主机访问)
 *     -> tinyusb_driver_install (S3 原生 USB, 枚举成可移动盘)
 *
 * 互斥: 主机挂载 U 盘时 MSC 会把文件系统切给 USB, 应用侧 VFS 不可用;
 * 因此每次写日志用 open/append/close, 失败(被主机占用)只告警不阻塞。
 * ============================================================ */
#include "qso_log.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_partition.h"
#include "wear_levelling.h"
#include "esp_vfs_fat.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tinyusb.h"
#include "tinyusb_msc.h"
#include "tinyusb_default_config.h"

static const char *TAG = "qso_log";

#define QSO_PART_LABEL   "vfs"          /* FAT 分区名(见 partitions-16MiB.csv) */
#define QSO_MOUNT_PATH   "/storage"
#define QSO_LOG_PATH     QSO_MOUNT_PATH "/log.txt"

static wl_handle_t s_wl = WL_INVALID_HANDLE;
static tinyusb_msc_storage_handle_t s_msc = NULL;
static bool s_ready = false;

/* 最近 QSO 摘要环形(给 LCD 日志页) */
static char s_lines[QSO_LOG_RAM_MAX][40];
static int  s_line_head = 0;
static int  s_line_n = 0;

const char *qso_log_line(int idx)
{
    if (idx < 0 || idx >= s_line_n) return NULL;
    int i = (s_line_head - 1 - idx + QSO_LOG_RAM_MAX * 2) % QSO_LOG_RAM_MAX;
    return s_lines[i];
}

int qso_log_lines(void)
{
    return s_line_n;
}

static void qso_log_ram_add(const ft8_qso_record_t *rec)
{
    char *d = s_lines[s_line_head];
    snprintf(d, 40, "%s %s %+d/%+d %02u%02uZ",
             rec->call, rec->grid[0] ? rec->grid : "----",
             rec->rst_sent, rec->rst_rcvd,
             rec->hour_off, rec->minute_off);
    d[39] = '\0';
    s_line_head = (s_line_head + 1) % QSO_LOG_RAM_MAX;
    if (s_line_n < QSO_LOG_RAM_MAX) s_line_n++;
}

/* ---------------- 读取 log.txt 尾部行 ---------------- */
/* 只缓存摘要 + 文件偏移; 数组在 PSRAM 动态增长, 不设条数上限 */
static qso_log_sum_t *s_sums = NULL;
static long          *s_offs = NULL;
static int            s_cap  = 0;
static int            s_tail_n = 0;
static char           s_detail[QSO_LINE_MAX];    /* 详情按需读入 */
static int            s_detail_idx = -1;

static bool tail_reserve(int need)
{
    if (need <= s_cap) return true;
    int cap = s_cap ? s_cap : 256;
    while (cap < need) cap *= 2;

    qso_log_sum_t *ns = heap_caps_realloc(s_sums, (size_t)cap * sizeof(*ns),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ns == NULL) ns = realloc(s_sums, (size_t)cap * sizeof(*ns));
    if (ns == NULL) return false;
    s_sums = ns;

    long *no = heap_caps_realloc(s_offs, (size_t)cap * sizeof(*no),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (no == NULL) no = realloc(s_offs, (size_t)cap * sizeof(*no));
    if (no == NULL) return false;
    s_offs = no;

    s_cap = cap;
    return true;
}

/* 从 ADIF 行取某标签的值(如 call/gridsquare/rst_rcvd/freq) */
static void adif_field(const char *line, const char *tag, char *out, size_t cap)
{
    out[0] = '\0';
    char pat[24];
    size_t pl = 0;
    pat[pl++] = '<';
    for (const char *t = tag; *t && pl < sizeof(pat) - 3; ) pat[pl++] = *t++;
    pat[pl++] = ':';
    pat[pl] = '\0';

    const char *p = strstr(line, pat);
    if (p == NULL) return;
    p += pl;
    int len = atoi(p);
    const char *v = strchr(p, '>');
    if (v == NULL) return;
    v++;
    if (len < 0) len = 0;
    size_t n = (size_t)len;
    size_t avail = strlen(v);           /* 不能超过该行剩余长度, 否则越界读到换行/垃圾 */
    if (n > avail) n = avail;
    if (n >= cap) n = cap - 1;
    memcpy(out, v, n);
    out[n] = '\0';
}

static void adif_parse_sum(const char *line, qso_log_sum_t *s)
{
    memset(s, 0, sizeof(*s));
    adif_field(line, "call",       s->call, sizeof(s->call));
    adif_field(line, "gridsquare", s->grid, sizeof(s->grid));
    adif_field(line, "rst_rcvd",   s->rst,  sizeof(s->rst));
    adif_field(line, "freq",       s->freq, sizeof(s->freq));
    adif_field(line, "time_off",   s->time, sizeof(s->time));
    if (s->time[0] == '\0')
        adif_field(line, "time_on", s->time, sizeof(s->time));
}

const char *qso_log_tail_line(int idx)
{
    if (idx < 0 || idx >= s_tail_n) return NULL;
    if (idx != s_detail_idx) {
        FILE *f = fopen(QSO_LOG_PATH, "r");
        if (f == NULL) return NULL;
        if (fseek(f, s_offs[idx], SEEK_SET) != 0 ||
            fgets(s_detail, sizeof(s_detail), f) == NULL) {
            fclose(f);
            return NULL;
        }
        fclose(f);
        size_t l = strlen(s_detail);
        while (l > 0 && (s_detail[l - 1] == '\n' || s_detail[l - 1] == '\r')) s_detail[--l] = '\0';
        s_detail_idx = idx;
    }
    return s_detail;
}

const qso_log_sum_t *qso_log_tail_summary(int idx)
{
    if (idx < 0 || idx >= s_tail_n) return NULL;
    return &s_sums[idx];
}

int qso_log_tail_count(void)
{
    return s_tail_n;
}

int qso_log_tail(int max_lines)
{
    FILE *f = fopen(QSO_LOG_PATH, "r");
    if (f == NULL) return s_tail_n;     /* 被主机占用等: 保留旧内容 */

    char buf[QSO_LINE_MAX];
    int n = 0;
    for (;;) {
        long off = ftell(f);
        if (fgets(buf, sizeof(buf), f) == NULL) break;
        size_t l = strlen(buf);
        while (l > 0 && (buf[l - 1] == '\n' || buf[l - 1] == '\r')) buf[--l] = '\0';
        if (buf[0] != '<') continue;    /* 超长行被拆出的续段(非 ADIF 行首), 跳过 */

        if (max_lines > 0 && n >= max_lines) {
            for (int i = 1; i < max_lines; i++) {
                s_sums[i - 1] = s_sums[i];
                s_offs[i - 1] = s_offs[i];
            }
            n = max_lines - 1;
        }
        if (!tail_reserve(n + 1)) break;    /* 内存不足则停 */
        s_offs[n] = off;
        adif_parse_sum(buf, &s_sums[n]);
        n++;
    }
    fclose(f);

    s_tail_n = n;
    s_detail_idx = -1;                  /* 偏移已变, 详情缓存失效 */
    return s_tail_n;
}

/* ---------------- ADIF 拼接 ---------------- */
/* 手工拼接, 不用 snprintf 的 %s(避免 -Werror=format-truncation) */
static int adif_add(char *buf, int pos, int cap, const char *name, const char *val)
{
    char num[12];
    int nl = snprintf(num, sizeof(num), "%d", (int)strlen(val));
    if (nl < 0) nl = 0;

    size_t rem = (size_t)(cap - pos);
    if (rem == 0) return cap;

    size_t used = 0;
    #define APPEND_CH(ch) do { if (used + 1 < rem) buf[pos + used] = (ch); used++; } while (0)
    #define APPEND_S(s)   do { const char *_p = (s); while (*_p && used + 1 < rem) buf[pos + used++] = *_p++; } while (0)
    APPEND_CH('<');
    APPEND_S(name);
    APPEND_CH(':');
    APPEND_S(num);
    APPEND_CH('>');
    APPEND_S(val);
    APPEND_CH(' ');
    #undef APPEND_CH
    #undef APPEND_S
    return pos + (int)used;
}

/* 网格(Maidenhead)中心 -> 经纬度(度)。支持 4/6 位, 非法返回 false */
static bool grid_to_latlon(const char *grid, double *lat, double *lon)
{
    if (grid == NULL) return false;
    size_t n = strlen(grid);
    if (n < 4) return false;

    char a = grid[0], b = grid[1], c = grid[2], d = grid[3];
    if (a < 'A' || a > 'R' || b < 'A' || b > 'R') return false;
    if (c < '0' || c > '9' || d < '0' || d > '9') return false;

    double lo = (a - 'A') * 20.0 - 180.0 + (c - '0') * 2.0;
    double la = (b - 'A') * 10.0 - 90.0 + (d - '0') * 1.0;

    if (n >= 6) {
        char e = grid[4], f = grid[5];
        if (e < 'A' || e > 'X' || f < 'A' || f > 'X') return false;
        lo += (e - 'A') * (2.0 / 24.0) + (1.0 / 24.0);   /* 取子方格中心 */
        la += (f - 'A') * (1.0 / 24.0) + (0.5 / 24.0);
    } else {
        lo += 1.0;   /* 取方框中心 */
        la += 0.5;
    }
    *lat = la;
    *lon = lo;
    return true;
}

/* 两网格间大圆距离(km); 任一非法返回 -1 */
static int grid_distance_km(const char *g1, const char *g2)
{
    double la1, lo1, la2, lo2;
    if (!grid_to_latlon(g1, &la1, &lo1) || !grid_to_latlon(g2, &la2, &lo2)) return -1;

    const double R = 6371.0;
    const double d2r = M_PI / 180.0;
    double dla = (la2 - la1) * d2r;
    double dlo = (lo2 - lo1) * d2r;
    double s = sin(dla / 2) * sin(dla / 2) +
               cos(la1 * d2r) * cos(la2 * d2r) * sin(dlo / 2) * sin(dlo / 2);
    double c = 2 * atan2(sqrt(s), sqrt(1 - s));
    return (int)(R * c + 0.5);
}

/* ---------------- 初始化(只挂载本地, 不启 USB) ---------------- */
esp_err_t qso_log_init(void)
{
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, QSO_PART_LABEL);
    if (part == NULL) {
        ESP_LOGE(TAG, "找不到 FAT 分区 '%s'", QSO_PART_LABEL);
        return ESP_ERR_NOT_FOUND;
    }

    /* WL -> MSC 存储(应用挂到 /storage), 先不安装 USB 驱动 */
    esp_err_t err = wl_mount(part, &s_wl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wl_mount 失败: %s", esp_err_to_name(err));
        return err;
    }

    tinyusb_msc_driver_config_t drv_cfg = { 0 };
    err = tinyusb_msc_install_driver(&drv_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MSC 驱动安装失败: %s", esp_err_to_name(err));
        wl_unmount(s_wl);
        s_wl = WL_INVALID_HANDLE;
        return err;
    }

    tinyusb_msc_storage_config_t st_cfg = {
        .medium = { .wl_handle = s_wl },
        .fat_fs = {
            .base_path = QSO_MOUNT_PATH,
            .config = {
                .max_files = 4,
                .format_if_mount_failed = true,
            },
            .do_not_format = false,
            .format_flags = 0,
        },
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP,
    };
    err = tinyusb_msc_new_storage_spiflash(&st_cfg, &s_msc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建 MSC 存储失败: %s", esp_err_to_name(err));
        tinyusb_msc_uninstall_driver();
        wl_unmount(s_wl);
        s_wl = WL_INVALID_HANDLE;
        return err;
    }

    s_ready = true;

    vTaskDelay(pdMS_TO_TICKS(100));
    FILE *f = fopen(QSO_LOG_PATH, "a");
    if (f) { fclose(f); ESP_LOGI(TAG, "存储就绪: %s", QSO_MOUNT_PATH); }
    else   { ESP_LOGW(TAG, "打不开 %s", QSO_LOG_PATH); }
    return ESP_OK;
}

/* ---------------- 启动 USB 大容量存储 ---------------- */
esp_err_t qso_log_usb_start(void)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err == ESP_OK) ESP_LOGI(TAG, "USB 大容量存储已启动(U盘)");
    else               ESP_LOGE(TAG, "TinyUSB 驱动安装失败: %s", esp_err_to_name(err));
    return err;
}

/* 把存储从 USB 主机收回应用侧(解除 PC 挂载), 使应用可读写 */
void qso_log_usb_release(void)
{
    if (s_msc) tinyusb_msc_set_storage_mount_point(s_msc, TINYUSB_MSC_STORAGE_MOUNT_APP);
}

/* ---------------- 写一条 QSO ---------------- */
void qso_log_on_qso(const ft8_qso_record_t *rec, void *arg)
{
    (void)arg;
    if (!s_ready || rec == NULL) return;

    qso_log_ram_add(rec);

    char d_on[16] = "00000000", t_on[16] = "000000";
    char d_off[16] = "00000000", t_off[16] = "000000";
    if (rec->have_time) {
        snprintf(d_on,  sizeof(d_on),  "%04u%02u%02u", rec->year_on,  rec->month_on,  rec->day_on);
        snprintf(t_on,  sizeof(t_on),  "%02u%02u%02u", rec->hour_on,  rec->minute_on, rec->second_on);
        snprintf(d_off, sizeof(d_off), "%04u%02u%02u", rec->year_off, rec->month_off, rec->day_off);
        snprintf(t_off, sizeof(t_off), "%02u%02u%02u", rec->hour_off, rec->minute_off, rec->second_off);
    }

    char rst_s[8], rst_r[8], freq[16];
    snprintf(rst_s, sizeof(rst_s), "%d", rec->rst_sent);
    snprintf(rst_r, sizeof(rst_r), "%d", rec->rst_rcvd);
    snprintf(freq,  sizeof(freq),  "%.6f", (double)rec->freq_mhz);

    int dist = grid_distance_km(rec->my_grid, rec->grid);
    char comment[64];
    if (dist >= 0)
        snprintf(comment, sizeof(comment), "Distance: %d km, QSO by ESP32-FT8", dist);
    else
        snprintf(comment, sizeof(comment), "QSO by ESP32-FT8");

    char line[QSO_LINE_MAX];
    line[0] = '\0';
    int p = 0;
    p = adif_add(line, p, sizeof(line), "call",             rec->call);
    p = adif_add(line, p, sizeof(line), "QSL_RCVD",         "N");
    p = adif_add(line, p, sizeof(line), "QSL_MANUAL",       "N");
    p = adif_add(line, p, sizeof(line), "gridsquare",       rec->grid[0] ? rec->grid : "");
    p = adif_add(line, p, sizeof(line), "mode",             "FT8");
    p = adif_add(line, p, sizeof(line), "rst_sent",         rst_s);
    p = adif_add(line, p, sizeof(line), "rst_rcvd",         rst_r);
    p = adif_add(line, p, sizeof(line), "qso_date",         d_on);
    p = adif_add(line, p, sizeof(line), "time_on",          t_on);
    p = adif_add(line, p, sizeof(line), "qso_date_off",     d_off);
    p = adif_add(line, p, sizeof(line), "time_off",         t_off);
    p = adif_add(line, p, sizeof(line), "band",             rec->band[0] ? rec->band : "");
    p = adif_add(line, p, sizeof(line), "freq",             freq);
    p = adif_add(line, p, sizeof(line), "station_callsign", rec->station_callsign);
    p = adif_add(line, p, sizeof(line), "my_gridsquare",    rec->my_grid);
    p = adif_add(line, p, sizeof(line), "comment",          comment);

    /* 预留 <eor>\n 的位置, 保证每条记录一定以换行结束(否则会与下一条粘行) */
    if (p > (int)sizeof(line) - 7) p = (int)sizeof(line) - 7;
    p += snprintf(line + p, sizeof(line) - (size_t)p, "<eor>\n");
    line[p] = '\0';

    FILE *f = fopen(QSO_LOG_PATH, "a");
    if (f == NULL) {
        ESP_LOGW(TAG, "写入 %s 失败(存储可能正被 USB 主机挂载)", QSO_LOG_PATH);
        return;
    }
    fputs(line, f);
    fclose(f);
    ESP_LOGI(TAG, "QSO 已记录: %s", line);
}

/* ================= cfg.txt 持久化(仅持久化选定字段) ================= */
static void trim_tail(char *s)
{
    size_t l = strlen(s);
    while (l > 0 && (s[l - 1] == '\n' || s[l - 1] == '\r' || s[l - 1] == ' ')) s[--l] = '\0';
}

esp_err_t cfg_store_load(ft8_app_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    FILE *f = fopen(CFG_STORE_PATH, "r");
    if (f == NULL) return ESP_ERR_NOT_FOUND;

    char line[160];
    while (fgets(line, sizeof(line), f)) {
        char *eq = strchr(line, '=');
        if (eq == NULL) continue;
        *eq = '\0';
        char *k = line, *v = eq + 1;
        while (*k == ' ') k++;
        while (*v == ' ') v++;
        trim_tail(v);

        if (!strcmp(k, "callsign")) {
            strncpy(cfg->callsign, v, sizeof(cfg->callsign) - 1);
            cfg->callsign[sizeof(cfg->callsign) - 1] = '\0';
        } else if (!strcmp(k, "grid")) {
            strncpy(cfg->grid, v, sizeof(cfg->grid) - 1);
            cfg->grid[sizeof(cfg->grid) - 1] = '\0';
        } else if (!strcmp(k, "band")) {
            strncpy(cfg->band, v, sizeof(cfg->band) - 1);
            cfg->band[sizeof(cfg->band) - 1] = '\0';
        } else if (!strcmp(k, "qso_freq_mhz")) {
            cfg->qso_freq_mhz = (float)atof(v);
        } else if (!strcmp(k, "protocol")) {
            cfg->protocol = (ftx_protocol_t)atoi(v);
        } else if (!strcmp(k, "usb_mount_enable")) {
            cfg->usb_mount_enable = atoi(v) ? true : false;
        } else if (!strcmp(k, "utc_enable")) {
            cfg->utc_enable = atoi(v) ? true : false;
        } else if (!strcmp(k, "gps_utc_enable")) {
            cfg->gps_utc_enable = atoi(v) ? true : false;
        } else if (!strcmp(k, "gps_use_pps")) {
            cfg->gps_use_pps = atoi(v) ? true : false;
        } else if (!strcmp(k, "tx_slot_parity")) {
            cfg->tx_slot_parity = atoi(v) & 1;
        } else if (!strcmp(k, "tx_delay_ms")) {
            cfg->tx_delay_ms = (uint32_t)strtoul(v, NULL, 10);
        } else if (!strcmp(k, "audio_level")) {
            cfg->audio_level = (float)atof(v);
        } else if (!strcmp(k, "rx_parse_ms")) {
            cfg->rx_parse_ms = (uint32_t)strtoul(v, NULL, 10);
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "已加载 %s", CFG_STORE_PATH);
    return ESP_OK;
}

esp_err_t cfg_store_save(const ft8_app_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    FILE *f = fopen(CFG_STORE_PATH, "w");
    if (f == NULL) {
        /* 可能正被 USB 主机挂载: 收回给应用侧再试一次 */
        qso_log_usb_release();
        vTaskDelay(pdMS_TO_TICKS(50));
        f = fopen(CFG_STORE_PATH, "w");
    }
    if (f == NULL) return ESP_FAIL;

    fprintf(f, "# FT8 cfg (auto generated)\n");
    fprintf(f, "callsign=%s\n",        cfg->callsign);
    fprintf(f, "grid=%s\n",            cfg->grid);
    fprintf(f, "band=%s\n",            cfg->band);
    fprintf(f, "qso_freq_mhz=%.6f\n",  (double)cfg->qso_freq_mhz);
    fprintf(f, "protocol=%d\n",        (int)cfg->protocol);
    fprintf(f, "usb_mount_enable=%d\n", cfg->usb_mount_enable ? 1 : 0);
    fprintf(f, "utc_enable=%d\n",      cfg->utc_enable ? 1 : 0);
    fprintf(f, "gps_utc_enable=%d\n",  cfg->gps_utc_enable ? 1 : 0);
    fprintf(f, "gps_use_pps=%d\n",     cfg->gps_use_pps ? 1 : 0);
    fprintf(f, "tx_slot_parity=%d\n",  cfg->tx_slot_parity);
    fprintf(f, "tx_delay_ms=%lu\n",    (unsigned long)cfg->tx_delay_ms);
    fprintf(f, "audio_level=%.2f\n",   (double)cfg->audio_level);
    fprintf(f, "rx_parse_ms=%lu\n",    (unsigned long)cfg->rx_parse_ms);
    fclose(f);
    return ESP_OK;
}
