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
static char s_tail[QSO_TAIL_MAX][QSO_LINE_MAX];
static qso_log_sum_t s_tail_sum[QSO_TAIL_MAX];
static int  s_tail_n = 0;

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
    return s_tail[idx];
}

const qso_log_sum_t *qso_log_tail_summary(int idx)
{
    if (idx < 0 || idx >= s_tail_n) return NULL;
    return &s_tail_sum[idx];
}

int qso_log_tail_count(void)
{
    return s_tail_n;
}

int qso_log_tail(int max_lines)
{
    if (max_lines <= 0) max_lines = QSO_TAIL_MAX;
    if (max_lines > QSO_TAIL_MAX) max_lines = QSO_TAIL_MAX;

    FILE *f = fopen(QSO_LOG_PATH, "r");
    if (f == NULL) return s_tail_n;     /* 被主机占用等: 保留旧内容 */

    /* 滚动窗口: 始终保留最后 max_lines 行(原文 + 解析摘要) */
    char buf[QSO_LINE_MAX];
    int n = 0;
    while (fgets(buf, sizeof(buf), f)) {
        size_t l = strlen(buf);
        while (l > 0 && (buf[l - 1] == '\n' || buf[l - 1] == '\r')) buf[--l] = '\0';
        if (n < max_lines) {
            strncpy(s_tail[n], buf, QSO_LINE_MAX - 1);
            s_tail[n][QSO_LINE_MAX - 1] = '\0';
            adif_parse_sum(s_tail[n], &s_tail_sum[n]);
            n++;
        } else {
            for (int i = 1; i < max_lines; i++) {
                memcpy(s_tail[i - 1], s_tail[i], sizeof(s_tail[0]));
                s_tail_sum[i - 1] = s_tail_sum[i];
            }
            strncpy(s_tail[max_lines - 1], buf, QSO_LINE_MAX - 1);
            s_tail[max_lines - 1][QSO_LINE_MAX - 1] = '\0';
            adif_parse_sum(s_tail[max_lines - 1], &s_tail_sum[max_lines - 1]);
        }
    }
    fclose(f);

    s_tail_n = n;
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

/* ---------------- 初始化 ---------------- */
esp_err_t qso_log_init(bool usb_mount)
{
#if !SOC_USB_OTG_SUPPORTED
    usb_mount = false;   /* 芯片无 USB OTG, 只能本地挂载 */
#endif
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, QSO_PART_LABEL);
    if (part == NULL) {
        ESP_LOGE(TAG, "找不到 FAT 分区 '%s'", QSO_PART_LABEL);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err;

    if (!usb_mount) {
        /* 只本地挂载 FAT(带磨损均衡), 不启动 USB */
        esp_vfs_fat_mount_config_t mcfg = {
            .max_files = 4,
            .format_if_mount_failed = true,
        };
        err = esp_vfs_fat_spiflash_mount_rw_wl(QSO_MOUNT_PATH, QSO_PART_LABEL, &mcfg, &s_wl);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "本地挂载 FAT 分区失败: %s", esp_err_to_name(err));
            return err;
        }
        s_ready = true;
        vTaskDelay(pdMS_TO_TICKS(50));
        FILE *f0 = fopen(QSO_LOG_PATH, "a");
        if (f0) { fclose(f0); ESP_LOGI(TAG, "日志就绪(本地): %s", QSO_LOG_PATH); }
        else    { ESP_LOGW(TAG, "打不开 %s", QSO_LOG_PATH); }
        return ESP_OK;
    }

    /* ---- U 盘模式: WL -> MSC 存储 -> USB 驱动 ---- */
    err = wl_mount(part, &s_wl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wl_mount 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* MSC 驱动(带默认事件回调) */
    tinyusb_msc_driver_config_t drv_cfg = { 0 };
    err = tinyusb_msc_install_driver(&drv_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MSC 驱动安装失败: %s", esp_err_to_name(err));
        wl_unmount(s_wl);
        s_wl = WL_INVALID_HANDLE;
        return err;
    }

    /* 把 vfs 分区做成 MSC 存储: 应用挂到 /storage, 未格式化则自动格式化 */
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

    /* 安装 USB 设备驱动(默认描述符, 由 Kconfig 打开 MSC) */
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB 驱动安装失败: %s", esp_err_to_name(err));
        /* 仍可尝试本地写文件, 只是不暴露 U 盘 */
    }

    s_ready = true;

    /* 首次确保文件存在(挂载/格式化需要一点时间) */
    vTaskDelay(pdMS_TO_TICKS(100));
    FILE *f = fopen(QSO_LOG_PATH, "a");
    if (f) {
        fclose(f);
        ESP_LOGI(TAG, "日志就绪: %s (U盘模式可挂载该分区)", QSO_LOG_PATH);
    } else {
        ESP_LOGW(TAG, "暂时打不开 %s(可能正被 USB 主机占用)", QSO_LOG_PATH);
    }
    return ESP_OK;
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

    char line[320];
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
    if (p < (int)sizeof(line) - 8)
        p += snprintf(line + p, sizeof(line) - (size_t)p, "<eor>\n");
    if (p >= (int)sizeof(line)) p = (int)sizeof(line) - 1;
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
