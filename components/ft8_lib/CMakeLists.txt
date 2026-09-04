# ft8_lib 组件 (FT8/FT4 编解码库, 移植自 Karlis Goba 的 ft8_lib)
#
# 包含: ft8/  (编码/解码/LDPC/CRC/消息文本)
#       fft/  (kiss_fft, 供 common/monitor 做 STFT)
#       common/(仅 monitor.{c,h} + common.h, 音频输入由 ESP32 I2S 提供)
#
# 说明:
#   - LDPC/奇偶校验表已编译进 ft8/constants.c, ft4_ft8_public/*.dat
#     仅是离线再生成脚本的输入, 运行时不需要, 故未拷贝.
#   - message.c 用到 stpcpy, 编译期需要 _GNU_SOURCE / HAVE_STPCPY.
#   - 解码属于重浮点运算, 这里用 -O2 优化.

idf_component_register(
    SRCS
        "ft8/constants.c"
        "ft8/crc.c"
        "ft8/decode.c"
        "ft8/encode.c"
        "ft8/ldpc.c"
        "ft8/message.c"
        "ft8/text.c"
        "fft/kiss_fft.c"
        "fft/kiss_fftr.c"
        "common/monitor.c"
    INCLUDE_DIRS "." "ft8" "fft" "common"
)

component_compile_options(
    -O2
    -D_GNU_SOURCE
    -DHAVE_STPCPY
    -Wno-error
)
