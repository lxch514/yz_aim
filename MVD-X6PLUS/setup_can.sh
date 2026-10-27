#!/bin/sh
# 配置 can0 位时序（SJW=8）并复位控制器，由 run_test.py 在启动测试前调用；只改接口参数，不发任何 CAN 帧。
#
# 用法：sh setup_can.sh [接口] [波特率]
#   默认 can0 1000000。可单独执行，也可被脚本调用。

IF=${1:-can0}
BITRATE=${2:-1000000}
SJW=8
SP=0.828

if ! ip link show "$IF" >/dev/null 2>&1; then
    echo "[×] 没有找到接口 $IF" >&2
    exit 1
fi

# 必须 down/up 才会真正复位控制器，否则 ERROR-PASSIVE 的残留状态会带进下一次测试
ip link set "$IF" down 2>/dev/null || { echo "[×] $IF 无法 down" >&2; exit 1; }

if ! ip link set "$IF" type can bitrate "$BITRATE" sample-point "$SP" sjw "$SJW" 2>/dev/null; then
    echo "[!] 驱动不接受 sjw/sample-point，退回只设波特率" >&2
    ip link set "$IF" type can bitrate "$BITRATE" 2>/dev/null || \
        echo "[!] 该驱动不支持用 ip 改波特率，沿用设备树默认值" >&2
fi

ip link set "$IF" up || { echo "[×] $IF 无法 up" >&2; exit 1; }

sleep 0.3

IPOUT=$(ip -details link show "$IF")
# 只取位时序那一行的 sjw，驱动能力行里的 "sjw 1..128" 会误匹配
GOT_SJW=$(echo "$IPOUT" | sed -n 's/.*phase-seg2 [0-9][0-9]* sjw \([0-9][0-9]*\).*/\1/p' | head -n 1)
GOT_STATE=$(echo "$IPOUT" | sed -n 's/.*can state \([A-Z-]*\).*/\1/p' | head -n 1)
echo "位时序：bitrate $BITRATE、采样点 $SP、SJW ${GOT_SJW:-未知}；控制器状态 ${GOT_STATE:-未知}"

# SJW 读回不对说明设置没生效，直接算失败，避免跑一轮无效测试
if [ "$GOT_SJW" != "$SJW" ]; then
    echo "[×] SJW 读回 ${GOT_SJW:-未知}，未达到 $SJW" >&2
    exit 1
fi

if [ "$GOT_STATE" != "ERROR-ACTIVE" ]; then
    echo "[!] 控制器不在 ERROR-ACTIVE，先查接线与 120Ω 终端；本次仍继续，但收发可能不可信" >&2
fi

exit 0
