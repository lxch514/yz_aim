#!/bin/bash
# Configure and start this suite; /root/RS00 is never written or invoked.
set -eu
cd "$(dirname "$0")"
MINUTES="${1:-30}"
THROTTLE="${2:-500}"
SAMPLE_POINT="${SAMPLE_POINT:-0.833}"
SJW="${SJW:-8}"
IFACE=can0
CHECK_ONLY=0
EXTRA=("${@:3}")
for ((i=0; i<${#EXTRA[@]}; i++)); do
    case "${EXTRA[$i]}" in
        --iface) i=$((i+1)); IFACE="${EXTRA[$i]:?--iface needs a name}" ;;
        --iface=*) IFACE="${EXTRA[$i]#--iface=}" ;;
        --check-only) CHECK_ONLY=1 ;;
    esac
done
case "$IFACE" in *[!a-zA-Z0-9_-]*|'') echo "Invalid CAN interface"; exit 1 ;; esac
[ "$(id -u)" = 0 ] || { echo '请用 root 运行'; exit 1; }
exec 9>".control-${IFACE}.lock"
flock -n 9 || { echo '本套件已有测试运行，未重配接口。'; exit 1; }
export RS00_X6_LOCK_FD=9
if [ "$CHECK_ONLY" = 0 ]; then
    echo "配置 ${IFACE}：1Mbps，采样点 ${SAMPLE_POINT}，SJW=${SJW}；会重启该接口。"
    ip link set "$IFACE" down
    # Do not silently enable loopback/listen-only or mask error notifications.
    ip link set "$IFACE" type can bitrate 1000000 sample-point "$SAMPLE_POINT" sjw "$SJW" \
        listen-only off loopback off restart-ms 100
    # Some drivers do not support bus-error reporting; record fallback explicitly.
    if ! ip link set "$IFACE" type can berr-reporting on; then
        echo '驱动不支持 berr-reporting on；错误帧诊断可能不完整，仍记录控制器 xstats。'
    fi
    # Keep the old queue size for a controlled comparison; no unbounded retries in Python.
    ip link set "$IFACE" txqueuelen 1000
    ip link set "$IFACE" up
fi
ip -details -statistics link show "$IFACE"
exec python3 -B x6test.py --minutes "$MINUTES" --fan-throttle "$THROTTLE" "${EXTRA[@]}"
