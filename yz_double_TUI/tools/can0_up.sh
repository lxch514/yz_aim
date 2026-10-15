#!/bin/sh
# 拉起板载 CAN 接口并打印诊断信息；只改接口状态，不发送任何 CAN 帧。
#
# 用法：sh tools/can0_up.sh [接口] [波特率]
#   sh tools/can0_up.sh                    # 默认 can0 1000000
#   sh tools/can0_up.sh can0 500000
#
# 为什么一定要 down/up：接口在 boot 时可能已被设备树默认参数带起来，或者在之前
# 的调试里因为"发帧没人应答"掉进了 ERROR-PASSIVE。保持原状态再设参数不会真正复位
# 控制器，现象就是"什么都收不到、而且 rx_packets 还在飞快地涨"。
# 所以本脚本一律 down -> 重设参数 -> up。

IF=${1:-can0}
BITRATE=${2:-1000000}

echo "=== 1) 内核是否识别到 CAN 接口 ==="
ip -br link show 2>/dev/null | sed -n '1,40p'
echo "--- /sys/class/net ---"
ls /sys/class/net 2>/dev/null | tr '\n' ' '
echo
if ! ip link show "$IF" >/dev/null 2>&1; then
    echo "[!] 没有找到接口 $IF。"
    echo "    先确认：1) 设备树是否使能了该 CAN 节点；2) 内核是否有 CAN 驱动。"
    echo "    可执行：dmesg | grep -i -E 'can|m_can|rockchip_can'"
    echo "            cat /proc/device-tree/model"
    echo "            grep -i can /sys/kernel/debug/pinctrl/*/pinmux-pins"
    exit 1
fi

echo
echo "=== 2) 复位控制器：down -> 重设参数 -> up ==="
ip link set "$IF" down || { echo "[!] $IF 无法 down"; exit 1; }
if ip link set "$IF" type can bitrate "$BITRATE" 2>/dev/null; then
    echo "[√] 已通过 ip 设置仲裁波特率 $BITRATE"
else
    echo "[!] 该驱动不支持用 ip 改波特率（常见于由设备树固定的 rockchip CAN）。"
    echo "    波特率必须与电机内部设置一致，否则收不到任何应答。"
fi
# bus-off 后自动恢复，便于现场调试；不支持时忽略。
ip link set "$IF" type can restart-ms 100 2>/dev/null \
    && echo "[√] 已开启 bus-off 自动恢复（restart-ms 100）" \
    || echo "[i] 该驱动不支持 restart-ms，bus-off 后需要手工 down/up"
ip link set "$IF" txqueuelen 1000 2>/dev/null && echo "[√] txqueuelen = 1000"
ip link set "$IF" up || { echo "[!] $IF 无法 up"; exit 1; }
sleep 1

echo
echo "=== 3) 控制器状态 ==="
ip -details -statistics link show "$IF"
IPSTATE=$(ip -details link show "$IF" | sed -n 's/.*can state \([A-Z-]*\).*/\1/p')
echo "解析到的 state = ${IPSTATE:-未知}"
case "$IPSTATE" in
    ERROR-ACTIVE)
        echo "[√] 控制器正常（ERROR-ACTIVE）" ;;
    *)
        echo "[!] 控制器不在 ERROR-ACTIVE。此状态下收发都不可信，先按下面第 5 节排查接线，"
        echo "    然后重新跑本脚本（脚本已经带 down/up 复位）。" ;;
esac
echo
echo "[提醒] 不要用 rx_packets 判断总线有没有报文：控制器处于 ERROR-PASSIVE 时，"
echo "       这个计数会以每秒上百的速度虚增，看着像有流量，其实一个字节都进不了套接字。"

echo
echo "=== 4) 被动收听 2 秒（只接收，不发送）看总线上有没有节点在说话 ==="
if command -v candump >/dev/null 2>&1; then
    N=$(timeout 2 candump -n 100 "$IF" 2>/dev/null | wc -l)
    echo "candump 收到 $N 帧"
    if [ "$N" != "0" ]; then
        timeout 1 candump -t d "$IF" 2>/dev/null | head -10
    fi
elif command -v python3 >/dev/null 2>&1; then
    python3 - "$IF" <<'PYEOF'
import socket, sys, time
iface = sys.argv[1]
try:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, 1)
    s.bind((iface,)); s.settimeout(2)
except OSError as e:
    print("python 打开 CAN 套接字失败：%s" % e); raise SystemExit
n = 0; t0 = time.time()
while time.time() - t0 < 2:
    try: s.recv(16)
    except socket.timeout: break
    n += 1
print("收到 %d 帧" % n)
PYEOF
else
    echo "[i] 没有 candump，也没有 python3，跳过这一步"
fi
echo "说明：电机默认不一定开心跳，所以\"收到 0 帧\"不代表电机没接；"
echo "      但\"能收到帧\"就说明总线上确实有活节点。"

echo
echo "=== 5) 接线与现场核对（读不到电机时按顺序查）==="
echo "  1) 板子的 CAN_H/CAN_L 与电机的 CANH(白)/CANL(棕) 一一对应，不能接反。"
echo "  2) 电机内部没有 120Ω 终端电阻，必须在总线最远端的 CAN_H-CAN_L 之间并 1 个 120Ω。"
echo "     万用表量 CAN_H-CAN_L 的直流电阻：只有一个 120Ω 终端时约 120Ω；两端都有约 60Ω；"
echo "     读到 ∞ 说明根本没有终端电阻，通信一定失败。"
echo "  3) 板上 RK3588 的 CAN 是 TTL 电平，必须经过底板上的收发器才能接 CAN_H/CAN_L。"
echo "     如果底板没贴收发器 / 收发器没有供电或使能，会出现\"能发不能收、而且没人应答\"。"
echo "  4) 电机波特率、节点号必须与命令一致；改过电机波特率要断电重新上电才生效。"
echo "  5) 最后一步再用只读命令验证：能读到设备类型 0x20192 / 厂商 0x331 / 产品 1 才算通。"

echo
echo "接着可以执行："
echo "  ./build/yz_aim_can0 --if $IF --node 1 --bitrate $BITRATE status"
