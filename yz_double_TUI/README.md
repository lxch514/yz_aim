# yz_aim_can0 —— 用板载 CAN（can0）直连维护 YZ-AIM CANopen 一体伺服

本目录是一个**独立、自包含**的维护工具：直接用 TQ3588（RK3588）**本机的 CAN 控制器**（SocketCAN 接口
`can0`）跟 YZ-AIM 一体伺服通信，**不经过 USB 转 CAN 适配器**，也不需要 `liblyscan.so` / 厂商上位机。

它与桌面上的 `yz_aim`（USB-CAN 版 TUI 工具）的关系：

| | `yz_aim`（原工具） | 本工具 `yz_aim_can0` |
|---|---|---|
| 物理链路 | USB 口 → 六叶树 USBCANFD-Mini → CAN | 板载 CAN 控制器 → 外部收发器 → CAN |
| 依赖 | `liblyscan.so`、`libusb-1.0.so.0` | 只用内核 SocketCAN，无第三方库 |
| 界面 | FTXUI 全屏 TUI，需在完整仓库内构建 | 单文件命令行，`g++` 一条命令即可编译 |
| 驱动核心 | `src/yz_aim.cpp` | **逐字复用同一份**（只删掉了 LYS 工厂函数声明） |

因此本工具保留了原工具的全部门禁逻辑：身份校验、报警/限位检查、ARM→ENABLE 顺序、心跳所有权、
运动前后重新读取 `0x6064`、动作后确认静止、退出前确认失能。

---

## 1. 目录结构

```
yz_aim_can0/
├── CMakeLists.txt              可选：用 cmake 构建
├── build.sh                    一键编译（板上原生 / --cross 交叉）
├── README.md                   本文件
├── include/
│   └── yz_aim.h                驱动接口（沿用原仓库，删掉了 LYS 工厂声明）
├── src/
│   ├── yz_aim.cpp              驱动核心：SDO 客户端 + 电机门禁与运动（原样复用）
│   ├── socketcan_transport.h   传输层接口
│   ├── socketcan_transport.cpp SocketCAN 实现（can0 收发）
│   ├── can_link_query.h        链路状态查询接口
│   ├── can_link_query.cpp      rtnetlink 只读查询接口 UP 与内核波特率
│   └── yz_aim_can0.cpp         命令行主程序
└── tools/
    └── can0_up.sh              拉起 can0 + 现场诊断
```

---

## 2. 硬件前提（先确认，再动手）

1. **内核里要有 can0**
   ```sh
   ip -br link show
   ls /sys/class/net
   dmesg | grep -i -E 'can|m_can|rockchip_can'
   ```
   RK3588 最多支持 3 路 CAN 2.0B。天嵌 TQ3588 规格书里把 CAN 列在驱动支持列表中，但
   **CAN0 的 CAN_TX/CAN_RX 是经 BTB 座子引出的 3.3V TTL 电平**，必须由底板上的
   **CAN 收发器**（SIT1042 / TJA1050 / TJA1042 之类）转换成差分 CANH/CANL 才能接电机。
   如果 `ls /sys/class/net` 里没有 `can0`，先把设备树里 CAN0 的节点和引脚复用打开，
   并确认底板上确实有收发器和接线端子。

2. **接线**：电机 DB9 的第 8 脚是 CANL（棕线）、第 9 脚是 CANH（白线），
   分别接板的 CANL / CANH，不要接反。电机内部**没有** 120Ω 终端电阻，
   请在**总线最远端**并 1 个 120Ω（手册 §7.01）。

3. **波特率与节点号必须与电机内部一致**。手册没有写出厂默认波特率；
   改波特率的步骤见手册 §7.13（也可用本工具的 `set-bitrate`，改完必须断电重新上电）。
   节点号默认按手册例子取 1。

---

## 3. 编译

### 3.1 在 TQ3588 板上原生编译

前提：板上是可编译的 rootfs（Ubuntu 版通常自带 `g++`；Buildroot 版一般没有，先 `g++ --version` 确认）。

```sh
cd yz_aim_can0
chmod +x build.sh tools/can0_up.sh
./build.sh                 # 生成 build/yz_aim_can0
./build.sh --test          # 编译并运行合成台架测试（不碰 can0，见 10）
```

### 3.2 在 Ubuntu 宿主机上交叉编译（推荐）

```sh
sudo apt install -y g++-aarch64-linux-gnu
cd yz_aim_can0
./build.sh --cross         # 生成 build-cross/yz_aim_can0（aarch64）
scp build-cross/yz_aim_can0 root@<板子IP>:/root/
```

### 3.3 用 cmake（可选）

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"          # 产物 build/bin/yz_aim_can0
```
交叉编译时再加 `-DCMAKE_TOOLCHAIN_FILE=../cmake/aarch64-linux-gnu.toolchain.cmake`。

---

## 4. 启动 CAN 到跑起来（整套流程）

```sh
# ⓪ 先停掉控制器服务：它也占着 can0，两边同时用会互相干扰
sudo systemctl stop km2-controller.service
systemctl is-active km2-controller.service     # 应输出 inactive
# 本板实测没有 km2-controller.service，只有下面三个，都不占 can0，可以照常运行：
#   snappy-link-watch.service / snappy-rs485-udev.service / snappy-log-cleanup.timer
# 换板子时按下面的方式确认到底谁占着 can0：
#   systemctl list-units --type=service --state=running | grep -i -E "can|motor|control"
#   fuser -v /sys/class/net/can0 2>/dev/null; ip -details link show can0

# ① 拉起 can0：脚本会先 down 再设参数再 up（不是多余动作，见 9.1），
#    然后打印控制器 state、被动收听 2 秒、给接线检查清单
sudo sh tools/can0_up.sh can0 1000000
#    必须看到 [√] 控制器正常（ERROR-ACTIVE）

# ② 只读握手：能读到设备类型 0x20192 / 厂商 0x331 / 产品 1，说明链路和节点号都对了
./build/yz_aim_can0 --if can0 --node 1 --bitrate 1000000 status

# ③ 进入交互模式（推荐，心跳由本进程维持）
./build/yz_aim_can0 --if can0 --node 1 --bitrate 1000000
```

工具启动时会把控制器的 `can state` 一并打出来；如果已经不是 `ERROR-ACTIVE`，
会直接提示你先跑 `tools/can0_up.sh` 复位，而不是让你去猜为什么全是超时。

一条命令跑完一个动作（例如只看一次状态、或单独执行一次失能）：

```sh
./build/yz_aim_can0 --if can0 --node 1 status
./build/yz_aim_can0 --if can0 --node 1 disable      # 退出前会自动停止并失能
```

也可以把命令按行喂进去（脚本化）：

```sh
printf 'enable\nrel 5\nrel -5\ndisable\n' | ./build/yz_aim_can0 --if can0 --node 1 --yes
```

---

## 5. 命令表

| 命令 | 说明 |
|---|---|
| `status` | 只读：读一次完整快照（身份/状态字/报警/输出/位置/转速/电流/电压/温度/电量） |
| `monitor [秒] [间隔ms]` | 只读周期刷新，默认 10 秒 / 500 ms；等待期间继续维持心跳 |
| `read-od <索引>[:<子>] [宽度]` | 只读任意对象字典，宽度 1/2/4，缺省自动从 4→2→1 试探 |
| `enable` | 只读安全检查（ARM）后使能输出；之后本进程每约 200 ms 发心跳 `0x77F:05` |
| `disable` | 停止 + 确认静止 + 清操作位 + `0x2601=0`，并核对读回 |
| `stop` | 急停保持 `0x6040=0x010F`（输出仍然使能） |
| `rel <角度>` | 相对位置运动，单位是**输出轴角度**（可小数、可负） |
| `abs <角度>` | 绝对位置运动，单位是**输出轴角度** |
| `pulses <脉冲>` | 绝对位置运动，单位是 `0x6064` 原始电机脉冲 |
| `jog <rpm> [毫秒]` | 速度模式点动，默认 500 ms，最长 5000 ms |
| `spin <rpm> [秒]` | **速度模式连续运转**，默认 60 s、最长 600 s；`spin -300` 反转。运行中按 **Ctrl-C** 随时停（会做确认停止并保持，会话继续可用） |
| `run <rpm>` | **速度模式常转**：启动后**立即回到提示符**，电机一直转到你敲 `stop` 为止；`run -300` 反转。没有时长上限，运行期间提示符变成 `yz(常转)>` |
| `speed <rpm>` | 选择本会话的运动速度 1~3000（只改本会话，不写设备） |
| `set-profile-speed <rpm>` | 写 `0x6081` 位置轮廓速度（要求输出失能且静止） |
| `set-accel <rpm/s>` | 写 `0x6083` 加速度 10~1000（要求输出失能且静止） |
| `set-current <mA>` | 写 `0x261D` 最大电流 10~10000，10 的整数倍（要求输出失能且静止） |
| `set-zero` | 置当前轴位为零点：`0x260A` ← 60006 再 ← 60016（需 `--yes`） |
| `set-node <1~127>` | 暂存新节点号并保存（需 `--yes`，断电重新上电后生效） |
| `set-bitrate <125\|250\|500\|1000>` | 暂存 CAN 波特率（需 `--yes`，断电重新上电后生效） |
| `help` / `quit` | 帮助 / 退出（退出前必然执行一次确认失能） |

常用别名：`s`=status，`e`=enable，`d`=disable，`x`=stop，`r`=rel，`a`=abs，`p`=pulses，
`j`=jog，`sp`=spin，`go`=run，`v`=speed，`?`=help，`q`=quit。

### 5.1 想让电机转起来，有四种方式

| 想要的效果 | 用什么 | 说明 |
|---|---|---|
| 只转一小段看看 | `jog 300 1000` | 速度模式点动，1 秒后自动刹停；最长只能 5 秒 |
| 转固定时长 | `spin 300 120` | 速度模式连续运转，默认 60 秒、最长 600 秒；**Ctrl-C 随时停** |
| **一直转，直到我说停** | `run 300` → … → `stop` | 启动后立刻回到提示符，想转多久转多久；`run -300` 反转 |
| 转一个精确角度 | `speed 300` → `rel 90` / `abs 0` | 位置模式，走完自动停；途中只能 Ctrl-C 打断 |

`jog` / `spin` / `run` 走的是**同一套**时序（`0x6060←3` 速度模式、`0x6083←500`、`0x60FF←rpm`、
`0x6040←0x000F` 使能），只是"能转多久"和"谁来盯"不同，看护逻辑完全一样：

- 每 ~70 ms（`run` 是每 200 ms）读一次 `0x6041` 状态字 / `0x260E` 报警 / `0x606C` 转速 / `0x6064` 位置；
- 出现报警、限位、转速超 3000、或**实际位置超出 ±60000 电机圈**时，立刻停机并报错；
- 进入前按"命令转速 × (时长 + 刹车时间)"预估行程，超过 ±60000 电机圈直接拒绝下发（保护多圈编码器）；
- 结束时（时限到 / Ctrl-C / `stop`）必然走一次**确认停止**。

### 5.2 `run` 常转模式

```
yz(只读)> enable
yz(已使能)> run 300
已启动常转 300 电机RPM（5.882 输出RPM），一直转到输入 stop 为止。
  运行期间每约 200 ms 发送心跳 0x77F:05 并复查状态字/报警/转速/位置，每秒打印一行状态；
  stop = 停止并保持（输出仍使能）；disable = 停止并失能；Ctrl-C 等同于 stop；quit 退出。
yz(常转)> [常转 1s] 状态 0437 报警 0000 输出 已使能 位置 415223 转速 300.0RPM 电流 0.812A 电压 26.31V 25℃
yz(常转)> stop
[成功] 急停保持（0x6040=0x010F）
yz(已使能)> disable
[成功] 停止并失能输出
yz(只读)> quit
```

要点：

- **谁在盯**：`run` 之后的看护由交互式主循环承担（每 100 ms 轮询输入，每 200 ms 复查一次安全项），
  所以提示符会变成 `yz(常转)>`，每秒打印一行实时状态。这也是它和 `spin` 的根本区别。
- **只允许停止类命令**：常转期间 `stop` / `disable` / `quit` / `status` / `monitor` / `read-od` 可用，
  其余写操作（`rel`/`abs`/`jog`/`spin`/`run`/`set-*`）一律被拒绝并提示先 `stop`。
- **Ctrl-C 等同于 `stop`**：常转期间按 Ctrl-C 只停止转动（输出仍使能、会话继续可用），不会退出程序。
- **安全网有两层**：本进程 `stop` 之后就不再发心跳；若进程被 kill 或崩溃，心跳一停，
  驱动在约 1 s 后自己报 `0x20` 通讯掉线报警并停机。
- **启动前的行程门禁**：常转没有终点，无法预估全程，所以改为要求**运行方向上至少留出一分钟行程**
  （一分钟行程 = 转速 × 32768 脉冲）。剩余行程不足就直接拒绝启动——比如已经开到接近 +60000 圈时只能 `run -300`。
- **`run` 只能在交互模式用**。单命令模式（`yz_aim_can0 run 300`）执行完立刻就会停止并失能，所以被明确拒绝。
- **`run` 之前必须先 `enable`**；`stop` 之后再 `run` 会自动补一次只读 ARM（这一步会打印在日志里）。

`jog` / `spin` / `rel` / `abs` 运行期间命令行仍是被占住的，不能一边转一边敲 `stop`，
提前停靠 **Ctrl-C**。按了 Ctrl-C 只会打断这一次运动——电机会被确认停止并保持，
**本会话继续可用**，可以接着敲 `disable` / `quit`（退出前仍会自动确认一次失能）。

选项：`--if`（默认 can0）、`--node`（默认 1）、`--bitrate`（默认 1000000，仅用于与内核报告值比对）、
`--timeout MS`（单次动作总预算，默认 20000）、`--yes`、`--version`、`--help`。

一次典型操作：

```
yz(只读)> status                       # 只读，确认报警 0x0000、输出已禁止、转速 0
yz(只读)> enable                       # 安全检查通过后使能
yz(已使能)> speed 30                   # 本次运动速度 30 电机RPM（默认就是 30）
yz(已使能)> rel 5                      # 输出轴正向 5°（= 23211 电机脉冲）
yz(已使能)> rel -5                     # 转回
yz(已使能)> disable                    # 停稳并失能
yz(只读)> quit
```

---

## 6. 安全模型（和原工具一致，务必读一遍）

- **启动只读**：打开接口、读快照、ARM 都不写控制字、不使能、不发心跳。
- **心跳所有权**：`enable` 时会先读走原来的 `0x1016:1` 消费者参数，装成
  `节点0x7F / 1000ms`，然后由**本进程**每约 200 ms 发一次 `0x77F:05`。
  等待输入、监视、运动、对话框期间都在维持。
  **不要直接 `kill` 进程**：心跳一停，驱动约 1 s 后就报 `0x20` 通信掉线并停机。
  请用 `disable` 或 `quit`（两者都会先做一次确认失能）。
- **每次运动前都会重新做只读 ARM 检查并重读 `0x6064`**，目标坐标必须在手册范围内
  （带电池多圈记忆的坐标范围 ±60000 电机圈 = ±1966080000 电机脉冲）。
- **动作完成后必须重新 ARM**：这是驱动的设计（`stop()` 会清掉 armed 状态），
  本工具会在下次运动前自动补一次，所以连做几个动作是正常的。
- **超时即锁止**：任何一次运动超时/SDO 失败会让本会话进入"锁止"状态，
  此时只能 `stop` / `disable` / 退出，需要退出工具重新开始。这是有意的安全设计。
- `stop` 和 `disable` 是两个不同操作，**都不代表机械抱闸已生效**。
- 本工具**不会**自动回零、不会自动保存参数、不会改限位、不会重连。
  `set-zero` / `set-node` / `set-bitrate` 改写的是设备内部参数，必须加 `--yes` 才执行。

---

## 7. 坐标与换算基准

本设备按 **51:1** 减速比、**32768 电机脉冲/电机圈** 换算（与 `yz_aim` 工具相同）：

| 电机脉冲 | 输出轴角度 |
|---|---|
| 1 | 0.000215° |
| 23211 | 5° |
| 69632 | 15° |
| 139264 | 30° |
| 417792 | 90° |
| 1671168 | 360°（1 输出圈） |

手册坐标范围 ±1966080000 电机脉冲 = ±1176.470588 输出圈 = ±423529.411765°。
这个换算是**理论坐标换算**，不代表机械允许行程。时间预算按
`5000 ms + 2×ceil(rpm×1000/500) + ceil(|Δ脉冲|×60000/(rpm×32768))` 估算，
例如 90°/30 RPM 约 30.6 s，90°/3000 RPM 约 17.3 s。

---

## 8. 排查

| 现象 | 先看这里 |
|---|---|
| 打开失败"CAN 接口不存在" | `ip -br link show`；设备树没使能 CAN0 时不会有这个接口 |
| 打开失败"CAN 接口未启动" | `sh tools/can0_up.sh can0 1000000` |
| 所有读都是超时 | 按第 9 节顺序查：先看控制器 state，再查接线/终端电阻/波特率/节点号 |
| 工具开头提示"控制器状态 ERROR-PASSIVE / BUS-OFF" | **先复位**：`sh tools/can0_up.sh can0 1000000`。此状态下收发都不可信 |
| 提示"内核报告的波特率与 --bitrate 不一致" | 内核报告 0 表示该驱动不上报（可能由设备树固定）；非 0 且不一致时几乎收不到应答 |
| 收到应答但报 `SDO abort` | 该对象不存在或宽度不符；用 `read-od 1000:0 4` 这类显式宽度再试 |
| 报 "ambiguous SDO session" | 上一次事务不可信，**退出并重新启动**本工具 |
| 反复 bus-off | 接线/终端电阻/共地问题；`tools/can0_up.sh` 里已尝试打开 `restart-ms` |
| `enable` 失败提示报警 | 读 `0x260E`：`0x20` 通信掉线（重新有心跳会恢复）、`0x10` 电池掉电、`0x12` 堵转、`0x14` 失速、`0x15` 过压 |

---

## 9. 现场排障顺序（在 TQ3588 + RK3588 上实测过的方法）

这套步骤是用来回答"到底是软件问题还是接线问题"，按顺序做，每一步都有明确判据。

### 9.1 先复位控制器，别信 `rx_packets`

```sh
sh tools/can0_up.sh can0 1000000
ip -details link show can0 | grep -E "can state|bitrate"
```

必须看到 `can state ERROR-ACTIVE`。

**关键坑**：控制器一旦因为"发过帧没人应答"掉进 `ERROR-PASSIVE`，
`/sys/class/net/can0/statistics/rx_packets` 会以**每秒上百**的速度虚增，
看着像总线上有大量报文，其实一个字节都进不了任何 CAN 套接字。
所以**不要用 rx_packets 判断总线有没有报文**，用下面的被动收听。

### 9.2 被动收听（只接收，不发送）

```sh
timeout 3 candump -t d can0        # 装了 can-utils 就用这个
```

收到 0 帧有几种可能：电机本来就没开心跳（默认多半没开），或者电机没接、没上电、
或者物理层不通。所以**收到 0 帧不能证明电机没接**；
但**收到帧就一定说明总线上有活节点**，此时再查波特率/节点号才有意义。

### 9.3 用回环模式把"软件"和"接线"分开

回环模式**不会往总线上发任何帧**，是安全的：

```sh
ip link set can0 down
ip link set can0 type can bitrate 1000000 loopback on
ip link set can0 up
python3 - <<'EOF'
import socket, struct
s=socket.socket(socket.AF_CAN, socket.SOCK_RAW, 1); s.bind(("can0",)); s.settimeout(2)
for i in range(5): s.send(struct.pack("=IB3x8s", 0x321, 8, bytes([i]*8)))
n=0
try:
    while True: s.recv(16); n+=1
except socket.timeout: pass
print("回环收到", n, "帧（送 5 帧，收到 5 帧 = 控制器/驱动/软件都正常）")
EOF
# 一定要恢复！
ip link set can0 down
ip link set can0 type can bitrate 1000000 loopback off restart-ms 100
ip link set can0 txqueuelen 1000
ip link set can0 up
```

- 回环 5/5：**控制器、驱动、SocketCAN、软件路径全部正常**，问题一定在物理层 → 查 9.4。
- 回环收不到：才是驱动/控制器层面的问题。

### 9.4 物理层按顺序查

1. **波特率**：`ip -details link show can0` 报的与电机内部一致（本手册 §7.13 可改电机波特率，
   改完必须断电重新上电）。
2. **接线**：板子 `CAN_H` → 电机 `CANH(白)`，板子 `CAN_L` → 电机 `CANL(棕)`，不能接反。
3. **终端电阻**：电机内部**没有** 120Ω，必须在总线最远端 `CAN_H`–`CAN_L` 之间并 1 个 120Ω。
   万用表量 `CAN_H`–`CAN_L` 直流电阻：只有一端有终端 ≈ 120Ω；两端都有 ≈ 60Ω；读到 ∞ = 没有终端。
4. **收发器**：RK3588 的 CAN 是 TTL 电平，必须经过底板上的 CAN 收发器才能接 `CAN_H/CAN_L`。
   底板没贴收发器 / 收发器没供电或没使能时，现象和"电机没接"完全一样：能发、没人应答、也收不到。
5. **节点号**：`--node` 必须等于电机内部节点号。
6. **判据**：`cansend can0 123#00` 发一帧后
   ```sh
   ip -details link show can0 | grep "can state"
   ```
   如果立刻变 `ERROR-PASSIVE`，说明这一帧**没有被任何节点 ACK**，即总线上没有活着的对端。

### 9.5 最后一步：只读握手

```sh
./build/yz_aim_can0 --if can0 --node 1 --bitrate 1000000 status
```

读到 `设备类型 0x20192 / 厂商 0x331 / 产品代码 1` 才算链路打通。到这一步之前不要 `enable`。

---

## 10. 实测记录与未验证事项

### 已在 TQ3588（192.168.0.181，Ubuntu 22.04.5，内核 6.1.141）上实测通过

- 编译：`./build.sh` 用 g++ 11.4.0 一次通过，无警告无错误。
- can0 存在，驱动 `rockchip_canfd`，设备树节点 `can@fea50000`，`compatible = rockchip,can-2.0`（经典 CAN 2.0，非 FD），
  引脚复用正确（`gpio0-15/16` → `can0m0-pins`），时钟 198 MHz。
- **回环模式送 5 帧收到 5 帧** → 控制器 / 驱动 / SocketCAN / 软件路径全部正常。
- 在 can0 上以 1000000 / 500000 / 250000 / 125000 四档分别被动收听，**均收到 0 帧**；
  发帧后控制器立刻进入 `ERROR-PASSIVE` → 该总线上当时没有任何活着的对端（电机未接/未上电/物理层不通）。
- 曾经遇到过：接口被留在 `ERROR-PASSIVE` 状态（设备树默认时序 `sample-point 0.868 sjw 4`），
  此时 `rx_packets` 以 ~125 帧/秒虚增。`down` 后重设 `bitrate 1000000`（内核算出 `sample-point 0.747 sjw 1`）
  再 `up`，状态回到 `ERROR-ACTIVE`，虚增立即停止。**这就是 `can0_up.sh` 一律 down/up 的原因。**

### 仍未验证

- **`spin`（连续运转）是后加的命令**：它复用了 `jog` 的整条已验证时序，只改了时长上限，
  但**没有在真实电机上跑过**（现场当时总线上没有任何节点）。第一次用请从
  `spin 100 5`（低速、5 秒）开始，确认能正常起停、能报出位置和转速，再放长时间。
- `run` 常转**没有在真实电机上跑过**。它和 `jog`/`spin` 共用同一套速度模式时序与看护，
  且已用合成台架测试覆盖（见下），但真机第一次请从 `run 100` 起，手边准备好 `stop`。
- 电机侧的真实波特率、节点号、是否装电池多圈选件、机械允许行程、零位，都需要现场确认。
- 使能/运动/置零/改节点/改波特率这些**写操作一次都没有在真实电机上跑过**——
  在 9.5 的只读握手通过之前，不要执行它们。
- RK3588 的 CAN 驱动在部分 BSP 上不支持用 `ip` 改波特率（由设备树固定），脚本会打印提示而不是静默失败。

### 合成台架测试（`./build.sh --test`）

`test/yz_aim_sustain_test.cpp` 用一个只回答内存的假驱动器，把 `run` 的报文与看护路径整条跑一遍，
**不打开 can0、不与任何真实硬件通信**。截至本版 **65 项检查全过**：

- 启动报文顺序：`0x6040←0x010F` → … → `0x60FF←转速` → 心跳 `0x77F:05` → `0x6040←0x000F`（控制字正好写 2 次）；
- 正常巡检能报出转速 300.0 电机RPM、实时位置、命令转速，且不打断运转；
- 报警 `0x12` → 停机、清常转标志、锁止会话、**停发心跳**；
- 转速 3100 电机RPM → `unsafe`；位置越出 ±60000 圈 → `limit`；
- `stop` 能结束常转且不算故障（心跳继续维持，输出仍使能）；
- `request_cancel()`（=Ctrl-C）→ 确认停止后按"已取消"收尾，不算故障；
- 拒绝 `0 / ±3001 RPM`、未使能时的调用、node 127；
- 行程门禁：300 电机RPM 需 ≥ 9,830,400 脉冲（一分钟），9,000,000 拒绝、10,000,000 放行；
- `stop` 会释放 ARM，第二次 `run` 必须先重新 ARM（交互式 CLI 会自动补这一步）。

测试只覆盖驱动层（`src/yz_aim.cpp`）。CLI 层的主循环、提示符切换、`sustain_tick` 节拍
在板子上用真实进程跑过参数与门禁分支，但**整条"启动→常转→stop"链路仍未接真实电机验证**。






ip link set can0 down
ip link set can0 type can bitrate 1000000 sample-point 0.833
ip link set can0 up

ip -details link show can0

yz_aim/

# 改ID

./build/yz_aim_can0 --if can0 --node 1 --yes
yz(只读)> set-node 2
yz(只读)> quits

```sh
# can0 版
cd /root/yz_aim_can0
sh tools/can0_up.sh can0 1000000                                  
./build/yz_aim_can0 --if can0 --node 1 --yes set-node 2       
./build/yz_aim_can0 --if can0 --node 2 status    
# USB 版
cd /root/yz_aim && ./build.sh --only yz_aim_cli
./build-native/release/bin/yz_aim_cli --bitrate 1000000 --node 1 --yes set-node 2
```



# 然后断电重新上电，这台就是 node 2 了

```bash
# ── 0) 编译（已实测：一次通过，含 yz_aim_cli，零警告）
cd /root/yz_aim
bash build.sh                      
ctest --test-dir build-native/release --output-on-failure   

# ── 1) 环境检查（这一套没有"启动 CAN"，USB 链路不碰 can0）
lsusb | grep 3068                 
ls -l /usr/local/lib/liblyscan.so  
```

插上USB转CAN

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

./build-native/release/bin/yz_aim_cli \
    --bitrate 1000000 --abit-timing 0x01400106 \
    --node 1 --lys 0 --channel 0 \
    --library /usr/local/lib/liblyscan.so
```



yz_aim_can0/

```bash
# ── 0) 编译（已实测通过）
cd /root/yz_aim_can0
./build.sh                         

# ── 1) 启动 CAN ← 唯一需要"启动 CAN"的一套
sh tools/can0_up.sh can0 1000000

# ── 2) 只读握手
./build/yz_aim_can0 --if can0 --node 1 --bitrate 1000000 status

# ── 3) 交互模式
./build/yz_aim_can0 --if can0 --node 1 --bitrate 1000000
```

## 进去之后调转速的顺序

```
status                  ← 先只读一次，确认身份读到 0x20192 / 厂商 0x331 / 产品 1
read-od 0x1000:0 4      ← 再确认一次设备类型 = 0x20192，链路才算真通

enable                  ← 使能输出，提示符变成 yz(已使能)>
speed 300               ← 设本会话速度 300 电机RPM（默认 30）
rel 90                  ← 输出轴正转 90°
rel -90                 ← 反转 90°
jog 200 500             ← 200 RPM 正转 500 ms；jog -200 500 就是反转
spin 300                # 正转 300 电机RPM，跑 60 秒后自动停
spin -300               # 反转，跑 60 秒
spin 300 600            # 正转，跑满 10 分钟
run 300                 # 常转
stop                    ← 急停保持
disable                 ← 停止并失能
quit                    ← 退出（退出前会自动再确认一次失能）
```

yz_aim_can0_double/

```sh
cd /root/yz_aim_can0_double
sh tools/can0_up.sh can0 1000000
./build.sh                                    
./build/yz_aim_can0 --if can0 --node 1 --node2 2
```



