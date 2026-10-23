# YZ-AIM CANopen 维护工具

`yz_aim_setup` and `yz_aim_test` are standalone FTXUI maintenance frontends for the YZ-AIM CANopen motor driver. They are not linked into Controller. Opening, observing, arming without ENABLE, and exiting a read-only or arm-only session emit no CAN writes, NMT, or heartbeat. Once explicit ENABLE owns the output, the worker services its heartbeat during idle time and confirmation dialogs, including while refreshing telemetry. A real session refuses to open the adapter while `km2-controller.service` is active.

## 1.5.0 厂家手册速度与多圈坐标范围（隔离发布，实机待验）

本版按原始 PDF `/home/zakiu/km2-x/YZ-AIM_canopen用户手册_v1_66（出线版).pdf` 第 11 页 `0x6081` 位置轮廓速度 **0..3000 电机 RPM**、第 12 页 `0x60FF` 速度目标 **−3000..+3000 电机 RPM** 扩展维护工具。`yz_aim_test` 的 `v` 选择本会话后续动作速度 **1..3000 电机 RPM**（默认 30，选择本身不写设备，运动时按门禁写入）；运动时不能选择 0 作为移动速度。`yz_aim_setup` 的 `s` 可设置轮廓速度 **0..3000 电机 RPM**，仍需已有的禁止输出、静止、报警和读回门禁；这不改变加速度、电流、节点或波特率设置。

第 7 页的**带电池多圈记忆选件**位置范围为 **−60000..+60000 电机圈**，按手册例子的 32768 电机脉冲/圈是 **−1966080000..+1966080000 电机脉冲**。按用户给定的本机减速比 **51:1**，输出轴约 **±1176.470588 圈 / ±423529.411765°**（只表示从零坐标的理论换算，不表示允许的机械行程）。`r`/`a` 的相对/绝对输入均以输出轴角度表示，可输入整圈及跨圈角度；`p` 仍输入电机脉冲绝对坐标。绝对目标必须在上述手册坐标范围内；相对输入既须可表示为有符号 32 位增量，又须以**动作前重新读取**的 `0x6064` 实测位置计算，保证结果坐标仍在手册范围内。32 位编码范围本身**不是**厂家的行程范围。本版不再另设 90° 单次软件上限，保留原有身份、报警、限位、速度采样、使能所有权、心跳与停止确认门禁；动作时限按大行程/高速度适配。电机实际是否安装电池多圈选件、零位、机械行程和负载保持能力均未确认；软件坐标限制不能代替物理行程限位。普通 `e/d`、空格/`x` 停止和 `q` 退出操作保持不变。

位置动作预算按 `5000 ms + 2 × ceil(所选电机RPM × 1000 / 500) + ceil(abs(目标脉冲差) × 60000 / (所选电机RPM × 32768))` 计算（单位 ms；两个斜坡按工具使用的 500 RPM/s 估算）。STOP 动作给 10 s 预算，其中静止回读最多等待 7 s；这只是在软件上给减速与 SDO 留时间，不证明机械已安全停下。点动动作另留运动时长、设置时间和 STOP 预算。

1.5.0 Debug/Release AArch64 `gen_version` 与六项 YZ 目标交叉构建成功，生成 `APP_BASE_VERSION=6.20.0`。Release 在 TQ3588 上仅执行不加载真实 USB 的合成回归：protocol **34** 项、driver **171** 项、fake SDK **34** 项、TUI policy **PASS**、PTY **22/22**（41.225 s）。80×24 模拟画面显示选定 **3000** 电机 RPM、目标 **1672368** 与持续更新的实测 **929626** 电机脉冲（位置采样年龄 **31 ms**），主按键可见。初次失败来自测试夹具 120 ms 延迟、旧 31 RPM 边界及审计/提示断言；仅修正测试后，生产工具 SHA-256 未变，以最终通过记录为准。

八项 Release 工件已隔离发布在 `/root/km2/yz-aim-20260927/release-v1.5.0/`，发布前后逐项 SHA-256 均匹配，清单 SHA-256 `91ab2d8953d246de44843698fea89a102a3fe51665e585208d0b5ab5f00e1f6b`，两工具 `--version` 均为 1.5.0。证据见工作区 `logs/yz-aim-20260928/release-v1.5.0-{board-mock,board-driver-complete,board-ui,board-pty-final,final-deploy}.log` 及 `release-v1.5.0-captures/`。测试板 boot `f58baff2-2b98-4d00-9ba5-181ad7835e77`、Controller inactive、无当前 YZ 工具进程；原 1.4.0 工件 SHA 未变，本任务产生的模拟锁 250/251 和板端抓图临时文件已清理。未访问真实 USB/CAN、未执行电机运动或服务操作；掉电多圈记忆和机械行程仍待现场验证。以下旧版章节保留当时的操作和验证历史。

## 1.4.0 速度选择、运动中位置刷新和简化操作

在测试板先退出旧版 `yz_aim_test`、确认 `km2-controller.service` 未运行，再启动本版：

```sh
./yz_aim_test --bitrate 1000000 --abit-timing 0x01400106 \
  --node 1 --lys 0 --channel 0 --library /usr/local/lib/liblyscan.so
```

启动只读；按 `e` 检查并使能，`v` 输入下次运动的 **1~30 电机 RPM**，`r` 输入相对输出轴角度（正负、可含小数），`a` 输入绝对输出轴角度，回车执行。`v` 仅更新本次会话的选择，不立即写入设备；启动运动时将所选速度写入 `0x6081`。界面同屏显示**实测 `0x6064` 位置**、动作目标、选定速度和位置采样年龄；运动期间由负责心跳的同一 CAN 工作线程逐周期读取 `0x6041/0x260E/0x606C/0x6064` 并发送进度，前端持续重绘。读数受 SDO/适配器延迟约束，停止更新或采样过期不得视为电机静止。

单次运动最多 **90° 输出轴角度 = 417792 电机脉冲**；绝对目标由驱动重新读取实时位置并限幅，不能只凭输入时屏幕数值决定。位置总时间预算为 `5000 ms + ceil(abs(delta_pulses) * 60000 / (所选电机RPM * 32768))`，在 90° / 30 RPM 时为 30.5 s，在 90° / 1 RPM 时为 770 s。`空格`/`x` 请求停止；只有确认保持且静止后才可再次 `r/a`，失败仍锁定安全状态。`d` 停稳后确认失能；`q` 退出时本会话拥有的输出须完成撤使能确认。高级操作留在 `?`：`m/M` 按 `[`/`]` 选 5°/15°/30°/90° 步长、`p` 原始绝对电机脉冲、`j/J` 以选定速度短点动；`R` 手动刷新。setup 的配置热键不变。

1.4.0 Release 已隔离发布到测试板 `/root/km2/yz-aim-20260927/release-v1.4.0/`，板端协议 32、驱动 144、fake SDK 34 项检查通过，TUI policy PASS、PTY 18/18 PASS；这均为不访问真实 USB 的合成回归。八项工件的最终 SHA、模拟画面和发布记录在工作区 `logs/yz-aim-20260928/release-v1.4.0-*`；旧 1.3.0 工件未覆盖。

该程序不会自动清零、回零、断电、重连或证明机械制动。跨圈掉电比较由现场人员在确认行程后分步操作，先按 `d` 确认失能，记录断电前 `0x6064` 和实际轴位，断/上电后在**只读状态**比较原始位置与轴位。每次动作仍需检查方向、独立急停及行程余量。真实 USB/CAN 稳定性、运动和掉电保持尚待当前版现场验证；模拟测试不等于真实电机验收。

## 1.3.0 可见输出角度步长与通信静止确认（已部署，实机待验）

本版本保留 `verify_stopped` 修复：旧路径在收到 `0x6040` 停止保持 ACK 后只读取一次速度，电机仍在滑行时可能误报静止失败，随后停止本会话心跳并出现 alarm `0x0020`。核心工作线程现在最多进行 **1000 ms** 有界静止轮询并继续维护心跳；静止阈值仍为 **0.5 RPM**，心跳安全期限仍为 **700 ms**。这只是软件确认与心跳时序修复，不等同于机械制动或现场安全验收。

为使小步运动在界面上可见，`test` 用 `[` / `]` 选择相对角度步长 **5° / 15° / 30° / 90°**，默认 **5°**；`m/M` 按当前步长执行正/负相对位置。按本设备 **51:1**、32768 电机脉冲/电机圈换算，各步长约为 **23211 / 69632 / 139264 / 417792** 电机脉冲。`p` 仍是 **ABSOLUTE** 绝对目标；单次目标差最大 **417792** 电机脉冲（约 90° 输出角），例如当前 `3,088,730` 选择 5° 时，输入目标 `3,111,941` 表示绝对目标增加约 23211 脉冲。旧版 ±3276 脉冲仅约 0.706° 输出角，本版本以可见角度步长替代该过小的单步限制。`j/J` 保持 ±1 motor RPM、250 ms 点动。

运动仍由用户在本地工具内明确发起，不自动运动，也不支持远程触发。相对/绝对位置动作固定 profile **30 motor RPM**；位置动作预算按 `5000 ms + ceil(abs(delta) * 60000 / (30 * 32768))` 计算，单次最大 90°（417792 电机脉冲）时总预算为 **30500 ms**。普通 STOP 与 setup 操作预算为 **5 s**。完整跨多圈仍需多次操作并确认实际行程余量，软件上限不代表物理行程或保持能力。

界面保留第一关键失败和后续报警，二者分开保存/显示，并在退出按 `q` 后打印 `Result.context`：SDO 对象、处理阶段、已用时间和预算。退出后重新打开新的 SDO 会话是诊断要求；重新连接或 USB reset 都不能单独证明 USB 故障已定位。`0x2601` 读回只按 bit 0 判断输出状态：raw `4`、`6` 为 disabled，raw `5`、`7` 为 enabled；高位含义仍未定义，界面保留完整 raw 值，写入仍只使用 canonical `0/1`。

本地 SDK 反汇编确认 `OpenUSBDevice` 在 `27190` 调用 `libusb_reset_device`，`CloseUSBDevice` 在 `27364`；证据仍见 `logs/yz-aim-20260928/release-v1.2.1-sdk-reset-evidence.log`，该文件名属于 1.2.1 预发布分析。最终 1.3.0 工件日志使用 `release-v1.3.0` 命名；靠近 open/close 的 `dmesg` reset 记录不足以单独诊断硬件原因。

Debug/Release AArch64 六个 YZ 目标交叉构建成功，版本头为 6.18.0；Release 目标板仅模拟：协议 31 项、驱动 111 项、fake SDK 34 项、TUI policy PASS、PTY 16/16 PASS（16.314 s）。最终 8 件工件部署到 TQ3588 `/root/km2/yz-aim-20260927/release-v1.3.0/`，发布前后 SHA-256 逐项一致，两 CLI `--version` 均为 1.3.0。证据位于工作区 `logs/yz-aim-20260928/release-v1.3.0-board-ui-final.log`、`release-v1.3.0-board-driver-final.log`、`release-v1.3.0-final-deploy.log` 及 `release-v1.3.0-SHA256SUMS`。80×24 模拟界面已核对步长、失能键和绝对目标示例。Controller 保持 inactive；用户原有真实工具进程保留，本任务没有发送真实电机命令。当前版本真实通信、运动、精确物理位移及掉电保持仍待用户验证。

多圈掉电验证可选 15°（69632 脉冲，2.125 电机圈）：确认现场允许该行程后，`e` 使能，按 `]` 选 15°，按 `m` 完成一次位置运动。`d` 成功失能后记录原始 `0x6064` 和轴位，退出再由现场人员断电、上电；重开工具先只读比较位置和轴位。不得用仅有发送应答或模拟结果判断本机掉电记忆能力。

## 1.2.0 有界绝对位置与点动修复

`yz_aim_test` 新增 `p`：输入**有符号十进制绝对位置**（`0x6064` 的电机脉冲，例如 `-1200`），每次执行前在工作线程重新读取设备身份、当前位置、转速、输出和报警。与新读数相差 0 或超过 **3276 电机脉冲**（约 0.1 电机圈、按实际 51:1 为约 0.7059 输出度）的目标会被拒绝，不下发位置指令；小步可重复执行，不会自动回零、重置多圈原点、保存参数或在重连后继续运动。`e` 使能和退出 `d` 失能的既有要求仍适用；`m/M` 相对 ±100 与 `j/J` 点动保留。点动的 250 ms 期限仅决定何时停机，不再截断该时刻在途的状态读取；完整遥测采样在设备归本会话使能期间逐项维护心跳，采样失败会锁定当前会话、停止心跳并尝试一次**未确认**的停止帧。USB 实际断线无法靠此修复，无法收到停止确认时须按现场停止措施处理。

**多圈与掉电验证（现场人工操作，当前版本没有实机结论）：**先确认空载、行程余量、独立急停及电机版本；打开工具读取并记下 `0x6064` 原始位置。按 `e`，再逐次按 `p` 输入距界面当前位置不超过 3276 的新目标，检查每步“绝对位置：成功”、静止后的目标位置和状态；若要区分单圈与多圈，累计至少跨过 **32768 电机脉冲**（同方向走 11 次 3276 脉冲约 7.76 输出度）。逐次确认实际机械允许范围，不能把软件单步限幅当作整机行程限位。结束按 `d`，必须看到“输出失能：成功”、最低位为 0 且转速静止，再记下断电前原始 `0x6064` 和报警，退出工具，由现场人员断电、重上电。重新启动只读工具，在按 `e` 或任何运动键**之前**记录新 `0x6064`/报警并比较；如果上电自动寻零功能已配置，设备可能自行运动，需先核对配置及现场安全。两次读数接近且明确跨一电机圈只说明这台电机在该次掉电后保持位置读数，还需结合实际轴位和断电时轴有无被带动判断闭环与保持能力。手册说普通版编码器上电显示单圈位置；**带电池或刹车多圈选件**才描述上电读回多圈位置，界面的电量字段不能单独证明选件存在。

1.2.0 已于 2026-09-28 隔离部署到测试板 `/root/km2/yz-aim-20260927/release-v1.2.0/`，八项工件 SHA-256 在部署前后均通过，Release 板端协议 26 项、驱动 90 项、假 SDK 34 项、界面策略 PASS、PTY 13/13 PASS，均为**不访问 USB 的模拟测试**。旧 1.1.0 位于独立目录、未覆盖。当前仍**没有** 1.2.0 真实 USB-CAN、电机、断电或掉电后读回验收。

## 1.1.0 中文操作界面

`yz_aim_setup` 和 `yz_aim_test` 进入后默认只读；真实设备须按现场已核对的速率、节点和 LYS 模块参数启动。界面状态、读数、帮助和常见失败提示均显示中文，原始 `0x2601` 数值及电机侧/输出侧单位仍保留供核对。本设备按实际减速比 **51:1** 换算。

| 程序 | 按键 | 操作 |
| --- | --- | --- |
| test | `e` / `d` | 一键完成安全检查并使能 / 停止、确认静止后失能 |
| test | `m` / `M`（或 ↑ / ↓） | 各相对位移 +100 / −100 电机脉冲；运动前重新检查安全条件 |
| test | `j` / `J`（或 → / ←） | +1 / −1 电机 RPM，点动 250 ms |
| test | `p`（1.2.0 起） | 输入有符号绝对位置；重新读取当前位置后，单次限幅 ±3276 电机脉冲 |
| setup | `s` / `a` / `c`（1.1.0 历史） | 输入速度（1~30 电机 RPM）、加速度（10~1000 电机 RPM/s）或电流（10~10000 mA、10 mA 整数倍），回车设置并核对读回；不再需要二次确认 |
| setup | `n` / `b` | 输入节点或波特率后回车，再回车确认暂存；变更需断电重启并重新连接 |
| 两者 | `空格` / `x`，`r`，`?`，`q` / `Esc` | 请求停止、刷新、帮助、退出；设置/确认框的 `Esc`/`q` 取消，空格/`x` 请求停止 |

按 `e` 只做使能，不会自行运动；运动仍要求本会话先成功使能，驱动保持新鲜采样、报警/限位/静止检查、心跳所有权及有界速度/行程。退出已拥有输出的会话时仍执行撤使能并核对；通信参数的二次确认在 `Esc`/`q` 取消时绝不下发。2026-09-28 已将 Release 版工具、策略程序和 mock PTY 脚本部署到测试板 `/root/km2/yz-aim-20260927/release-v1.1.0/`，板端 SHA-256 逐项校验通过、工具 `--version` 返回 1.1.0；尚未进行本版本板端策略/PTy 回归或真实运动验证。下文 1.0.2 的板端记录只属于 2026-09-27 旧版本。

## Build

Build only with the repository's AArch64 cross-build:

```sh
./build-cross.sh Debug
cmake --build build-cross/debug --target yz_aim_setup yz_aim_test yz_aim_tui_policy_test -j"$(nproc)"
./build-cross.sh Release
cmake --build build-cross/release --target yz_aim_setup yz_aim_test yz_aim_tui_policy_test -j"$(nproc)"
```

Do not configure a host-native device build or run these AArch64 programs on the build host. The frontend targets are present when the shared maintenance TUI/FTXUI option is enabled. The mock transport can be used on the authorized AArch64 test board without loading the vendor library or accessing USB:

```sh
python3 drivers/yz_aim/test/test_yz_aim_tui_virtual.py \
  --setup build-cross/debug/bin/yz_aim_setup \
  --test build-cross/debug/bin/yz_aim_test
```

Replace `debug` with `release` for the Release flavor. The PTY runner checks CLI parsing, resize/help/terminal restoration, mock read-only behavior, simplified setup keys, communication-setting cancellation, owned heartbeat, bounded motion keys, and STOP semantics. Run it only on the authorized AArch64 board; it launches the supplied binaries. Optional `--capture-dir DIR` saves raw ANSI frames at 80×24, 120×36, and 160×50 for visual review. The earlier Debug/Release AArch64 board regression passed after the PDO-backlog, signed-speed, and cleanup-cancellation fixes: protocol 26 checks, driver 53 checks, fake SDK 34 checks, and UI policy PASS in each flavor; the PTY runner passed 9/9 cases in Debug (11.896 s) and Release (10.334 s). The six compact mock PTY captures cover setup/test at 80×24, 120×36, and 160×50; the main session visually reviewed the 80×24 setup and 120×36 test screens. These are earlier-revision results, not 1.0.2 acceptance. Core 1.0.2 and the updated UI were cross-built in Debug/Release; this round’s AArch64 Debug/Release core regressions passed (protocol 26 checks, driver 75 checks, fake SDK 34 checks and UI policy PASS per flavor). The final 12-case PTY runner passed in both flavors (Release 12/12 in 15.816 s; Debug 12/12 in 17.716 s), after its Python helper was fixed to read the last complete ANSI frame. Business binaries were unchanged; the earlier partial-frame failures are superseded by the final run. A bounded 1.0.2 real-board session observed encoder movement for +100 and −100 raw-pulse commands and confirmed output disable; this does not establish mechanical travel calibration, loaded motion or safety certification.

## Run

Both programs default to the full-screen TUI and require a TTY for interactive operation. `--help` and `--version` exit without opening a transport.

```sh
# Synthetic data; displays MOCK / NO HARDWARE and never loads LYS/USB.
./yz_aim_setup --mock
./yz_aim_test --mock

# Real adapter; bitrate must be selected explicitly.
./yz_aim_setup --bitrate 1000000 --abit-timing 0x01400106 \
  --node 1 --lys 0 --channel 0 --library /usr/local/lib/liblyscan.so
./yz_aim_test --bitrate 1000000 --abit-timing 0x01400106 \
  --node 1 --lys 0 --channel 0 --library /usr/local/lib/liblyscan.so
```

Options are `--bitrate 125000|250000|500000|1000000`, `--node 1..127` (default 1; node 127 is read-only), `--lys INDEX` (default 0), `--channel INDEX` (default 0), `--library PATH` (default `liblyscan.so`), `--abit-timing WORD`, `--mock`, `--tui`, `--no-mouse`, `--help`, and `--version`. `--abit-timing` accepts decimal or `0x` hexadecimal. The SDK demo word `0x01400106` is the default only for 1 Mbit/s. It completed read-only classic-CAN SDO requests on the specific board/device combination below; this does not establish write or motion safety, or support for other bitrates. Other rates require a separately verified timing override. A selected bitrate is an operator input, not a generic hardware-support claim.

On target hardware, `liblyscan.so` and `libusb-1.0.so.0` must be available to the runtime loader. The driver loads target libusb globally before loading the vendor library. Opening the adapter and taking a snapshot are read-only. The frontend does not start/stop Controller or otherwise manage services.

Read-only board verification completed on `192.168.1.148`, boot ID `2871ffc8-0bf8-4de6-b591-ee8aa984ca50`, using the LYS `3068:0009` adapter at device 0/channel 0, node 1, classic CAN 1 Mbit/s with nominal timing `0x01400106`. Identity matched device type `0x20192`, vendor `0x331`, product `1`. The OD baseline reports `0x2601=7` (`workspace-root logs/yz-aim-20260927/real-readonly-od-baseline.log`). During this session the vendor clarified that readback uses bit 0 for output state: raw `7` is enabled and raw `6` is disabled. Higher bits carry additional information whose individual meanings are unspecified here; the UI preserves the complete raw value, and the driver writes only canonical `0` or `1`, never a readback value. The earlier real-board TUI run showed UNKNOWN and rejected ARM under the then-current policy; that read-only evidence remains historical and did not send any control write. The initial baseline was SDO-upload-only (15 requests), followed by SDK stop/close; it sent no controlled write, NMT, heartbeat, or motion command. The final real-board read-only polling session read for 5 seconds, held the ARM confirmation modal for 35.005 seconds, correctly rejected ARM because `0x2601=7`, then resumed complete reads for 2.3 seconds without a queue-drain or SDO-padding error; it exited rc 0 with termios restored. Its last frame showed 24.78 V, 25 °C, alarm 0, speed 0.0 motor RPM, current 0.004 A, position 3,088,644 motor pulses, battery 60%, and output UNKNOWN (raw `0x0007`). A separate Release setup run at 80×24 read for 5 seconds, entered the profile-speed confirmation dialog, canceled it with `q` without sending `SET-SPEED`, refreshed successfully, and exited rc 0 with termios restored. That historical read-only session sent no controlled writes, NMT, heartbeat, enable, or motion. Later isolated diagnostics exercised STOP, output disable, and a single heartbeat; see `workspace-root logs/yz-aim-20260927/motion-diag-disable-wire.log` and `motion-diag-heartbeat-wire.log`. See `workspace-root logs/yz-aim-20260927/real-test-final.log`, `real-test-final.ansi`, `real-setup-final.log`, and rendered frames `real-test-final-frame.png` / `real-setup-final-frame.png`.

## Version 1.0.2 bounded board observation

On 2026-09-27, the Release `yz_aim_test` on the same boot (`2871ffc8-0bf8-4de6-b591-ee8aa984ca50`) started at position 3,088,738 pulses with output raw6 and alarm `0x0020`. ARM and the guarded ENABLE returned OK. At enabled idle and through 2.2-second motion confirmation waits the observed alarm remained zero. A commanded +100 showed position 3,088,853 (+115 observed), and the later commanded −100 showed 3,088,735 (−118 from that reading, −3 against the initial reading). Position settled to 3,088,730 after confirmed DISABLE. Both relative commands returned OK; these samples establish encoder change and bounded control only, not exact motion calibration or visible output mechanics. STOP/DISABLE completed; final UI output was raw6 and speed 0.0 motor RPM, with exit code 0 and termios restored. A subsequent read-only wire check found controlword 0, output raw6, position 3,088,730, speed raw1 (=0.1 motor RPM, within the driver’s stopped threshold), and alarm `0x0020` after restoring the original consumer `0x007f03e8` and retiring the owned producer heartbeat. Communication alarm while **disabled** is expected with that pre-existing consumer; it is not evidence that output remained enabled. `LCAN_Stop` and `LCAN_CloseDevice` returned 1. No Controller deployment or service change was performed. Evidence: `workspace-root logs/yz-aim-20260927/motion-fix-v102-real-acceptance.log`, `motion-fix-v102-final-wire.log`, and `motion-fix-v102-board-regression.log`; final PTY and artifact SHA evidence: `motion-fix-v102-final-pty.log`.

Final board audit on the unchanged boot found Controller inactive, no YZ-AIM/Controller process and no USB owner; the LYS adapter was present after SDK close at `/dev/bus/usb/003/026` (the previous enumeration `/003/019` no longer existed). The task-created mock lock 2 was removed after confirming it unowned, while pre-existing lock 0 was preserved. Release `yz_aim_setup` and `yz_aim_test` were each 543,216 bytes, with SHA-256 `9db4e6999131a038d5f2bb144cd70967185a14d0c39e6c974e49da8e4ab77872` and `88575cd1594522ef62cbb97434f770bbdc0819a623ed87d24eeeb3fb09408fdb` respectively. All 15 artifacts matched the board manifest. Final audit: `workspace-root logs/yz-aim-20260927/motion-fix-v102-final-board-audit.log`; board evidence was fetched under `motion-diag-board/motion-diag` with selected real frames in `motion-v102-frames`. SSH-master closure and host temporary-file cleanup remain pending the parent’s confirmation.

## Operator behavior and units

When a fresh `0x2601` readback has bit 0 clear and the other core gates pass, the setup screen exposes verified volatile profile speed, acceleration, and current-limit readback, plus manual node-ID and bitrate staging. Ordinary `s/a/c` parameter writes need only numeric input and Enter; node/bitrate staging retains a separate Enter confirmation, cancelable by `Esc` or `q`. Volatile profile values are motor-side RPM and RPM/s, and current limit is mA. Staged node/bitrate changes show the old/new values and require a manual power cycle and a new read-only connection; bitrate migration follows the documented §7.13 sequence and adds no generic save command. Refresh never writes or saves. Higher bits do not change the disabled/enabled decision; configuration that requires disabled output remains gated on a fresh sample with bit 0 clear.

The test screen uses `e` to run the existing read-only ARM safety check and then ENABLE in the single CAN worker; each motion key repeats ARM before bounded motion and requires an already owned ENABLE. ARM accepts only exact communication-loss alarm `0x0020` as a recoverable alarm, provided identity, stationary speed, position, limits and output evidence are fresh; ARM itself sends no write or heartbeat. Explicit ENABLE first acknowledges STOP `0x010F` and verifies stationary speed, then sends `0x0100` to clear operation bits and verifies output bit 0 is disabled. It zeros the old speed target and clears the old position target to observed actual position **before** starting its owned heartbeat. Only after alarm `0x0020` clears does it write `0x000F`, canonical `0x2601=1`, and verify enabled bit 0 and stationary state. Other alarms do not enter this recovery path. While the output remains owned, the single motor worker services the heartbeat about every 200 ms, even while idle or awaiting confirmation; movement completion and explicit STOP keep that heartbeat running. The manual warns that a heartbeat resumed after timeout can restart an old motion, so an overdue heartbeat latches unsafe instead of being sent late. The tool offers direct `m/M` and `j/J` keys for bounded positive/negative moves and jogs, Space or `x` for STOP, `d` for output disable, and `q`/Esc for exit. STOP and output disable are separate operations and do not certify a mechanical brake. On owned-output exit, cleanup verifies held/stationary status, sends `0x0100` to clear operation bits, writes canonical `0x2601=0`, verifies output bit 0 is disabled, then retires its heartbeat and restores the previous watchdog consumer. The real bench read back `0x6040=0x0000` and `0x2601=0x0006` after `0x0100`; writing only `0x2601=0` while operation bits stayed set had acknowledged without disabling output. Read-only and merely armed exits remain write-free; an external output is not silently adopted.

The UI shows motor-side raw pulses/RPM and output-side angle/RPM. The 51:1 reduction is the actual ratio confirmed by the user for this device; it is not a generic YZ-AIM default. The manual's CAN position example uses 32768 counts per motor turn, giving 1,671,168 raw counts per output revolution at 51:1. Output angle is inferred from that example; confirm counts per output turn on the physical unit. This conversion does not guarantee reducer accuracy or backlash.

Protocol objects, exact configuration sequences, watchdog behavior, and remaining physical-verification limits are documented in [docs/protocol.md](docs/protocol.md).

单个运行脚本：
```sh
cd /root/yz_aim && python3 -B single_axis_cycle_test.py --node 1 --minutes 30
```