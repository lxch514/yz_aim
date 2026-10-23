# yz_aim 单文件夹编译与运行说明

本文件说明：**手上只有 `yz_aim/` 这一个文件夹（没有完整控制器仓库）时**，怎么编译、
编出来的是什么、以及怎么接真实电机跑起来。原始 `README.md` 是完整仓库视角的说明，
其中提到的 `yz_aim_setup` / `yz_aim_test` 全屏界面工具在本文件夹下**编不出来**，原因见下文。

---

## 一、这个文件夹里能编出什么

| 目标 | 来源 | 依赖 | 说明 |
|---|---|---|---|
| `yz_aim_cli` | `tools/yz_aim_cli.cpp` | `include/` + `src/` | **本次新增**。命令行前端，唯一能直接接真实电机的交互工具 |
| `yz_aim_driver` | `src/yz_aim.cpp` `src/lys_transport.cpp` | `include/` `sdk/` | 静态库，全部协议与安全门禁都在这里 |
| `yz_aim_fake_lys` | `test/fake_lys_sdk.cpp` | — | 动态库，给传输层测试当假 SDK |
| `yz_aim_protocol_test` | `test/yz_aim_protocol_test.cpp` | mock | 协议编解码自动测试 |
| `yz_aim_driver_test` | `test/yz_aim_driver_test.cpp` | mock | 驱动全流程自动测试 |
| `yz_aim_transport_test` | `test/yz_aim_transport_test.cpp` | fake SDK | 传输层自动测试 |

**编不出来的目标**（`CMakeLists.txt` 里被 `if(TARGET maintenance_tui)` 保护，条件不成立自动跳过，
不会报错）：

- `yz_aim_setup`、`yz_aim_test`（两个全屏 TUI 工具）
- `yz_aim_tui_policy_test`

原因：`tools/yz_aim_tui.h` 和 `test/yz_aim_tui_policy_test.cpp` 都 `#include "maintenance_tui.h"`，
那是父仓库提供的 FTXUI 显示层封装。本文件夹只带了上面那一层壳，缺底层实现，因此**无法脱离仓库编译**。
`yz_aim_cli` 就是为补这个缺口写的：同样的安全门禁、同样的命令语义，纯终端逐行输入，不依赖 FTXUI。

---

## 二、编译方式

### 方式 A：板上原生编译（推荐）

在板子上（TQ3588，Ubuntu）执行：

```bash
cd /root/yz_aim
bash build.sh
```

- 首次需要工具链：`apt update && apt install -y build-essential cmake`
- 产物：`build-native/release/bin/yz_aim_cli`
- 想编 Debug：`bash build.sh --debug` → `build-native/debug/bin/yz_aim_cli`
- 想只编某一个目标：`bash build.sh --only yz_aim_cli`
- 想清干净重来：`bash build.sh --clean`

### 方式 B：x86_64 宿主机交叉编译，再拷到板上

宿主机需要：`sudo apt install -y cmake g++-aarch64-linux-gnu`

```bash
cd <宿主上的路径>/yz_aim
bash build.sh --cross
# 产物：build-cross/release/bin/yz_aim_cli
scp build-cross/release/bin/yz_aim_cli root@<板子IP>:/root/yz_aim/
```

### 方式 C：不用 CMake，一条 g++ 命令

```bash
cd /root/yz_aim
g++ -std=c++17 -O2 -Iinclude -Isrc \
    tools/yz_aim_cli.cpp src/yz_aim.cpp src/lys_transport.cpp \
    -o yz_aim_cli -ldl -lpthread
```

> 注意：`src/lys_transport.cpp` 里写的是 `#include "../sdk/lyscan_abi.h"`（相对路径），
> 所以**必须保持文件夹结构不变**，不能把 `src/*.cpp` 单独拷出来编。
> 这也正是你要求「文件和文件夹名字都不变」的原因。

### 跑自动测试（可选，不需要真实适配器）

```bash
cd /root/yz_aim
bash build.sh
ctest --test-dir build-native/release --output-on-failure
```

---

## 三、运行 `yz_aim_cli` 的前提

1. 适配器（LYS USBCANFD-Mini）插在**板子的 USB Host 口**上，`lsusb | grep 3068` 应能看到
   `3068:0009`。是板子上的程序自己去开 USB 设备，不是「PC 通过 USB 控制板子」。
2. 运行期动态库存在：
   - `liblyscan.so`（常见路径 `/usr/local/lib/liblyscan.so`）
   - `libusb-1.0.so.0`
3. **必须先停掉控制器服务**，否则两个进程抢总线：

   ```bash
   systemctl stop km2-controller.service
   systemctl is-active km2-controller.service   # 应为 inactive
   ```

   `yz_aim_cli` 启动时会自己检查一次，若服务仍在跑会直接退出（返回码 3）。

---

## 四、运行

```bash
cd /root/yz_aim
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

./build-native/release/bin/yz_aim_cli \
    --bitrate 1000000 \
    --abit-timing 0x01400106 \
    --node 1 --lys 0 --channel 0 \
    --library /usr/local/lib/liblyscan.so
```

进去之后是逐行命令，`help` 看命令表，`quit` 退出。启动即只读，不会自己写控制字。

常用命令：

```
status                 只读，读一次完整快照
monitor 10 500         只读，刷 10 秒、每 500 ms 一次
read-od 0x1000:0 4     只读，读设备类型（应为 0x20192）
enable                 使能输出（之后本进程维持 0x77F:05 心跳）
rel 90                 输出轴相对转 +90°
abs 0                  输出轴绝对回 0°
jog 200 500            速度模式点动 200 电机RPM、500 ms
stop                   急停保持
disable                停止并失能
quit                   退出（退出前会自动确认失能）
```

---

## 五、已知限制（如实声明）

- 本文件夹内容**未在编写机上编译验证**（编写机没有 C/C++ 编译器）。已做的检查是：
  括号/花括号平衡、换算与时间预算公式复算、行尾为 LF。第一次在板上 `bash build.sh`
  若报编译错误，把完整报错贴回来即可。
- `yz_aim_cli` 没有内置模拟设备（TUI 里的 mock 没随本文件夹提供），只能接真实适配器。
- `--node 127` 是只读节点：`enable`/`disable`/`stop`/运动/写内部参数等命令一律拒绝，
  只放行 `status`、`monitor`、`read-od`。等待输入期间仍会维持心跳。
- 波特率只有 1 Mbit/s 有厂商时序字 `0x01400106`；其它速率必须显式给 `--abit-timing`，
  否则工具会拒绝打开适配器（这是刻意设计，避免自己乱猜时序把总线搞坏）。
