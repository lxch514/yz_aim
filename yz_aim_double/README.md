# YZ-AIM USB-CAN双轴镜像程序

## USB-CAN启动与当前缺项

2026-10-21板上已枚举 `3068:0009 LYS USBCANFD-Mini`，但未找到真实厂家
`liblyscan.so`，所以目前完成源码与编译、模拟回归，尚不能真实打开USB CAN通道。
不要把 `test/libfake_lys.so` 当真实驱动使用；它仅供测试。

需要原适配器厂家提供的 **Linux AArch64/ARM64** `liblyscan.so`（不是Windows DLL或x86库）。
将真实库放到 `/root/yz_aim_double/lib/liblyscan.so` 后，程序会自动优先使用它；
也可通过 `--library /原有路径/liblyscan.so` 指定。libusb-1.0.so.0板上已存在。
不需要修改/usr/lib、/usr/local/lib或运行ldconfig，不修改原/root/yz_aim。

```sh
cd /root/yz_aim_double
./build/yz_aim_double --node 1 --node2 2 --lys 0 --channel 0 --bitrate 1000000
```

显式路径的等价命令：

```sh
./build/yz_aim_double --library /root/yz_aim_double/lib/liblyscan.so --node 1 --node2 2 --lys 0 --channel 0 --bitrate 1000000
```

默认1M采用原/root/yz_aim的厂家示例仲裁时序0x01400106，仅使用经典CAN。
其他波特率必须同时提供已验证的 `--abit-timing`，不推算未知时序。
适配器只打开一次，两轴共享物理句柄、各有SDO接收队列，B轴先回复不会被A轴读掉。
周期PDO/心跳按COB-ID保留最新值，SDO应答不丢弃；实际SDK读入的帧在路由前打印。
同一USB适配器不要与原单轴程序或其他工具同时打开。

需要另存逐帧日志：先 `mkdir -p /root/yz_aim_double/logs`，然后在bash运行：

```sh
./build/yz_aim_double --node 1 --node2 2 --bitrate 1000000 2> >(tee -a /root/yz_aim_double/logs/can-frames.log >&2)
```

只接目标电机时改节点（原ID=1，改为2）：

```sh
./build/yz_aim_double --node 1 --node2 0 --bitrate 1000000 --yes set-node 2
```

按程序提示确认保存后给电机重新上电，再使用新ID。USB版不需要启动can0。

# 双轴镜像控制与CAN诊断（2026-10-21，1.2.0-trace）

默认伸出 extend：节点1负、节点2正；retract反向。速度单位为电机RPM，时间单位为毫秒。
双轴速度范围1~300 RPM，时长1~600000 ms。只运行与当前接线对应的一种程序。

启动后为只读。接好两台不同节点号的电机、确认当前机构位置后，在程序提示符内依次输入：

```text
sync-status
sync-enable
sync-zero
extend 30 1000
retract 30 1000
sync-status
```

低速短行程确认后，按实际行程选择命令，例如：

```text
extend 300 45000
retract 300 45000
sync-status
sync-disable
quit
```

extend/retract为阻塞定时动作：结束后再输入下一条；运行中Ctrl+C要求两轴停止。
常转可用 `sync-run 100`（伸出）或 `sync-run -100`（缩进），随后输入 `sync-stop`。
定时到达时先向两轴发送停止帧再等确认；实际机械减速停稳比停止请求晚，分别记入CSV。

## 调整零点

`sync-zero`在两轴静止时记录当前固定软件零点，不移动电机，不改硬件编码器零点。
应先让机构处于正确初始位置/两端齐平，再 `sync-zero`。它只记基准，不自动回零或找限位。
同一会话内extend/retract保留该零点，退出重启后需要重新设置。

共同微调初始位置：先 `sync-enable`、静止时 `sync-zero`，用低速短时
`extend 10 200` 或 `retract 10 200` 调整，停止后在选定初始位置再次 `sync-zero`。
需要重新记零时：`sync-stop` → 确认机构齐平并静止 → `sync-zero` → `sync-status`。
不要用重新置零隐藏已存在的机械错位。

单独校平一轴时，仅在机械条件允许单侧独立运动时使用：
`one 1 5 200`（节点1正向5RPM、200ms）或 `one 2 -5 200`，停止后再检查并 `sync-zero`。
这两条的速度符号是节点自身方向；单独校平会改变两端的相对位置。

若确实需要把设备编码器绝对值写成零，先 `sync-disable`、`quit`，再用单轴参数
`--node 1 --node2 0 --yes set-zero`，另一台用 `--node 2 --node2 0 --yes set-zero`。
`set-zero`改硬件坐标，与通常使用的 `sync-zero` 不同；它要求输出已失能、读回静止。
硬件置零后重启双轴会话并重新 `sync-zero`。软件零点不能直接用于 `sync-abs 0`，
因为sync-abs使用A轴设备绝对编码器坐标，不是sync-zero的相对行程。

## 收发帧与偏差记录

默认连续输出CAN诊断到stderr；交互空闲时也会被动读入节点帧，不额外发送探测帧。
逐帧格式示例：

```text
[CAN can0 +1.234567s #18] TX       ID=601 DLC=8 40 64 60 00 00 00 00 00
[CAN can0 +1.235012s #19] RX       ID=581 DLC=8 43 64 60 00 00 00 00 00
[CAN STAT can0] TX=100 RX=95 TX_ERR=0 KERNEL_RX_DROP=0 TRACE_DROP=0 OUTPUT_DROP=0
```

时间为本进程启动后的单调时钟秒数，时间戳对应传输层调用完成/接收读出，并非总线硬件时间。
`TX`只表示内核/USB SDK接受了发送请求，不等于电机已收到或执行。需结合相应RX/SDO诊断。
0x601/602为发往节点1/2的SDO，0x581/582为应答，0x77F为共享心跳（单向发送，不期待一帧一答）。
因此TX和RX总数不应强行相等。读请求超时时保留有界重试；无确认的写不盲目重发。

- `TX_ERR`：传输层发送失败。
- `KERNEL_RX_DROP`：SocketCAN套接字接收队列溢出累计；只在后续收到内核溢出信息时更新，
  不涵盖所有控制器/线缆丢失。USB SDK没有同等可验证计数，该字段在USB版为0，不代表硬件没丢帧。
- `TRACE_DROP`：异步打印缓冲拥堵时丢弃的诊断记录。
- `OUTPUT_DROP`：非阻塞终端输出失败或不完整的次数。后两项是诊断输出损失，不是CAN丢帧。
- `RX_ERROR`：SocketCAN总线错误帧；板载版仅订阅当前节点SDO/PDO/心跳以及错误帧，
  不是总线上所有设备的抓包器。USB版在节点路由之前记录SDK读到的经典标准数据帧。

终端打印使用独立的有界队列/非阻塞输出；拥堵只增加诊断丢弃计数，不阻塞CAN控制线程。
需要安静输出时在启动参数添加 `--no-trace`；位置偏差CSV仍会生成。
每次速度动作在本程序目录 `logs/motion-*.csv` 保存位置、时间对齐偏差、采样质量、
修正目标、停止请求时间和停稳时间。CSV写入失败仍按真实诊断故障处理。

保持固定零点、默认0.5°累计偏差保护和最大5RPM的小幅减速修正。可以用 `sync-correct 5`
查看/设置修正上限，`sync-tol 0.5`设置允许输出角度偏差。不要为了消除停机直接关闭保护。
真实报警、限位、持续无新鲜反馈、危险累计超差、无确认写仍可能使两轴停止。

## 验证范围

板上编译、65项持续运行检查、95项双轴原语、板载版2084项、USB版2087项真实CLI/假驱动器检查通过。
USB SDK假库36项、USB路由/终端堵塞18项通过。两版实际控制循环各模拟运行45秒、300RPM，
停止帧请求时间45000ms，最大有效偏差0.171257°；另有伸出/缩进50秒控制器模型。
这些测试不访问真实电机，不能据此保证实际负载、机构或接线永不故障。

```sh
cd /root/yz_aim_double && python3 -B dual_cycle_test.py --rpm 100 --move-ms 3000 --minutes 30
```
