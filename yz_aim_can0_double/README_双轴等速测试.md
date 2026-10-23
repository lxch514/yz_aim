# CAN0双轴等速测试

两个电机均接can0，节点号分别为1、2，波特率1 Mbit/s。伸出为节点1负转、节点2正转；缩进反向。

1.2.3-start-confirm修正：两节点的启动写确认同时接收。仅速度模式启动`0x6040=0x000F`的ACK缺失时，在原运动截止时间内逐轴读回模式、目标、状态、报警、输出和实际转速；全部符合才继续，不重发启动命令，并在CSV/中文日志中注明读回确认。读回失败、不符或SDO拒绝仍停止。故障锁止后仅允许停止、失能、心跳消费者恢复及状态读取，禁止重新使能或运动，也不恢复已故障的主站心跳。摘要中的`start_readback_verified_actions`和`start_confirmation_failures`分别记录这两类结果；ACK等待约300ms和独立监测500ms阈值是不同指标。

默认程序仍保留原来的位置同步保护。测试时显式选择`sync-mode speed`：两轴目标转速等幅反向，位置差只记录，不参与修正或停机。电机本身速度环运行；实际速度可能有小幅差异，查看CSV和报告中的实测值。驱动报警、限位、方向/超速、通信和反馈失效保护保留。

交互控制：

```sh
cd /root/yz_aim_can0_double
./build/yz_aim_can0 --if can0 --node 1 --node2 2
```

依次输入：

```text
sync-mode speed
sync-dir 2 1
sync-enable
sync-zero
extend 300 3000
retract 300 3000
sync-status
sync-disable
quit
```

`sync-zero`只记录本会话的软件位置基准，不移动电机、不写硬件零点。`sync-stop`停止两轴；`sync-mode position`在停止后恢复原位置同步模式。

运动反馈按100ms周期发起；双轴SDO事务本身耗时，繁忙或重试时周期会变长。`--timeout 3000`是准备/状态命令总预算，不是轮询间隔；单条SDO通常仍有100/120ms等待窗口。每个节点SDO事务间隔至少10ms，直接停止帧不受该间隔影响。CAN0内核波特率与启动参数不一致时，程序拒绝发送SDO或运动指令。

1分钟自动测试（300 RPM，每方向3秒）：

```sh
python3 -B dual_cycle_test.py --rpm 300 --move-ms 3000 --minutes 1
```

仅在允许连续运行且行程足够时，显式将`--minutes 1`改为`--minutes 30`。脚本不会自动升级为30分钟测试。Ctrl+C执行停止、失能并记录结果。

全部输出在`logs/parallel-30min-年月日-时分秒/`，其中有`can-frames.log`、`controller.log`、`test.log`、每动作`motion-*.csv`、`summary.json`、`analysis.json`以及中文`YZ-AIM_两轴通信统计_*.md`。目录前缀表示与原测试相同的组织格式，实际时长由参数和报告说明。分钟统计写入test.log，异常即时写入。

`can-frames.log`保留所有应用可见CAN收发帧；TX与RX数量不必相等。SDO成功率、明确的驱动/套接字丢弃和CAN错误分别统计，不能由TX−RX计算丢帧，也不将应用事务成功解释为物理总线绝对无丢帧。

重新生成已有测试报告（不控制电机）：

```sh
python3 -B dual_test_report.py logs/parallel-30min-实际时间戳
```
