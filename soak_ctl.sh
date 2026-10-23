#!/bin/sh
# soak_ctl.sh start|stop|status|tail —— 让老化测试脱离 SSH 终端运行
ROOT=/root/yz_aim
RUNTIME=$ROOT/.runtime
PIDFILE=$RUNTIME/soak.pid
LOG=$ROOT/logs/soak-detached.log

find_pid() {
    ps -eo pid,args | awk '/python3 .*soak_test\.py/ && !/awk/ {print $1; exit}'
}

case "$1" in
    start)
        shift
        if [ -n "$(find_pid)" ]; then
            echo "已经在运行，PID=$(find_pid)"
            exit 1
        fi
        mkdir -p "$RUNTIME" "$ROOT/logs"
        setsid python3 -B "$ROOT/soak_test.py" "$@" > "$LOG" 2>&1 < /dev/null &
        sleep 2
        pid=$(find_pid)
        if [ -z "$pid" ]; then
            echo "启动失败，日志尾部："
            tail -20 "$LOG"
            exit 1
        fi
        echo "$pid" > "$PIDFILE"
        echo "已后台启动，PID=$pid"
        echo "日志：$LOG"
        echo "查看进度：$0 tail"
        ;;
    stop)
        pid=$(find_pid)
        if [ -z "$pid" ]; then
            echo "没有在运行"
            exit 0
        fi
        echo "发送停止信号给 PID=$pid，等待优雅收尾…"
        kill -TERM "$pid"
        for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
            if [ -z "$(find_pid)" ]; then
                rm -f "$PIDFILE"
                latest=$(ls -1dt "$ROOT"/logs/single-*min-*/ 2>/dev/null | head -1)
                echo "进程已退出。最近一轮：$latest"
                if [ -n "$latest" ]; then
                    python3 -c "import json,sys,pathlib
p=pathlib.Path(sys.argv[1])/'summary.json'
if p.is_file():
    d=json.loads(p.read_text())
    print('  结果=%s  失能确认=%s  动作数=%s' % (d.get('outcome'), d.get('disabled_confirmed'), d.get('completed_actions')))
else:
    print('  没有 summary.json，请人工确认电机已停止、输出已关闭')" "$latest"
                fi
                exit 0
            fi
            sleep 3
        done
        echo "60 秒仍未退出，请检查：ps -eo pid,args | grep soak_test"
        exit 1
        ;;
    status)
        pid=$(find_pid)
        if [ -n "$pid" ]; then
            echo "运行中，PID=$pid，已运行 $(ps -o etime= -p "$pid" | tr -d ' ')"
        else
            echo "没有在运行"
        fi
        echo "--- 最近汇总 ---"
        latest=$(ls -1dt "$ROOT"/logs/soak-*/ 2>/dev/null | head -1)
        [ -n "$latest" ] && sed -n '1,40p' "$latest/index.md" 2>/dev/null
        ;;
    tail)
        echo "日志：$LOG（Ctrl+C 只退出查看，不影响运行）"
        tail -f "$LOG"
        ;;
    *)
        echo "用法：$0 {start|stop|status|tail} [soak_test.py 的参数]"
        echo "例：  $0 start --node 2 --rounds 14 --minutes 60"
        exit 2
        ;;
esac
