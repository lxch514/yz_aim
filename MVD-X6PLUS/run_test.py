#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MVD 电机 + X6 PLUS 推进风机测试启动脚本：按需编译 C++ 程序，然后直接接管进程运行。

用法：
    ./run_test.py                         默认 30 分钟，自动识别电机台数，风机油门 500
    ./run_test.py 1                       1 分钟，自动识别
    ./run_test.py 30 1                    30 分钟，只用 1 台电机
    ./run_test.py 30 2                    30 分钟，两台电机同时运行
    ./run_test.py 30 2 --mode turns       按圈数模式：正转 2 圈 / 停 1s / 反转 2 圈
    ./run_test.py 30 2 --fan-throttle 1000    风机油门改成 1000
    ./run_test.py 30 2 --fan-throttle 0       风机只使能、不转
    ./run_test.py 30 2 --fan-disable          完全不控制风机
    ./run_test.py 30 2 --read-only            只读长测：电机不使能不动作，只看通信
    ./run_test.py 30 2 --no-can-setup         跳过自动配置 can0（SJW=8），沿用现场参数
    ./run_test.py --help                  其余参数原样传给 C++ 程序
"""

import fcntl
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(HERE, "build")
BIN = os.path.join(BUILD_DIR, "mvd_x6_cli")
SOURCE_DIRS = ("src", "include")
SOURCE_EXTS = (".cpp", ".h")


def need_build():
    """判断是否需要（重新）编译：可执行文件缺失或源码更新过。"""
    if not os.path.exists(BIN):
        return True
    bin_mtime = os.path.getmtime(BIN)
    candidates = [os.path.join(HERE, "CMakeLists.txt")]
    for name in SOURCE_DIRS:
        root_dir = os.path.join(HERE, name)
        for root, _, files in os.walk(root_dir):
            for f in files:
                if f.endswith(SOURCE_EXTS):
                    candidates.append(os.path.join(root, f))
    for path in candidates:
        if os.path.exists(path) and os.path.getmtime(path) > bin_mtime:
            return True
    return False


def build():
    """首次完整配置，之后只做增量编译。"""
    cache = os.path.join(BUILD_DIR, "CMakeCache.txt")
    if not os.path.exists(cache):
        print("首次编译，正在配置构建目录 ……", flush=True)
        subprocess.check_call(["cmake", "-S", HERE, "-B", BUILD_DIR])
    print("正在编译 ……", flush=True)
    jobs = str(max(1, (os.cpu_count() or 2)))
    subprocess.check_call(["cmake", "--build", BUILD_DIR, "--target", "mvd_x6_cli", "-j", jobs])


def take_args(argv):
    """拆出「分钟数」和「电机台数」，其余参数原样透传。"""
    minutes = "30"
    motors = None
    args = list(argv)
    if args and re.match(r"^\d+(\.\d+)?$", args[0]):
        minutes = args.pop(0)
    if args and args[0] in ("1", "2", "auto"):
        motors = args.pop(0)
    return minutes, motors, args


def check_bitrate(iface):
    """提示接口速率是否与电机一致的 1Mbps。"""
    try:
        out = subprocess.run(["ip", "-details", "link", "show", iface],
                             capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return
    m = re.search(r"bitrate (\d+)", out)
    if m and m.group(1) != "1000000":
        print("警告：%s 当前速率 %s bps，MVD 电机出厂为 1Mbps，通信很可能失败。"
              % (iface, m.group(1)), flush=True)


def setup_can(iface):
    """调用工程内的 setup_can.sh 配置位时序（SJW=8）并复位控制器。"""
    script = os.path.join(HERE, "setup_can.sh")
    if not os.path.exists(script):
        print("警告：缺少 setup_can.sh，跳过接口配置。", flush=True)
        return True
    print("配置 %s 位时序（SJW=8）并复位控制器 ……" % iface, flush=True)
    try:
        rc = subprocess.call(["sh", script, iface])
    except OSError as e:
        print("警告：无法执行 setup_can.sh：%s" % (e,), flush=True)
        return True
    if rc != 0:
        print("错误：%s 位时序配置失败，测试未启动。" % iface, flush=True)
        print("      排查接线后可加 --no-can-setup 跳过本步。", flush=True)
        return False
    return True


def main():
    minutes, motors, extra = take_args(sys.argv[1:])
    if not os.path.isdir(os.path.join(HERE, "src")):
        print("找不到源码目录 src/，请在完整的 MVD 工程目录下运行。", flush=True)
        return 1
    if need_build():
        try:
            build()
        except (OSError, subprocess.CalledProcessError) as e:
            print("编译失败：%s" % (e,), flush=True)
            return 1

    iface = os.environ.get("IFACE", "can0")
    lock_path = os.path.join(HERE, ".mvd-x6-%s.lock" % iface)
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("本套件已有测试在运行，已退出。", flush=True)
        return 1
    os.set_inheritable(lock_fd, True)

    # 拿到锁之后再动接口，避免把正在运行的测试的位时序改掉
    if "--no-can-setup" in extra:
        extra.remove("--no-can-setup")
        check_bitrate(iface)
    elif not setup_can(iface):
        return 1

    cmd = [BIN, "--minutes", minutes, "--iface", iface]
    if motors:
        cmd += ["--motors", motors]
    cmd += extra
    print("执行：%s" % " ".join(cmd), flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    os.execv(BIN, cmd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
