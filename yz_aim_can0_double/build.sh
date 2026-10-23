#!/bin/sh
# 编译 yz_aim_can0：默认在板上原生编译，--cross 用 aarch64-linux-gnu 交叉编译。
set -e
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$SCRIPT_DIR"
TMPDIR="$SCRIPT_DIR/.build-tmp"
mkdir -p "$TMPDIR"
export TMPDIR

CXX=${CXX:-g++}
OUT=build
CXXFLAGS="-std=c++17 -O2 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Iinclude -Isrc"

case "$1" in
    "") ;;
    --cross)
        CXX=aarch64-linux-gnu-g++
        OUT=build-cross
        ;;
    --debug)
        CXXFLAGS="-std=c++17 -O0 -g -Wall -Wextra -Wno-unused-parameter -Iinclude -Isrc"
        ;;
    --test)
        # 合成台架测试：只与内存里的假驱动器对话，不打开 can0。
        echo "[*] 编译器：$CXX"
        $CXX -std=c++17 -O1 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Iinclude -Isrc \
            -o test/yz_aim_sustain_test test/yz_aim_sustain_test.cpp src/yz_aim.cpp -pthread
        exec ./test/yz_aim_sustain_test
        ;;
    --sync-test)
        # 双轴同步原语的合成台架测试：同样只与内存里的假驱动器对话，不打开 can0。
        echo "[*] 编译器：$CXX"
        $CXX -std=c++17 -O1 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Iinclude -Isrc \
            -o test/yz_aim_sync_test test/yz_aim_sync_test.cpp src/yz_aim.cpp -pthread
        exec ./test/yz_aim_sync_test
        ;;
    --runtime-test)
        echo "[*] 编译器：$CXX（仅内存驱动器，不打开CAN接口）"
        $CXX -std=c++17 -O1 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Iinclude -Isrc \
            -o test/yz_aim_runtime_test test/yz_aim_runtime_test.cpp src/yz_aim.cpp \
            src/motion_log.cpp src/frame_trace.cpp src/socketcan_transport.cpp src/can_link_query.cpp -pthread
        exec ./test/yz_aim_runtime_test
        ;;
    *)
        echo "用法：$0 [--cross|--debug|--test|--sync-test|--runtime-test]"
        exit 2
        ;;
esac

mkdir -p "$OUT"
echo "[*] 编译器：$CXX"
# shellcheck disable=SC2086
$CXX $CXXFLAGS -o "$OUT/yz_aim_can0" \
    src/yz_aim_can0.cpp \
    src/yz_aim.cpp \
    src/motion_log.cpp \
    src/frame_trace.cpp \
    src/socketcan_transport.cpp \
    src/can_link_query.cpp \
    -pthread
echo "[√] 生成 $OUT/yz_aim_can0"
"$OUT/yz_aim_can0" --version
