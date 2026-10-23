#!/bin/bash
# yz_aim 独立构建脚本：只依赖本文件夹内的 include/ src/ tools/ test/ sdk/，
# 不需要完整控制器仓库，也不需要 FTXUI。
#
#   ./build.sh                    板上原生编译（Release）
#   ./build.sh --debug            板上原生编译（Debug）
#   ./build.sh --cross            在 x86_64 宿主上交叉编译 aarch64（Release）
#   ./build.sh --cross --debug    交叉编译 Debug
#   ./build.sh --clean            先删除对应构建目录再编
#   ./build.sh --only yz_aim_cli  只编某一个目标（可重复指定）
#
# 构建目录（互不覆盖，也不会碰到设备上已有的 build/）：
#   原生：build-native/release/   或 build-native/debug/
#   交叉：build-cross/release/    或 build-cross/debug/
# 产物：<构建目录>/bin/yz_aim_cli
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMPDIR="$SCRIPT_DIR/.build-tmp"
mkdir -p "$TMPDIR"
export TMPDIR
BUILD_TYPE="Release"
CROSS=0
CLEAN=0
ONLY_TARGETS=()

for arg in "$@"; do
    case "$arg" in
        --cross)      CROSS=1 ;;
        --debug|Debug)     BUILD_TYPE="Debug" ;;
        --release|Release) BUILD_TYPE="Release" ;;
        --clean)      CLEAN=1 ;;
        --only)       ONLY_NEXT=1 ;;
        --help|-h)
            echo "用法: $0 [--cross] [--debug|--release] [--clean] [--only <目标>]"
            echo "  不带参数     = 板上原生 Release 编译"
            echo "  --cross      = x86_64 宿主交叉编译 aarch64"
            echo "  --only TOTAL = 只编指定目标，可写多次"
            exit 0
            ;;
        *)
            if [[ "${ONLY_NEXT}" == "1" ]]; then
                ONLY_TARGETS+=("$arg")
                ONLY_NEXT=0
            else
                echo "[!] 未知参数: $arg（用 --help 查看用法）"
                exit 1
            fi
            ;;
    esac
done

BUILD_TYPE_LOWER="$(printf '%s' "${BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')"
JOBS="$(nproc 2>/dev/null || echo 4)"

if [[ "${ONLY_NEXT}" == "1" ]]; then
    echo "[!] --only 后面缺少目标名"
    exit 1
fi

if [[ "${CROSS}" == "1" ]]; then
    BUILD_DIR="${SCRIPT_DIR}/build-cross/${BUILD_TYPE_LOWER}"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/aarch64-linux-gnu.toolchain.cmake"
    echo "[*] 模式: 交叉编译 aarch64 (${BUILD_TYPE})"
    if ! command -v aarch64-linux-gnu-g++ >/dev/null 2>&1; then
        echo "[!] 找不到 aarch64-linux-gnu-g++；请先安装 g++-aarch64-linux-gnu"
        exit 1
    fi
else
    BUILD_DIR="${SCRIPT_DIR}/build-native/${BUILD_TYPE_LOWER}"
    TOOLCHAIN_FILE=""
    echo "[*] 模式: 本机原生编译 (${BUILD_TYPE})"
    if ! command -v g++ >/dev/null 2>&1; then
        echo "[!] 找不到 g++；请先安装 build-essential / g++"
        exit 1
    fi
fi

if [[ "${CLEAN}" == "1" ]]; then
    echo "[*] 清理构建目录: ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

# 目录整体搬移过（例如从桌面拷到设备 /root）后，旧 CMakeCache 里记录的源码
# 路径会失效，必须丢掉重建，否则 cmake 会去找不存在的旧路径。
if [[ -f "${BUILD_DIR}/CMakeCache.txt" ]] && \
   ! grep -q "^CMAKE_HOME_DIRECTORY:INTERNAL=${SCRIPT_DIR}$" "${BUILD_DIR}/CMakeCache.txt"; then
    echo "[*] 检测到旧路径 CMakeCache，重建构建目录: ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"

# 每次都重新 configure：否则新增目标（例如 yz_aim_cli）不会被旧缓存收录。
echo "[*] CMake 配置中..."
if [[ "${CROSS}" == "1" ]]; then
    cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_TLS_VERIFY=OFF
else
    cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
fi

if [[ "${CROSS}" == "1" ]]; then
    COMPILER="$(grep "^CMAKE_CXX_COMPILER_AR:FILEPATH" "${BUILD_DIR}/CMakeCache.txt" | cut -d= -f2)"
    echo "[*] 使用编译器: ${COMPILER}"
    if [[ "${COMPILER}" != *"aarch64"* ]]; then
        echo "[!] 未检测到 aarch64 工具链，请检查 ${TOOLCHAIN_FILE}"
        exit 1
    fi
fi

if [[ "${#ONLY_TARGETS[@]}" -gt 0 ]]; then
    echo "[*] 开始编译指定目标 (${JOBS} 核心): ${ONLY_TARGETS[*]}"
    for target in "${ONLY_TARGETS[@]}"; do
        cmake --build "${BUILD_DIR}" -j"${JOBS}" --target "${target}"
    done
else
    echo "[*] 开始编译全部可编目标 (${JOBS} 核心)..."
    cmake --build "${BUILD_DIR}" -j"${JOBS}"
fi

echo ""
echo "[√] 编译完成 [${BUILD_TYPE}]"
if [[ -d "${BUILD_DIR}/bin" ]]; then
    echo "    产物目录: ${BUILD_DIR}/bin/"
    ls -lh "${BUILD_DIR}/bin/" || true
fi
