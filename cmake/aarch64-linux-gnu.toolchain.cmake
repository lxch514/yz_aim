# ================================================================
# CMake Toolchain File — aarch64-linux-gnu 交叉编译
#
# 用法:
#   cmake <yz_aim 目录> -B build-cross/release \
#       -DCMAKE_TOOLCHAIN_FILE=<yz_aim 目录>/cmake/aarch64-linux-gnu.toolchain.cmake
#
# 宿主机: x86_64 Ubuntu 22.04+
# 目标机: aarch64 TQ3588 (Ubuntu 20.04/22.04, Kernel 5.10.160)
# ================================================================

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 交叉编译器
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# sysroot（使用 multiarch 路径，无需独立 sysroot）
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)

# pkg-config 搜索目标架构的 .pc 文件
set(ENV{PKG_CONFIG_PATH}        "/usr/lib/aarch64-linux-gnu/pkgconfig")
set(ENV{PKG_CONFIG_LIBDIR}      "/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "")

# 查找策略：头文件和库只在目标 sysroot 中查找，工具在宿主机查找
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
