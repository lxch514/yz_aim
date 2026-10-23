#include "yz_aim.h"
#include "yz_aim_cli.h"
#include "yz_aim_tui.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace {
bool controller_is_active()
{
    return std::system(
        "systemctl is-active --quiet km2-controller.service >/dev/null 2>&1") == 0;
}
}

int main(int argc, char** argv)
{
    yz_aim::tool::Options options{};
    std::string error;
    if (!yz_aim::tool::parse_options(argc, argv, &options, &error)) {
        std::cerr << error << "\n" << yz_aim::tool::usage(argv[0]);
        return 2;
    }
    if (options.help) {
        std::cout << yz_aim::tool::usage(argv[0]);
        return 0;
    }
    if (options.version) {
        std::cout << "yz_aim_test 1.5.0\n";
        return 0;
    }
    if (!options.mock && controller_is_active()) {
        std::cerr << "km2-controller.service 正在运行；请按维护流程停止服务后再打开 LYS 设备。\n";
        return 3;
    }

    std::unique_ptr<yz_aim::Transport> transport;
    if (options.mock) {
        transport = yz_aim::tool::make_mock_transport(
            static_cast<uint8_t>(options.node));
        std::cerr << "模拟模式：已启用合成 CANopen 通信，不连接硬件。\n";
    } else {
        transport = yz_aim::open_lys_can(options.library, options.bitrate,
            error, options.device, options.channel, options.nominal_timing);
        if (!transport) {
            std::cerr << "打开 LYS 设备失败：" << error << "\n";
            return 4;
        }
    }
    try {
        yz_aim::Motor motor(*transport, static_cast<uint8_t>(options.node));
        const int result = yz_aim::tool::run_test(motor, options);
        if (options.mock) yz_aim::tool::print_mock_audit(*transport,
            static_cast<uint8_t>(options.node));
        return result;
    } catch (const std::exception& exception) {
        std::cerr << "YZ-AIM 测试程序失败：" << exception.what() << "\n";
        return 5;
    }
}
