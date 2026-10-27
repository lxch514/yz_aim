// MVD（慕纬度）TS 系列电机 CAN 协议：常量、帧编码与回复解析。
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace mvd {

// CAN 标识符 = 0x140 + Driver ID（1~32）
constexpr uint32_t kIdBase = 0x140;

// 位置单位 0.01°/LSB（电机轴），减速比 36，输出轴 1° = 3600 原始单位。
constexpr double kGearRatio = 36.0;
constexpr double kRawPerMotorDeg = 100.0;
constexpr double kRawPerOutDeg = kRawPerMotorDeg * kGearRatio;
constexpr double kRawPerTurn = 360.0 * kRawPerOutDeg;

// TS 系列转矩电流换算
constexpr double kIqAmpPerLsb = 66.0 / 4096.0;

using Frame = std::array<uint8_t, 8>;

// 命令字节的中文名。
std::string cmd_name(uint8_t cmd);

// 帧内前 len 字节转为空格分隔的大写十六进制。
std::string hexs(const Frame& f, int len);

// 只有一个命令字节的帧（0x88 / 0x81 / 0x9A / 0x92 / 0x9B ...）。
Frame cmd_simple(uint8_t cmd);
// 抱闸控制帧：0x00 抱死、0x01 释放、0x10 读状态。
Frame cmd_brake(uint8_t action);
// 多圈位置模式2：速度限制 1dps/LSB，位置 0.01°/LSB。
Frame cmd_pos2(double speed_dps, int64_t angle_raw);

// 状态1（0x9A）回复。
struct State1 {
  int temp = 0;
  double volt = 0.0;
  double cur = 0.0;
  uint8_t state = 0;
  uint8_t err = 0;
};
State1 parse_9a(const Frame& d);

// 位置/速度/转矩闭环回复。
struct Feedback {
  int temp = 0;
  int iq = 0;
  double iq_a = 0.0;
  int speed = 0;
  int enc = 0;
};
Feedback parse_fb(const Frame& d);

// 多圈角度（0x92）回复：7 字节有符号，0.01°/LSB。
int64_t parse_92(const Frame& d);

// 原始单位转输出轴角度。
double deg_out(int64_t raw);
double deg_out(double raw);

// 回复帧的中文解析备注。
std::string feedback_note(uint8_t cmd, const Frame& data);

}  // namespace mvd
