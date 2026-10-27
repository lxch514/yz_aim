// MVD 协议编解码实现。
#include "mvd/protocol.h"

#include <cmath>
#include <cstdio>
#include <map>

namespace mvd {
namespace {

uint16_t rd_u16(const Frame& d, int off) {
  return static_cast<uint16_t>(d[off] | (d[off + 1] << 8));
}

int16_t rd_i16(const Frame& d, int off) {
  return static_cast<int16_t>(rd_u16(d, off));
}

int sign_temp(uint8_t v) {
  return v > 127 ? static_cast<int>(v) - 256 : static_cast<int>(v);
}

}  // namespace

std::string cmd_name(uint8_t cmd) {
  static const std::map<uint8_t, std::string> kNames = {
      {0x80, "电机关闭"}, {0x81, "电机停止"}, {0x88, "电机运行"}, {0x8C, "抱闸控制"},
      {0x90, "读编码器"}, {0x92, "读多圈角度"}, {0x94, "读单圈角度"}, {0x9A, "读状态1"},
      {0x9B, "清错误"},   {0x9C, "读状态2"},   {0xA1, "转矩闭环"}, {0xA4, "多圈位置2"},
  };
  auto it = kNames.find(cmd);
  if (it == kNames.end()) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "cmd 0x%02X", cmd);
    return std::string(buf);
  }
  return it->second;
}

std::string hexs(const Frame& f, int len) {
  std::string out;
  char buf[4];
  for (int i = 0; i < len && i < 8; ++i) {
    if (i) out.push_back(' ');
    std::snprintf(buf, sizeof(buf), "%02X", f[i]);
    out += buf;
  }
  return out;
}

Frame cmd_simple(uint8_t cmd) {
  Frame f{};
  f[0] = cmd;
  return f;
}

Frame cmd_brake(uint8_t action) {
  Frame f{};
  f[0] = 0x8C;
  f[1] = action;
  return f;
}

Frame cmd_pos2(double speed_dps, int64_t angle_raw) {
  int sp = static_cast<int>(std::lround(speed_dps));
  if (sp < 0) sp = 0;
  if (sp > 65535) sp = 65535;
  uint32_t ang = static_cast<uint32_t>(angle_raw);
  Frame f{};
  f[0] = 0xA4;
  f[1] = 0x00;
  f[2] = static_cast<uint8_t>(sp & 0xFF);
  f[3] = static_cast<uint8_t>((sp >> 8) & 0xFF);
  f[4] = static_cast<uint8_t>(ang & 0xFF);
  f[5] = static_cast<uint8_t>((ang >> 8) & 0xFF);
  f[6] = static_cast<uint8_t>((ang >> 16) & 0xFF);
  f[7] = static_cast<uint8_t>((ang >> 24) & 0xFF);
  return f;
}

State1 parse_9a(const Frame& d) {
  State1 s;
  s.temp = sign_temp(d[1]);
  s.volt = rd_u16(d, 2) * 0.01;
  s.cur = rd_u16(d, 4) * 0.01;
  s.state = d[6];
  s.err = d[7];
  return s;
}

Feedback parse_fb(const Frame& d) {
  Feedback f;
  f.temp = sign_temp(d[1]);
  f.iq = rd_i16(d, 2);
  f.iq_a = f.iq * kIqAmpPerLsb;
  f.speed = rd_i16(d, 4);
  f.enc = rd_u16(d, 6);
  return f;
}

int64_t parse_92(const Frame& d) {
  uint64_t v = 0;
  for (int i = 0; i < 7; ++i) {
    v |= static_cast<uint64_t>(d[1 + i]) << (8 * i);
  }
  int64_t sv = static_cast<int64_t>(v & 0x00FFFFFFFFFFFFFFULL);
  if (sv & (1LL << 55)) sv -= (1LL << 56);
  return sv;
}

double deg_out(int64_t raw) { return static_cast<double>(raw) / kRawPerOutDeg; }
double deg_out(double raw) { return raw / kRawPerOutDeg; }

std::string feedback_note(uint8_t cmd, const Frame& data) {
  char buf[160];
  switch (cmd) {
    case 0x9A: {
      State1 p = parse_9a(data);
      std::snprintf(buf, sizeof(buf), "温度%d℃ 母线%.2fV %.2fA 状态0x%02X 错误0x%02X", p.temp,
                    p.volt, p.cur, p.state, p.err);
      return std::string(buf);
    }
    case 0x92: {
      int64_t a = parse_92(data);
      std::snprintf(buf, sizeof(buf), "角度 %.2f°(电机轴) = %.2f°(输出轴)", a / 100.0,
                    deg_out(a));
      return std::string(buf);
    }
    case 0xA4: {
      Feedback p = parse_fb(data);
      std::snprintf(buf, sizeof(buf), "转矩%dLSB(%.2fA) 转速%d dps 编码器%d 温度%d℃", p.iq, p.iq_a,
                    p.speed, p.enc, p.temp);
      return std::string(buf);
    }
    case 0x8C:
      return std::string("抱闸状态 ") + (data[1] == 0x01 ? "释放" : "抱死");
    default:
      return std::string();
  }
}

}  // namespace mvd
