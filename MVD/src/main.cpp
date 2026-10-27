// 入口：解析参数、建立运行时、按结果收尾。
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include "mvd/runtime.h"

namespace {

void usage() {
  std::printf(
      "MVD（慕纬度）TS 系列电机 CAN 位置模式通信长测\n"
      "\n"
      "用法：mvd_cli [选项]\n"
      "\n"
      "  --mode time|turns   动作模式：time 按秒数（默认），turns 按圈数\n"
      "  --minutes N         测试时长（分钟），默认 30\n"
      "  --motors auto|1|2   接入几台电机，默认 auto\n"
      "  --ids 1,2           候选电机 ID，逗号分隔\n"
      "  --iface can0        CAN 接口名\n"
      "  --speed X           速度（输出轴 °/s）；turns 模式未指定时默认 60\n"
      "  --fwd-ms/hold-ms/rev-ms   time 模式各阶段时长，默认 3000/1000/3000\n"
      "  --turns-fwd/--turns-rev  turns 模式正反转圈数，默认 2 / 2\n"
      "  --poll-ms 70        轮询周期\n"
      "  --read-ms 50        读应答窗口\n"
      "  --write-ms 100      写周期与写应答窗口\n"
      "  --hb-ms 50          心跳周期\n"
      "  --motor-lost-sec 10 连续无回复判掉线的秒数\n"
      "  --log-root PATH     日志根目录，默认程序目录下的 logs\n"
      "  --check-only        只探测与读状态，不动作\n"
      "  --read-only         只读长测：不使能、不松闸、不写位置，只发 0x9A 与 0x92\n"
      "  -h, --help          显示本帮助\n");
}

bool take_value(int argc, char** argv, int* i, const char* name, std::string* out) {
  if (*i + 1 >= argc) {
    std::fprintf(stderr, "参数 %s 缺少取值\n", name);
    return false;
  }
  ++(*i);
  *out = argv[*i];
  return true;
}

bool parse_ids(const std::string& text, std::vector<int>* out) {
  out->clear();
  std::string cur;
  for (char c : text) {
    if (c == ',' || c == ' ') {
      if (!cur.empty()) {
        out->push_back(std::atoi(cur.c_str()));
        cur.clear();
      }
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) out->push_back(std::atoi(cur.c_str()));
  return !out->empty();
}

// 程序所在目录，用于默认的日志根目录。
std::string exe_dir(const char* argv0) {
  char resolved[4096];
  if (::realpath(argv0, resolved) == nullptr) return std::string(".");
  std::string path(resolved);
  size_t pos = path.find_last_of('/');
  if (pos == std::string::npos) return std::string(".");
  if (pos == 0) return std::string("/");
  return path.substr(0, pos);
}

// 默认日志根目录：可执行文件在 build/ 下时取上级目录的 logs。
std::string default_log_root(const char* argv0) {
  std::string dir = exe_dir(argv0);
  if (dir.size() >= 6 && dir.compare(dir.size() - 6, 6, "/build") == 0) {
    return dir.substr(0, dir.size() - 6) + "/logs";
  }
  return dir + "/logs";
}

void on_signal(int) { mvd::Runtime::request_stop(); }

}  // namespace

int main(int argc, char** argv) {
  mvd::Config cfg;
  bool speed_given = false;
  std::string log_root;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    std::string v;
    if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else if (a == "--mode") {
      if (!take_value(argc, argv, &i, "--mode", &v)) return 2;
      if (v != "time" && v != "turns") {
        std::fprintf(stderr, "未知模式：%s\n", v.c_str());
        return 2;
      }
      cfg.mode = v;
    } else if (a == "--minutes") {
      if (!take_value(argc, argv, &i, "--minutes", &v)) return 2;
      cfg.minutes = std::atof(v.c_str());
      if (cfg.minutes <= 0.0) {
        std::fprintf(stderr, "--minutes 必须大于 0\n");
        return 2;
      }
    } else if (a == "--motors") {
      if (!take_value(argc, argv, &i, "--motors", &v)) return 2;
      if (v != "auto" && v != "1" && v != "2") {
        std::fprintf(stderr, "--motors 只能是 auto / 1 / 2\n");
        return 2;
      }
      cfg.motors_mode = v;
    } else if (a == "--ids") {
      if (!take_value(argc, argv, &i, "--ids", &v)) return 2;
      if (!parse_ids(v, &cfg.ids)) {
        std::fprintf(stderr, "--ids 解析失败\n");
        return 2;
      }
    } else if (a == "--iface") {
      if (!take_value(argc, argv, &i, "--iface", &v)) return 2;
      cfg.iface = v;
    } else if (a == "--speed") {
      if (!take_value(argc, argv, &i, "--speed", &v)) return 2;
      cfg.speed_out = std::atof(v.c_str());
      speed_given = true;
      if (cfg.speed_out <= 0.0) {
        std::fprintf(stderr, "--speed 必须大于 0\n");
        return 2;
      }
    } else if (a == "--fwd-ms") {
      if (!take_value(argc, argv, &i, "--fwd-ms", &v)) return 2;
      cfg.fwd_ms = std::atoi(v.c_str());
    } else if (a == "--hold-ms") {
      if (!take_value(argc, argv, &i, "--hold-ms", &v)) return 2;
      cfg.hold_ms = std::atoi(v.c_str());
    } else if (a == "--rev-ms") {
      if (!take_value(argc, argv, &i, "--rev-ms", &v)) return 2;
      cfg.rev_ms = std::atoi(v.c_str());
    } else if (a == "--turns-fwd") {
      if (!take_value(argc, argv, &i, "--turns-fwd", &v)) return 2;
      cfg.turns_fwd = std::atof(v.c_str());
    } else if (a == "--turns-rev") {
      if (!take_value(argc, argv, &i, "--turns-rev", &v)) return 2;
      cfg.turns_rev = std::atof(v.c_str());
    } else if (a == "--poll-ms") {
      if (!take_value(argc, argv, &i, "--poll-ms", &v)) return 2;
      cfg.poll_ms = std::atoi(v.c_str());
    } else if (a == "--read-ms") {
      if (!take_value(argc, argv, &i, "--read-ms", &v)) return 2;
      cfg.read_ms = std::atoi(v.c_str());
    } else if (a == "--write-ms") {
      if (!take_value(argc, argv, &i, "--write-ms", &v)) return 2;
      cfg.write_ms = std::atoi(v.c_str());
    } else if (a == "--hb-ms") {
      if (!take_value(argc, argv, &i, "--hb-ms", &v)) return 2;
      cfg.hb_ms = std::atoi(v.c_str());
    } else if (a == "--motor-lost-sec") {
      if (!take_value(argc, argv, &i, "--motor-lost-sec", &v)) return 2;
      cfg.motor_lost_sec = std::atof(v.c_str());
    } else if (a == "--log-root") {
      if (!take_value(argc, argv, &i, "--log-root", &v)) return 2;
      log_root = v;
    } else if (a == "--check-only") {
      cfg.check_only = true;
    } else if (a == "--read-only") {
      cfg.read_only = true;
    } else {
      std::fprintf(stderr, "未知参数：%s（用 --help 查看用法）\n", a.c_str());
      return 2;
    }
  }

  if (cfg.mode == "turns" && !speed_given) cfg.speed_out = 60.0;
  cfg.log_root = log_root.empty() ? default_log_root(argv[0]) : log_root;
  if (cfg.poll_ms <= 0 || cfg.hb_ms <= 0 || cfg.write_ms <= 0) {
    std::fprintf(stderr, "轮询、心跳、写周期必须大于 0\n");
    return 2;
  }
  if (cfg.fwd_ms <= 0 || cfg.rev_ms <= 0 || cfg.hold_ms < 0) {
    std::fprintf(stderr, "阶段时长不合法\n");
    return 2;
  }
  cfg.derive();

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  mvd::Runtime run(cfg);
  try {
    run.setup();
  } catch (const std::exception& e) {
    run.log(std::string("初始化失败：") + e.what());
    std::fprintf(stderr, "初始化失败：%s\n", e.what());
    run.finish("failed", e.what());
    return 1;
  }

  if (cfg.check_only) {
    run.log("仅检查模式：不动作。");
    std::printf("仅检查模式：未执行动作。\n");
    std::fflush(stdout);
    run.finish("not_tested", "check-only 未执行动作");
    return 0;
  }

  try {
    run.run();
  } catch (const mvd::Interrupted&) {
    run.log("收到中断，开始停机收尾。");
    std::printf("\n收到中断，正在停机收尾 ……\n");
    std::fflush(stdout);
    run.finish("interrupted", "用户中断");
    return 130;
  } catch (const std::exception& e) {
    run.log(std::string("运行异常：") + e.what());
    std::fprintf(stderr, "\n运行异常：%s\n", e.what());
    run.finish("failed", e.what());
    return 1;
  }
  run.finish("completed");
  return 0;
}
