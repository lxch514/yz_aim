// 统计汇总、中文报告与文件落盘。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

#include "mvd/runtime.h"

namespace mvd {
namespace {

const mj::Value& jget(const mj::Value& v, const std::string& key) {
  static const mj::Value kNull;
  const mj::Value* p = v.find(key);
  return (p == nullptr) ? kNull : *p;
}

mj::Value jnum_or_null(bool has, double v) {
  return has ? mj::Value::number(v) : mj::Value();
}

mj::Value size_num(size_t v) { return mj::Value::number(static_cast<double>(v)); }

// 原始单位的值按输出轴角度输出。
std::string fmt_raw(const mj::Value& v, int nd) {
  if (v.type() != mj::Value::NUM) return "-";
  return fmt_d(deg_out(v.as_num()), nd);
}

std::string fmt_num(const mj::Value& v, int nd) {
  if (v.type() != mj::Value::NUM) return "-";
  return fmt_d(v.as_num(), nd);
}

std::string fmt_int_val(const mj::Value& v) {
  if (v.type() != mj::Value::NUM) return "-";
  return std::to_string(v.as_int());
}

double round_to(double v, int nd) {
  double scale = std::pow(10.0, nd);
  return std::round(v * scale) / scale;
}

double mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  double s = 0.0;
  for (double x : v) s += x;
  return s / static_cast<double>(v.size());
}

mj::Value st_rec(const ReqStat& d) {
  mj::Value o = mj::Value::object();
  double rate = d.req ? (static_cast<double>(d.timeout) / static_cast<double>(d.req) * 100.0) : 0.0;
  o.set("req", mj::Value::number(static_cast<double>(d.req)));
  o.set("ok", mj::Value::number(static_cast<double>(d.ok)));
  o.set("timeout", mj::Value::number(static_cast<double>(d.timeout)));
  o.set("timeout_rate_pct", mj::Value::number(round_to(rate, 4)));
  if (d.lat.empty()) {
    o.set("lat_min", mj::Value());
    o.set("lat_avg", mj::Value());
    o.set("lat_p50", mj::Value());
    o.set("lat_p95", mj::Value());
    o.set("lat_p99", mj::Value());
    o.set("lat_max", mj::Value());
  } else {
    o.set("lat_min", mj::Value::number(percentile(d.lat, 0.0)));
    o.set("lat_avg", mj::Value::number(mean(d.lat)));
    o.set("lat_p50", mj::Value::number(percentile(d.lat, 0.5)));
    o.set("lat_p95", mj::Value::number(percentile(d.lat, 0.95)));
    o.set("lat_p99", mj::Value::number(percentile(d.lat, 0.99)));
    o.set("lat_max", mj::Value::number(percentile(d.lat, 1.0)));
  }
  return o;
}

mj::Value stop_json(const StopInfo* si) {
  if (si == nullptr) return mj::Value();
  mj::Value o = mj::Value::object();
  o.set("hold_angle_raw", jnum_or_null(si->has_hold, static_cast<double>(si->hold_raw)));
  o.set("brake", si->brake.empty() ? mj::Value() : mj::Value::string(si->brake));
  o.set("brake_ok", mj::Value::boolean(si->brake_ok));
  o.set("speed_dps", mj::Value::number(si->speed_dps));
  o.set("final_angle_raw", jnum_or_null(si->has_final, static_cast<double>(si->final_raw)));
  o.set("return_error_raw",
        jnum_or_null(si->has_return_err, static_cast<double>(si->return_err_raw)));
  return o;
}

}  // namespace

mj::Value Runtime::build_summary(const std::map<int, StopInfo>& stop) {
  mj::Value per = mj::Value::object();
  for (int mid : motors_) {
    MotorStat& st = stats_[mid];
    mj::Value phases = mj::Value::object();
    for (int pi = 0; pi < 3; ++pi) {
      const PhaseStat& ps = ph_stats_[mid][pi];
      std::vector<double> starts, ends;
      for (const auto& c : cycles_) {
        auto it = c.per_motor.find(mid);
        if (it == c.per_motor.end()) continue;
        const CyclePhase& cp = it->second[pi];
        if (cp.has_pos) {
          starts.push_back(static_cast<double>(cp.pos_start));
          ends.push_back(static_cast<double>(cp.pos_end));
        }
      }
      std::vector<double> deltas;
      for (size_t i = 0; i < starts.size(); ++i) deltas.push_back(ends[i] - starts[i]);
      mj::Value d = mj::Value::object();
      d.set("cycles", size_num(starts.size()));
      d.set("samples_pos", size_num(ps.pos.size()));
      d.set("samples_speed", size_num(ps.speed.size()));
      d.set("pos_first",
            ps.pos.empty() ? mj::Value() : mj::Value::number(static_cast<double>(ps.pos.front())));
      d.set("pos_last",
            ps.pos.empty() ? mj::Value() : mj::Value::number(static_cast<double>(ps.pos.back())));
      d.set("start_avg", starts.empty() ? mj::Value() : mj::Value::number(mean(starts)));
      d.set("end_avg", ends.empty() ? mj::Value() : mj::Value::number(mean(ends)));
      d.set("delta_avg", deltas.empty() ? mj::Value() : mj::Value::number(mean(deltas)));
      d.set("delta_min", deltas.empty() ? mj::Value()
                                        : mj::Value::number(*std::min_element(deltas.begin(),
                                                                             deltas.end())));
      d.set("delta_max", deltas.empty() ? mj::Value()
                                        : mj::Value::number(*std::max_element(deltas.begin(),
                                                                             deltas.end())));
      if (ps.speed.empty()) {
        d.set("speed_min", mj::Value());
        d.set("speed_max", mj::Value());
        d.set("speed_avg", mj::Value());
      } else {
        d.set("speed_min", mj::Value::number(*std::min_element(ps.speed.begin(), ps.speed.end())));
        d.set("speed_max", mj::Value::number(*std::max_element(ps.speed.begin(), ps.speed.end())));
        double sum = 0.0;
        for (int s : ps.speed) sum += s;
        d.set("speed_avg", mj::Value::number(sum / static_cast<double>(ps.speed.size())));
      }
      phases.set(kPhaseName[pi], d);
    }

    MotorState& ms = mstate_[mid];
    mj::Value pm = mj::Value::object();
    pm.set("tx", mj::Value::number(static_cast<double>(st.tx)));
    pm.set("rx", mj::Value::number(static_cast<double>(st.rx)));
    pm.set("tx_fail", mj::Value::number(static_cast<double>(st.tx_fail)));
    pm.set("rx_other", mj::Value::number(static_cast<double>(st.rx_other)));
    pm.set("rx_err_frame", mj::Value::number(static_cast<double>(st.rx_err_frame)));
    pm.set("heartbeat", st_rec(st.hb));
    pm.set("poll", st_rec(st.poll));
    pm.set("write", st_rec(st.write));
    pm.set("phases", phases);
    pm.set("temp", ms.has_temp ? mj::Value::number(ms.temp) : mj::Value());
    pm.set("volt", ms.has_volt ? mj::Value::number(ms.volt) : mj::Value());
    pm.set("err", ms.has_err ? mj::Value::number(ms.err) : mj::Value());
    auto oit = origin_.find(mid);
    pm.set("origin_raw", oit != origin_.end() ? mj::Value::number(static_cast<double>(oit->second))
                                              : mj::Value());
    pm.set("final_angle_raw",
           ms.has_angle ? mj::Value::number(static_cast<double>(ms.angle)) : mj::Value());
    auto sit = stop.find(mid);
    pm.set("stop", stop_json(sit == stop.end() ? nullptr : &sit->second));
    per.set(std::to_string(mid), pm);
  }

  mj::Value s = mj::Value::object();
  s.set("started_at", mj::Value::string(started_at_));
  s.set("ended_at", mj::Value::string(iso_now()));
  s.set("run_directory", mj::Value::string(run_dir_));
  s.set("outcome", mj::Value::string(outcome_));
  s.set("reason", mj::Value::string(reason_));
  mj::Value motors_v = mj::Value::array();
  for (int mid : motors_) motors_v.push(mj::Value::number(mid));
  s.set("motors", motors_v);
  s.set("settings", cfg_.to_json());
  s.set("cycles_completed", mj::Value::number(cycle_of_ < 0 ? 0 : cycle_of_ + 1));
  double wall = (t_start_ > 0.0) ? ((t_end_ > 0.0 ? t_end_ : now_s()) - t_start_) : 0.0;
  s.set("wall_seconds", t_start_ > 0.0 ? mj::Value::number(round_to(wall, 3)) : mj::Value());
  s.set("per_motor", per);

  mj::Value cyc = mj::Value::array();
  for (const auto& c : cycles_) {
    mj::Value o = mj::Value::object();
    o.set("cycle", mj::Value::number(c.cycle));
    for (const auto& kv : c.per_motor) {
      mj::Value m = mj::Value::object();
      for (int pi = 0; pi < 3; ++pi) {
        const CyclePhase& cp = kv.second[pi];
        mj::Value d = mj::Value::object();
        d.set("samples_pos", mj::Value::number(cp.samples_pos));
        d.set("pos_start", jnum_or_null(cp.has_pos, static_cast<double>(cp.pos_start)));
        d.set("pos_end", jnum_or_null(cp.has_pos, static_cast<double>(cp.pos_end)));
        d.set("speed_min", jnum_or_null(cp.has_speed, cp.speed_min));
        d.set("speed_max", jnum_or_null(cp.has_speed, cp.speed_max));
        m.set(kPhaseName[pi], d);
      }
      o.set(std::to_string(kv.first), m);
    }
    cyc.push(o);
  }
  s.set("cycles", cyc);

  // 推进风机
  FanStats fs = fan_.stats();
  mj::Value fan = mj::Value::object();
  fan.set("control", mj::Value::boolean(cfg_.fan.control));
  fan.set("throttle", mj::Value::number(cfg_.fan.throttle));
  fan.set("throttle_pct", mj::Value::number(round_to(cfg_.fan.throttle * 100.0 / kHwMaxThrottle, 2)));
  fan.set("period_ms_set", mj::Value::number(cfg_.fan.period_ms));
  fan.set("tx", mj::Value::number(static_cast<double>(fs.tx)));
  fan.set("tx_fail", mj::Value::number(static_cast<double>(fs.tx_fail)));
  fan.set("late", mj::Value::number(static_cast<double>(fs.late)));
  fan.set("telemetry", mj::Value::number(static_cast<double>(fs.telemetry)));
  fan.set("init_zero_sec", mj::Value::number(cfg_.fan.init_zero_sec));
  fan.set("stop_zero_frames", mj::Value::number(cfg_.fan.stop_zero_frames));
  if (fs.period_ms.empty()) {
    fan.set("period_min", mj::Value());
    fan.set("period_avg", mj::Value());
    fan.set("period_p50", mj::Value());
    fan.set("period_p95", mj::Value());
    fan.set("period_p99", mj::Value());
    fan.set("period_max", mj::Value());
  } else {
    fan.set("period_min", mj::Value::number(percentile(fs.period_ms, 0.0)));
    fan.set("period_avg", mj::Value::number(mean(fs.period_ms)));
    fan.set("period_p50", mj::Value::number(percentile(fs.period_ms, 0.5)));
    fan.set("period_p95", mj::Value::number(percentile(fs.period_ms, 0.95)));
    fan.set("period_p99", mj::Value::number(percentile(fs.period_ms, 0.99)));
    fan.set("period_max", mj::Value::number(percentile(fs.period_ms, 1.0)));
  }
  mj::Value fnodes = mj::Value::object();
  for (const auto& kv : fs.nodes) {
    const FanNode& fn = kv.second;
    mj::Value o = mj::Value::object();
    o.set("channel", mj::Value::number(fn.channel + 1));
    o.set("frames", mj::Value::number(static_cast<double>(fn.frames())));
    o.set("msg1", mj::Value::number(static_cast<double>(fn.msg1)));
    o.set("msg2", mj::Value::number(static_cast<double>(fn.msg2)));
    o.set("msg3", mj::Value::number(static_cast<double>(fn.msg3)));
    o.set("dup", mj::Value::number(static_cast<double>(fn.dup)));
    o.set("rpm", jnum_or_null(fn.has_rpm, fn.rpm));
    o.set("throttle_raw", jnum_or_null(fn.has_thr, fn.thr_raw));
    o.set("status", jnum_or_null(fn.has_status, fn.status));
    o.set("volt", jnum_or_null(fn.has_volt, fn.volt));
    o.set("current", jnum_or_null(fn.has_cur, fn.cur));
    o.set("temp", jnum_or_null(fn.has_temp, fn.temp));
    o.set("mos_temp", jnum_or_null(fn.has_mos, fn.mos_temp));
    o.set("cap_temp", jnum_or_null(fn.has_cap, fn.cap_temp));
    o.set("motor_temp", jnum_or_null(fn.has_motor_temp, fn.motor_temp));
    o.set("gap_max_ms", mj::Value::number(round_to(fn.max_gap_ms, 3)));
    o.set("last_age_ms",
          fn.last_rx > 0.0 ? mj::Value::number(round_to((now_s() - fn.last_rx) * 1000.0, 3))
                           : mj::Value());
    o.set("last_unique_age_ms",
          fn.last_unique_rx > 0.0 ? mj::Value::number(round_to((now_s() - fn.last_unique_rx) * 1000.0, 3))
                                  : mj::Value());
    fnodes.set(std::to_string(kv.first), o);
  }
  fan.set("nodes", fnodes);
  s.set("fan", fan);
  return s;
}

std::string Runtime::build_report(const mj::Value& s, const std::map<int, StopInfo>& stop) {
  const std::vector<int>& ms = motors_;
  const size_t n = ms.size();
  std::vector<std::string> L;
  char buf[640];

  auto pm = [&](int mid) -> const mj::Value& { return jget(jget(s, "per_motor"), std::to_string(mid)); };
  auto row_head = [&](const std::string& first) {
    std::string line = "| " + first + " |";
    for (int m : ms) line += " 电机 " + std::to_string(m) + " |";
    L.push_back(line);
    std::string sep = "|---";
    for (size_t i = 0; i < n; ++i) sep += "|---";
    sep += "|";
    L.push_back(sep);
  };
  auto row = [&](const std::string& label, const std::function<std::string(int)>& fn) {
    std::string line = "| " + label + " |";
    for (int m : ms) line += " " + fn(m) + " |";
    L.push_back(line);
  };

  std::snprintf(buf, sizeof(buf), "# MVD 电机 + X6 PLUS 推进风机 CAN 通信测试（%s，%s 分钟，%d 台电机）",
                cfg_.read_only ? "只读" : "带动作", num_text(cfg_.minutes).c_str(),
                static_cast<int>(n));
  L.push_back(buf);
  L.push_back("");

  std::snprintf(buf, sizeof(buf), "- 测试开始：%s；结束：%s，实际用时 %.1f 秒。",
                started_at_.c_str(), iso_now().c_str(),
                jget(s, "wall_seconds").as_num(0.0));
  L.push_back(buf);
  std::string outcome_line = "- 总体状态：`" + outcome_ + "`";
  if (!reason_.empty()) outcome_line += "；" + reason_;
  L.push_back(outcome_line + "。");

  std::string ids_text, can_ids;
  for (size_t i = 0; i < n; ++i) {
    if (i) {
      ids_text += "、";
      can_ids += "、";
    }
    ids_text += std::to_string(ms[i]);
    std::snprintf(buf, sizeof(buf), "0x%03X", kIdBase + ms[i]);
    can_ids += buf;
  }
  L.push_back("- 接线：MVD 电机接板子原生 `" + cfg_.iface + "`，经典 CAN，1Mbps；ID " + ids_text +
              "（CAN 标识符 " + can_ids + "）。");
  {
    std::string nodes_text;
    const FanCfg& fc = cfg_.fan;
    for (size_t i = 0; i < fc.nodes.size(); ++i) {
      if (i) nodes_text += "、";
      std::snprintf(buf, sizeof(buf), "0x%02X(CH%d)", fc.nodes[i],
                    i < fc.channels.size() ? fc.channels[i] + 1 : static_cast<int>(i + 1));
      nodes_text += buf;
    }
    if (cfg_.fan.control) {
      std::snprintf(buf, sizeof(buf),
                    "- 风机：两台 X6 PLUS 电调接同一路 `%s`，29 位扩展帧，节点 %s，源节点 0x%02X，"
                    "油门 %u（%.1f%%），%dHz 持续下发。",
                    cfg_.iface.c_str(), nodes_text.c_str(), kHwSrcNode,
                    static_cast<unsigned>(cfg_.fan.throttle),
                    cfg_.fan.throttle * 100.0 / kHwMaxThrottle,
                    1000 / (cfg_.fan.period_ms > 0 ? cfg_.fan.period_ms : 20));
    } else {
      std::snprintf(buf, sizeof(buf),
                    "- 风机：本次不控制（`--fan-disable`），节点 %s 只接收遥测，不下发油门与使能帧。",
                    nodes_text.c_str());
    }
    L.push_back(buf);
  }
  if (cfg_.read_only) {
    L.push_back("- 模式：**只读长测** —— 电机不清错、不使能、不松抱闸、不发 0xA4 写位置，"
                "只按 50ms 发 0x9A 心跳、70ms 发 0x92 读角度，用来观察纯读取链路的通信质量。"
                "风机是否动作由 `--fan-throttle` / `--fan-disable` 单独决定。");
  } else {
    std::snprintf(buf, sizeof(buf),
                  "- 模式：多圈位置模式2（0xA4），速度限制 %s dps（电机轴）= %s°/s（输出轴，减速比 36），"
                  "参数模式 `%s`。",
                  num_text(cfg_.speed_dps).c_str(), num_text(cfg_.speed_out).c_str(), cfg_.mode.c_str());
    L.push_back(buf);
  }

  if (cfg_.read_only) {
    std::snprintf(buf, sizeof(buf),
                  "- 循环：不写位置，按组计时只用于分阶段统计窗口；每组 %.1f 秒，共完成 %lld 组。",
                  cfg_.cycle_s, jget(s, "cycles_completed").as_int());
    L.push_back(buf);
    std::snprintf(buf, sizeof(buf),
                  "- 时序：心跳 0x9A %dms，读角度 0x92 %dms（窗口 %dms）；不发写位置 0xA4。",
                  cfg_.hb_ms, cfg_.poll_ms, cfg_.read_ms);
    L.push_back(buf);
    L.push_back("- 行程：电机不动作，仅记录 0x92 回复的角度是否漂移。");
  } else {
    if (cfg_.turns_mode) {
      std::snprintf(buf, sizeof(buf),
                    "- 循环：正转 %s 圈 → 停止保持 %dms → 反转 %s 圈（按 %s°/s 折算约 %dms + %dms）；"
                    "每组 %.1f 秒，共完成 %lld 组。",
                    num_text(cfg_.turns_fwd).c_str(), cfg_.hold_ms, num_text(cfg_.turns_rev).c_str(),
                    num_text(cfg_.speed_out).c_str(), cfg_.fwd_ms, cfg_.rev_ms, cfg_.cycle_s,
                    jget(s, "cycles_completed").as_int());
    } else {
      std::snprintf(buf, sizeof(buf), "- 循环：正转 %dms → 停止保持 %dms → 反转 %dms；每组 %.1f 秒，共完成 %lld 组。",
                    cfg_.fwd_ms, cfg_.hold_ms, cfg_.rev_ms, cfg_.cycle_s,
                    jget(s, "cycles_completed").as_int());
    }
    L.push_back(buf);

    std::snprintf(buf, sizeof(buf), "- 时序：轮询 %dms，读超时 %dms，写周期 %dms（超时 %dms），心跳 %dms 主动轮询。",
                  cfg_.poll_ms, cfg_.read_ms, cfg_.write_ms, cfg_.write_ms, cfg_.hb_ms);
    L.push_back(buf);

    if (cfg_.turns_mode) {
      std::snprintf(buf, sizeof(buf), "- 行程：正转 %s 圈、反转 %s 圈（输出轴），即 %.2f° = %.2f°（电机轴）。",
                    num_text(cfg_.turns_fwd).c_str(), num_text(cfg_.turns_rev).c_str(),
                    deg_out(cfg_.off_fwd_raw), cfg_.off_fwd_raw / 100.0);
    } else {
      std::snprintf(buf, sizeof(buf), "- 行程：单向 %.2f°（输出轴）= %.2f°（电机轴）。",
                    deg_out(cfg_.off_fwd_raw), cfg_.off_fwd_raw / 100.0);
    }
    L.push_back(buf);
  }
  L.push_back("");

  L.push_back("## 总览");
  L.push_back("");
  row_head("项目");
  row("完成组数", [&](int) { return std::to_string(jget(s, "cycles_completed").as_int()); });
  row("发送帧 / 接收帧", [&](int m) {
    return std::to_string(jget(pm(m), "tx").as_int()) + " / " +
           std::to_string(jget(pm(m), "rx").as_int());
  });
  row("发送失败", [&](int m) { return std::to_string(jget(pm(m), "tx_fail").as_int()); });
  row("心跳请求 / 应答 / 超时", [&](int m) {
    const mj::Value& hb = jget(pm(m), "heartbeat");
    return std::to_string(jget(hb, "req").as_int()) + " / " +
           std::to_string(jget(hb, "ok").as_int()) + " / " +
           std::to_string(jget(hb, "timeout").as_int());
  });
  row("心跳超时率", [&](int m) {
    return fmt_d(jget(jget(pm(m), "heartbeat"), "timeout_rate_pct").as_num(), 4) + "%";
  });
  row("读角度请求 / 超时", [&](int m) {
    const mj::Value& p = jget(pm(m), "poll");
    return std::to_string(jget(p, "req").as_int()) + " / " +
           std::to_string(jget(p, "timeout").as_int());
  });
  row("写位置请求 / 超时", [&](int m) {
    const mj::Value& p = jget(pm(m), "write");
    return std::to_string(jget(p, "req").as_int()) + " / " +
           std::to_string(jget(p, "timeout").as_int());
  });
  row("起始角度（输出轴°）",
      [&](int m) { return fmt_raw(jget(pm(m), "origin_raw"), 2); });
  row("结束角度（输出轴°）",
      [&](int m) { return fmt_raw(jget(pm(m), "final_angle_raw"), 2); });
  row("停机回原点偏差（°）",
      [&](int m) { return fmt_raw(jget(jget(pm(m), "stop"), "return_error_raw"), 2); });
  row("末次温度（℃）", [&](int m) { return fmt_int_val(jget(pm(m), "temp")); });
  row("末次母线电压（V）", [&](int m) { return fmt_num(jget(pm(m), "volt"), 2); });
  row("末次错误标志", [&](int m) {
    std::snprintf(buf, sizeof(buf), "0x%02X", static_cast<int>(jget(pm(m), "err").as_int()));
    return std::string(buf);
  });
  row("停止后抱闸", [&](int m) {
    const mj::Value& b = jget(jget(pm(m), "stop"), "brake");
    return b.type() == mj::Value::STR ? b.as_str() : std::string("未知");
  });
  L.push_back("");

  L.push_back("## 各阶段反馈");
  for (int mid : ms) {
    const mj::Value& p = pm(mid);
    L.push_back("");
    L.push_back("电机 " + std::to_string(mid) + "：");
    L.push_back("");
    L.push_back("| 阶段 | 组数 | 平均起始(°) | 平均结束(°) | 每组角度变化 avg（min~max）(°) | 速度 min/max/avg(dps) |");
    L.push_back("|---:|---:|---:|---:|---|---|");
    for (int pi = 0; pi < 3; ++pi) {
      const mj::Value& d = jget(jget(p, "phases"), kPhaseName[pi]);
      std::string span = fmt_raw(jget(d, "delta_avg"), 2) + "（" + fmt_raw(jget(d, "delta_min"), 2) +
                         "~" + fmt_raw(jget(d, "delta_max"), 2) + "）";
      if (cfg_.turns_mode && jget(d, "delta_avg").type() == mj::Value::NUM) {
        span += "，平均 " + fmt_d(deg_out(jget(d, "delta_avg").as_num()) / 360.0, 2) + " 圈";
      }
      std::string line = "| " + std::string(kPhaseCn[pi]) + " | " +
                         fmt_int_val(jget(d, "cycles")) + " | " + fmt_raw(jget(d, "start_avg"), 2) +
                         " | " + fmt_raw(jget(d, "end_avg"), 2) + " | " + span + " | " +
                         fmt_num(jget(d, "speed_min"), 0) + " / " +
                         fmt_num(jget(d, "speed_max"), 0) + " / " +
                         fmt_num(jget(d, "speed_avg"), 0) + " |";
      L.push_back(line);
    }
    L.push_back("");
    L.push_back(
        "按组统计：起始/结束取每组首末位置采样的平均值，角度变化给出每组的变化量与最小~最大值；"
        "速度为全程采样。角度与速度均取自电机回复帧，只看到发送指令不能证明电机已转动。");
  }
  L.push_back("");

  // ---------------- 推进风机 ----------------
  {
    const mj::Value& fan = jget(s, "fan");
    L.push_back("## 推进风机（X6 PLUS）");
    L.push_back("");
    std::string nodes_text;
    const FanCfg& fc = cfg_.fan;
    for (size_t i = 0; i < fc.nodes.size(); ++i) {
      if (i) nodes_text += "、";
      std::snprintf(buf, sizeof(buf), "0x%02X→CH%d", fc.nodes[i],
                    i < fc.channels.size() ? fc.channels[i] + 1 : static_cast<int>(i + 1));
      nodes_text += buf;
    }
    L.push_back("| 项目 | 本次 |");
    L.push_back("|---|---|");
    L.push_back("| 电调节点 / 油门通道 | " + nodes_text + " |");
    std::snprintf(buf, sizeof(buf), "| 油门指令（原始 / 百分比） | %u / %.1f%% |",
                  static_cast<unsigned>(cfg_.fan.throttle),
                  cfg_.fan.throttle * 100.0 / kHwMaxThrottle);
    L.push_back(buf);
    if (cfg_.fan.control) {
      std::snprintf(buf, sizeof(buf),
                    "| 初始化流程 | 逐台选 CAN 油门 → 广播油门使能 0xAA → 连续零油门 %.1fs |",
                    cfg_.fan.init_zero_sec);
    } else {
      std::snprintf(buf, sizeof(buf), "| 初始化流程 | 本次未控制风机（--fan-disable），只收遥测 |");
    }
    L.push_back(buf);
    std::snprintf(buf, sizeof(buf), "| 设定频率 / RawCommand 帧数 | %.1fHz / %lld |",
                  1000.0 / (cfg_.fan.period_ms > 0 ? cfg_.fan.period_ms : 20),
                  jget(fan, "tx").as_int());
    L.push_back(buf);
    std::snprintf(buf, sizeof(buf), "| 发送失败（次） | %lld |", jget(fan, "tx_fail").as_int());
    L.push_back(buf);
    std::snprintf(buf, sizeof(buf),
                  "| 发送周期 min/avg/p50/p95/p99/max（ms） | %s / %s / %s / %s / %s / %s |",
                  fmt_num(jget(fan, "period_min"), 3).c_str(),
                  fmt_num(jget(fan, "period_avg"), 3).c_str(),
                  fmt_num(jget(fan, "period_p50"), 3).c_str(),
                  fmt_num(jget(fan, "period_p95"), 3).c_str(),
                  fmt_num(jget(fan, "period_p99"), 3).c_str(),
                  fmt_num(jget(fan, "period_max"), 3).c_str());
    L.push_back(buf);
    std::snprintf(buf, sizeof(buf), "| 迟于 1.5 倍周期补发的次数 | %lld |",
                  jget(fan, "late").as_int());
    L.push_back(buf);
    std::snprintf(buf, sizeof(buf), "| 收尾（零油门 + 失能） | 零油门 %lld 帧 + 失能 0x55（电调不回读确认） |",
                  jget(fan, "stop_zero_frames").as_int());
    L.push_back(buf);
    L.push_back("");
    L.push_back(
        "| 节点 | 通道 | 遥测帧 | 其中重复 | 状态 | 转速（rpm） | 电压（V） | 电流（A） | 温度（℃） | "
        "同类消息最大间隔（ms） | 末次遥测距今（ms） |");
    L.push_back("|---|---|---:|---:|---|---:|---:|---:|---:|---:|---:|");
    const mj::Value& fnodes = jget(fan, "nodes");
    for (const auto& kv : fnodes.entries()) {
      const mj::Value& o = kv.second;
      bool stale = false;
      const mj::Value& age = jget(o, "last_age_ms");
      if (age.type() == mj::Value::NUM && age.as_num() > cfg_.fan.stale_sec * 1000.0) stale = true;
      std::string state_text;
      if (jget(o, "frames").as_int() == 0) {
        state_text = "无遥测";
      } else if (stale) {
        state_text = "疑似掉线";
      } else {
        state_text = "在线";
      }
      std::string node_hex;
      {
        int node_num = std::atoi(kv.first.c_str());
        char nb[16];
        std::snprintf(nb, sizeof(nb), "0x%02X", node_num);
        node_hex = nb;
      }
      std::snprintf(buf, sizeof(buf), "| %s | CH%d | %lld | %lld | %s | %s | %s | %s | %s | %s | %s |",
                    node_hex.c_str(), static_cast<int>(jget(o, "channel").as_int()),
                    jget(o, "frames").as_int(), jget(o, "dup").as_int(), state_text.c_str(),
                    fmt_int_val(jget(o, "rpm")).c_str(), fmt_num(jget(o, "volt"), 1).c_str(),
                    fmt_num(jget(o, "current"), 1).c_str(), fmt_num(jget(o, "temp"), 0).c_str(),
                    fmt_num(jget(o, "gap_max_ms"), 1).c_str(), fmt_num(age, 1).c_str());
      L.push_back(buf);
    }
    L.push_back("");
    L.push_back(
        "油门是 14 位原始值（0~8191），四通道同值下发：0x01 取 CH1、0x02 取 CH2，所以两台转速一致。"
        "遥测由电调主动上报：MSG1 转速/油门/状态，MSG2 电压/电流/温度，MSG3 MOS/电容/电机温度。");
    L.push_back("RawCommand 每 20ms 一帧是保活，中断后电调会停转；帧记录按每类帧抽样，统计按全部帧计算。");
    L.push_back(
        "状态是按遥测最后更新时间判断的，重复帧单独计数；广播遥测不需要主机逐帧应答，重复不能归因于主机未回复。");
    L.push_back("");
  }

  L.push_back("## 通信与时延");
  L.push_back("");
  row_head("项目");
  for (int key = 0; key < 3; ++key) {
    const char* cn = (key == 0) ? "心跳 9A" : ((key == 1) ? "读角度 92" : "写位置 A4");
    const char* field = (key == 0) ? "heartbeat" : ((key == 1) ? "poll" : "write");
    const char* flds[6] = {"lat_min", "lat_avg", "lat_p50", "lat_p95", "lat_p99", "lat_max"};
    const char* fcns[6] = {"min", "avg", "p50", "p95", "p99", "max"};
    for (int i = 0; i < 6; ++i) {
      std::string label = std::string(cn) + " 应答延迟 " + fcns[i] + "（ms）";
      std::string fname = flds[i];
      row(label, [&, field, fname](int m) {
        return fmt_num(jget(jget(pm(m), field), fname), 3);
      });
    }
  }
  row("非本机/无关帧", [&](int m) { return std::to_string(jget(pm(m), "rx_other").as_int()); });
  row("CAN 错误帧", [&](int m) { return std::to_string(jget(pm(m), "rx_err_frame").as_int()); });
  L.push_back("");
  L.push_back("应答延迟＝发送完成到收到对应命令回复的时长；超时表示该窗口内没有收到回复。");
  L.push_back("");

  L.push_back("## CAN 控制器计数");
  L.push_back("");
  L.push_back("| 计数 | 开始 | 结束 |");
  L.push_back("|---|---:|---:|");
  const char* rx_keys[4] = {"packets", "errors", "dropped", "over_errors"};
  const char* rx_labels[4] = {"rx_packets", "rx_errors", "rx_dropped", "rx_over_errors"};
  for (int i = 0; i < 4; ++i) {
    L.push_back("| " + std::string(rx_labels[i]) + " | " + std::to_string(can_before_.counter("rx", rx_keys[i])) +
                " | " + std::to_string(can_after_.counter("rx", rx_keys[i])) + " |");
  }
  const char* tx_keys[3] = {"packets", "errors", "dropped"};
  const char* tx_labels[3] = {"tx_packets", "tx_errors", "tx_dropped"};
  for (int i = 0; i < 3; ++i) {
    L.push_back("| " + std::string(tx_labels[i]) + " | " + std::to_string(can_before_.counter("tx", tx_keys[i])) +
                " | " + std::to_string(can_after_.counter("tx", tx_keys[i])) + " |");
  }
  L.push_back("| 控制器状态 | " + (can_before_.state.empty() ? std::string("未知") : can_before_.state) +
              " | " + (can_after_.state.empty() ? std::string("未知") : can_after_.state) + " |");
  L.push_back("| 错误计数 tx/rx | " + can_before_.berr.dump(-1) + " | " + can_after_.berr.dump(-1) + " |");
  L.push_back("| xstats | " + can_before_.xstats.dump(-1) + " | " + can_after_.xstats.dump(-1) + " |");
  L.push_back("");

  if (n == 2) {
    L.push_back("## 双机同步");
    L.push_back("");
    for (int pi = 0; pi < 3; pi += 2) {
      std::vector<double> diffs;
      for (const auto& c : cycles_) {
        auto a = c.per_motor.find(ms[0]);
        auto b = c.per_motor.find(ms[1]);
        if (a == c.per_motor.end() || b == c.per_motor.end()) continue;
        const CyclePhase& ca = a->second[pi];
        const CyclePhase& cb = b->second[pi];
        if (!ca.has_pos || !cb.has_pos) continue;
        double da = static_cast<double>(ca.pos_end - ca.pos_start);
        double db = static_cast<double>(cb.pos_end - cb.pos_start);
        diffs.push_back(std::fabs(da - db));
      }
      if (!diffs.empty()) {
        std::snprintf(buf, sizeof(buf), "- %s每组角度变化量之差：最大 %.2f°，平均 %.2f°（共 %zu 组）。",
                      kPhaseCn[pi], deg_out(*std::max_element(diffs.begin(), diffs.end())),
                      deg_out(mean(diffs)), diffs.size());
        L.push_back(buf);
      } else {
        L.push_back(std::string("- ") + kPhaseCn[pi] + "：本次没有可比较的组。");
      }
    }
    L.push_back("- 两台电机参数完全相同、同阶段切换，指令按电机顺序依次下发（毫秒级先后）；");
    L.push_back("  两台安装的绝对零位不同，所以只比较每组的角度变化量，不比较绝对角度。");
    L.push_back("");
  }

  L.push_back("## 结论与判读");
  L.push_back("");
  long long total_to = 0, total_req = 0;
  for (int m : ms) {
    for (const char* key : {"heartbeat", "poll", "write"}) {
      total_to += jget(jget(pm(m), key), "timeout").as_int();
      total_req += jget(jget(pm(m), key), "req").as_int();
    }
  }
  std::snprintf(buf, sizeof(buf), "- 共发出 %lld 条读写请求，收到 %lld 条应答，超时 %lld 条，整体超时率 %.4f%%。",
                total_req, total_req - total_to, total_to,
                total_req ? (static_cast<double>(total_to) / static_cast<double>(total_req) * 100.0)
                          : 0.0);
  L.push_back(buf);
  double worst = 0.0;
  for (int m : ms) worst = std::max(worst, jget(jget(pm(m), "heartbeat"), "timeout_rate_pct").as_num());
  std::snprintf(buf, sizeof(buf), "- 心跳最大超时率 %.4f%%%s。", worst,
                worst < 0.5 ? "，低于 0.5% 判定阈值" : "，**超过 0.5% 判定阈值，需要排查**");
  L.push_back(buf);
  L.push_back("- CAN 控制器错误计数与专用 xstats 前后无增加时可认为物理层正常；"
              "普通 rx_errors/tx_errors 为 0 不能排除全部 CAN 专用错误，需结合 bus-health.jsonl。");
  {
    const mj::Value& fan = jget(s, "fan");
    const bool fan_ran = jget(fan, "tx").as_int() > 0;
    std::snprintf(buf, sizeof(buf),
                  "- 风机：累计下发 %lld 帧、发送失败 %lld 次、迟发 %lld 次；收尾零油门 %lld 帧 + 失能 0x55%s。",
                  jget(fan, "tx").as_int(), jget(fan, "tx_fail").as_int(),
                  jget(fan, "late").as_int(), jget(fan, "stop_zero_frames").as_int(),
                  fan_ran ? "" : "（本次未执行）");
    L.push_back(buf);
    long long no_telem = 0;
    long long rpm_min = -1;
    long long rpm_max = 0;
    for (const auto& kv : jget(fan, "nodes").entries()) {
      const mj::Value& o = kv.second;
      if (jget(o, "frames").as_int() == 0) ++no_telem;
      const mj::Value& r = jget(o, "rpm");
      if (r.type() == mj::Value::NUM) {
        if (rpm_min < 0 || r.as_int() < rpm_min) rpm_min = r.as_int();
        if (r.as_int() > rpm_max) rpm_max = r.as_int();
      }
    }
    if (no_telem > 0) {
      if (jget(fan, "tx").as_int() == 0) {
        std::snprintf(buf, sizeof(buf),
                      "- 本次没有下发风机指令（check-only 或 --fan-disable），%lld 台电调没有遥测属正常；"
                      "要验证风机须实际运行。",
                      no_telem);
      } else {
        std::snprintf(buf, sizeof(buf), "- 有 %lld 台电调没有回遥测，检查该路 CAN 接线、供电与节点 ID。",
                      no_telem);
      }
    } else if (rpm_min >= 0) {
      std::snprintf(buf, sizeof(buf),
                    "- 电调末次反馈转速 %lld~%lld rpm；下发成功只表示内核接受发送，"
                    "不能证明桨叶真的在转，以电调遥测为准。",
                    rpm_min, rpm_max);
    } else {
      std::snprintf(buf, sizeof(buf), "- 电调遥测中没有转速字段，无法判读是否转动。");
    }
    L.push_back(buf);
  }
  if (!stop.empty()) {
    std::string line = "- 停止确认：";
    for (size_t i = 0; i < ms.size(); ++i) {
      if (i) line += "、";
      auto it = stop.find(ms[i]);
      line += "电机 " + std::to_string(ms[i]) + " 抱闸 " +
              ((it == stop.end() || it->second.brake.empty()) ? std::string("未知") : it->second.brake);
    }
    L.push_back(line + "。");
  } else if (cfg_.read_only) {
    L.push_back("- 停止确认：只读模式电机从未使能，无需停止与抱闸动作。");
  } else {
    L.push_back("- 停止确认：无。");
  }
  L.push_back("");
  L.push_back("## 文件说明");
  L.push_back("");
  L.push_back("- `test.log`：中文进度、每组动作、每分钟通信小结与异常。");
  L.push_back("- `can-frames.log`：逐帧收发记录与解析备注；风机帧按每类每节点抽样记录，统计按全部帧计算。");
  L.push_back("- `events.jsonl`：超时、发帧失败、错误帧、阶段切换、停机确认等结构化事件。");
  L.push_back("- `bus-health.jsonl`：每秒 CAN 控制器状态、计数与各电机最近收帧时长。");
  L.push_back("- `summary.json`：全部统计与逐组动作记录。");
  L.push_back("- `config.json`：本次参数与 CAN 控制器前后计数。");

  std::string out;
  for (size_t i = 0; i < L.size(); ++i) {
    if (i) out += "\n";
    out += L[i];
  }
  return out;
}

std::string Runtime::finish(const std::string& outcome, const std::string& reason) {
  t_end_ = now_s();
  outcome_ = outcome;
  reason_ = reason;

  // 风机先停：桨叶先归零再动电机，收尾过程更安全
  if (cfg_.fan.control && fan_.needs_shutdown()) {
    fan_.stop();
    log("风机：收尾（零油门 " + std::to_string(cfg_.fan.stop_zero_frames) + " 帧 → 失能 0x55）。");
    bool disabled = false;
    const long long fail_before = fan_.stats().tx_fail;
    try {
      disabled = fan_.shutdown([this](int ms) { receive_for(ms / 1000.0); });
    } catch (const std::exception& e) {
      log(std::string("风机收尾异常：") + e.what());
    }
    mj::Value e = mj::Value::object();
    FanStats fs = fan_.stats();
    e.set("zero_frames", mj::Value::number(cfg_.fan.stop_zero_frames));
    e.set("tx_total", mj::Value::number(static_cast<double>(fs.tx)));
    e.set("tx_fail", mj::Value::number(static_cast<double>(fs.tx_fail)));
    const long long shutdown_fail = fs.tx_fail - fail_before;
    e.set("shutdown_tx_fail", mj::Value::number(static_cast<double>(shutdown_fail)));
    e.set("disable_frame_sent", mj::Value::boolean(disabled));
    event("fan_shutdown", e);
    if (disabled && shutdown_fail == 0) {
      log("风机：零油门 " + std::to_string(cfg_.fan.stop_zero_frames) +
          " 帧已发，失能帧 0x55 已发（电调不回读确认）。");
    } else {
      log("风机：收尾发送失败 " + std::to_string(shutdown_fail) +
          " 次，失能帧 0x55 " + (disabled ? "已入队" : "发送失败或未完成") +
          "（电调不回读确认）。");
    }
  } else {
    fan_.stop();
  }
  if (!cfg_.check_only && !motors_.empty() && (t_end_ - t_start_) >= 5.0) minute_summary(true);

  std::map<int, StopInfo> stop;
  if (!motors_.empty() && !cfg_.check_only && !cfg_.read_only && bus_open_) {
    try {
      stop = shutdown_motors();
    } catch (const std::exception& e) {
      log(std::string("停机收尾异常：") + e.what());
    } catch (...) {
      log("停机收尾异常：未知错误");
    }
  } else if (cfg_.read_only && !motors_.empty()) {
    log("只读模式：电机从未使能，不做回原点、停止与抱闸动作。");
  }

  can_after_ = can_status(cfg_.iface);
  {
    mj::Value e = mj::Value::object();
    e.set("outcome", mj::Value::string(outcome));
    e.set("reason", mj::Value::string(reason));
    e.set("snapshot", can_after_.to_json());
    event("run_end", e);
  }

  {
    mj::Value config = mj::Value::object();
    config.set("settings", cfg_.to_json());
    config.set("started_at", mj::Value::string(started_at_));
    config.set("ended_at", mj::Value::string(iso_now()));
    double wall = (t_start_ > 0.0) ? (t_end_ - t_start_) : 0.0;
    config.set("wall_seconds", t_start_ > 0.0 ? mj::Value::number(round_to(wall, 3)) : mj::Value());
    config.set("can_before", can_before_.to_json());
    config.set("can_after", can_after_.to_json());
    std::string path = run_dir_ + "/config.json";
    FILE* fp = std::fopen(path.c_str(), "w");
    if (fp != nullptr) {
      std::string text = config.dump(2);
      std::fwrite(text.data(), 1, text.size(), fp);
      std::fclose(fp);
    }
  }

  mj::Value summary = build_summary(stop);
  {
    std::string path = run_dir_ + "/summary.json";
    FILE* fp = std::fopen(path.c_str(), "w");
    if (fp != nullptr) {
      std::string text = summary.dump(1);
      std::fwrite(text.data(), 1, text.size(), fp);
      std::fclose(fp);
    }
  }

  std::string report = build_report(summary, stop);
  std::string path =
      run_dir_ + "/MVD-X6PLUS_通信统计_" + cfg_.stamp + "_" + num_text(cfg_.minutes) + "分钟.md";
  {
    FILE* fp = std::fopen(path.c_str(), "w");
    if (fp != nullptr) {
      std::fwrite(report.data(), 1, report.size(), fp);
      std::fclose(fp);
    }
  }
  {
    double mins = (t_start_ > 0.0) ? (t_end_ - t_start_) / 60.0 : 0.0;
    std::printf("\n运行结束：共 %.1f 分钟，状态 %s。\n日志目录：%s\n", mins, outcome_.c_str(),
                run_dir_.c_str());
    std::fflush(stdout);
  }
  log("日志目录：" + run_dir_);

  if (files_open_) {
    f_test_.flush();
    f_frames_.flush();
    f_events_.flush();
    f_health_.flush();
    f_test_.close();
    f_frames_.close();
    f_events_.close();
    f_health_.close();
    files_open_ = false;
  }
  return path;
}

}  // namespace mvd
