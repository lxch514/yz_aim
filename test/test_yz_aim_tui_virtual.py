#!/usr/bin/env python3
"""AArch64 board PTY tests for the YZ-AIM maintenance frontends."""

import argparse
import fcntl
import os
import pathlib
import pty
import re
import select
import signal
import struct
import subprocess
import termios
import time
import unittest


ANSI_CSI = re.compile(rb"\x1b\[[0-?]*[ -/]*[@-~]")


def latest_frame(output):
    start_marker = b"\x1b[?2026h"
    end_marker = b"\x1b[?2026l"
    end = output.rfind(end_marker)
    if end < 0:
        raise AssertionError("no complete ANSI frame end in PTY output")
    start = output.rfind(start_marker, 0, end)
    if start < 0:
        raise AssertionError("no complete ANSI frame start in PTY output")
    start += len(start_marker)
    return ANSI_CSI.sub(b"", output[start:end]).decode(
        "utf-8", "replace"
    ).replace("\r", "")


class FrontendHarness:
    def __init__(self, binary, name, capture_dir=None):
        self.binary = str(pathlib.Path(binary).resolve())
        self.name = name
        self.capture_dir = pathlib.Path(capture_dir) if capture_dir else None
        self._wait_positions = {}

    def cli(self, *args, expect=0):
        result = subprocess.run(
            [self.binary, *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=5, check=False,
        )
        if result.returncode != expect:
            raise AssertionError(
                f"{self.name} {args}: rc={result.returncode}, output={result.stdout!r}"
            )
        return result.stdout

    def start(self, cols=120, rows=36, no_color=False, no_mouse=False, env_override=None):
        master, slave = pty.openpty()
        fcntl.ioctl(master, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
        before = termios.tcgetattr(slave)
        env = os.environ.copy()
        env.pop("YZ_AIM_MOCK_ABORT_ENABLE", None)
        if env_override:
            env.update(env_override)
        if no_color:
            env["NO_COLOR"] = ""
        args = [self.binary, "--mock"]
        if no_mouse:
            args.append("--no-mouse")
        process = subprocess.Popen(
            args, stdin=slave, stdout=slave, stderr=slave, close_fds=True, env=env
        )
        self._wait_positions[process.pid] = 0
        return process, master, slave, before, bytearray()

    @staticmethod
    def read_for(process, fd, output, duration=0.25):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], 0.04)
            if ready:
                try:
                    chunk = os.read(fd, 65536)
                except OSError:
                    break
                if chunk:
                    output.extend(chunk)
            if process.poll() is not None and not ready:
                break
        return output.decode("utf-8", errors="replace")

    def wait_for(self, process, fd, output, marker, timeout=6):
        deadline = time.monotonic() + timeout
        start = self._wait_positions.get(process.pid, 0)
        marker_bytes = marker.encode("utf-8")
        while time.monotonic() < deadline:
            text = self.read_for(process, fd, output, 0.12)
            if marker_bytes in bytes(output)[start:]:
                self._wait_positions[process.pid] = len(output)
                return text
            if process.poll() is not None:
                break
        raise AssertionError(
            f"{self.name}: did not observe {marker!r}; rc={process.poll()}; "
            f"last={bytes(output)[-1200:].decode('utf-8', 'replace')!r}"
        )

    @staticmethod
    def send(fd, data):
        os.write(fd, data.encode("ascii"))

    def finish(self, process, master, slave, before, output, send_q=True):
        if send_q and process.poll() is None:
            self.send(master, "q")
        deadline = time.monotonic() + 10
        while process.poll() is None and time.monotonic() < deadline:
            ready, _, _ = select.select([master], [], [], 0.05)
            if ready:
                try:
                    chunk = os.read(master, 65536)
                except OSError:
                    break
                if chunk:
                    output.extend(chunk)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2)
            raise AssertionError(f"{self.name}: process did not exit")
        try:
            while True:
                ready, _, _ = select.select([master], [], [], 0)
                if not ready:
                    break
                chunk = os.read(master, 65536)
                if not chunk:
                    break
                output.extend(chunk)
        except OSError:
            pass
        after = termios.tcgetattr(slave)
        if after != before:
            raise AssertionError(f"{self.name}: terminal attributes were not restored")
        text = output.decode("utf-8", errors="replace")
        self.close_fds(master, slave)
        self._wait_positions.pop(process.pid, None)
        return process.returncode, text

    def cleanup(self, process, master, slave):
        if process.poll() is None:
            try:
                self.send(master, "q\x1bq")
            except OSError:
                pass
            deadline = time.monotonic() + 3
            while process.poll() is None and time.monotonic() < deadline:
                try:
                    ready, _, _ = select.select([master], [], [], 0.05)
                    if ready:
                        os.read(master, 65536)
                except OSError:
                    break
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        self.close_fds(master, slave)
        self._wait_positions.pop(process.pid, None)

    @staticmethod
    def close_fds(master, slave):
        for fd in (master, slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def save_capture(self, label, output):
        if not self.capture_dir:
            return
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        target = self.capture_dir / f"{self.name}-{label}.ansi"
        target.write_bytes(bytes(output))


class YzAimTuiVirtual(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.setup = FrontendHarness(ARGS.setup, "setup", ARGS.capture_dir)
        cls.test = FrontendHarness(ARGS.test, "test", ARGS.capture_dir)

    def start(self, harness, **kwargs):
        result = harness.start(**kwargs)
        self.addCleanup(harness.cleanup, result[0], result[1], result[2])
        return result

    @staticmethod
    def send(fd, data):
        FrontendHarness.send(fd, data)

    @staticmethod
    def read_for(process, fd, output, duration=0.25):
        return FrontendHarness.read_for(process, fd, output, duration)

    def test_help_version_and_invalid_cli(self):
        self.assertIn("--bitrate", self.setup.cli("--help"))
        self.assertIn("yz_aim_setup 1.5.0", self.setup.cli("--version"))
        self.assertIn("yz_aim_test 1.5.0", self.test.cli("--version"))
        self.setup.cli("--bitrate", "300000", expect=2)
        self.test.cli("--node", "128", expect=2)
        self.test.cli("--bitrate", "500000", "--abit-timing", "bad", expect=2)

    def test_read_only_open_resize_help_no_color_and_exit(self):
        process, master, slave, before, output = self.start(self.setup,
            cols=80, rows=24, no_color=True, no_mouse=True
        )
        self.setup.wait_for(process, master, output, "读取：成功")
        self.read_for(process, master, output, 0.15)
        visible = output.decode("utf-8", errors="replace")[-12000:]
        diagnostic = visible[-2000:]
        for marker in ("报警", "输出", "已失能（原始值 0x0006）", "运行速度", "[s] 速度"):
            self.assertIn(marker, visible, diagnostic)
        for cols, rows in ((80, 24), (120, 36), (160, 50)):
            fcntl.ioctl(master, termios.TIOCSWINSZ,
                        struct.pack("HHHH", rows, cols, 0, 0))
            self.send(master, "?")
            self.read_for(process, master, output, 0.1)
            self.setup.save_capture(f"{cols}x{rows}", output)
            self.send(master, "?")
            self.read_for(process, master, output, 0.1)
        self.send(master, "q")
        rc, text = self.setup.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0)
        diagnostic = text[-2000:]
        self.assertIn("MOCK_WRITE_COUNT=0", diagnostic)
        self.assertIn("MOCK_HEARTBEAT_COUNT=0", diagnostic)
        self.assertNotRegex(text, re.compile(r"\x1b\[[0-9;]*m"), diagnostic)
        self.assertNotIn("\x1b[?1000h", text, diagnostic)

    def test_setup_cancel_then_single_enter_verified_setting(self):
        process, master, slave, before, output = self.start(self.setup, no_mouse=True)
        self.setup.wait_for(process, master, output, "读取：成功")
        self.send(master, "s")
        self.setup.wait_for(process, master, output, "请输入十进制数值")
        self.send(master, "12q")
        self.setup.wait_for(process, master, output, "输入已取消或超出范围")
        self.send(master, "s")
        self.setup.wait_for(process, master, output, "请输入十进制数值")
        self.send(master, "12\n")
        self.setup.wait_for(process, master, output, "运行速度：成功")
        self.send(master, "q")
        rc, text = self.setup.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        recent = text[-8000:]
        self.assertIn("12 电机 RPM / 0.235 输出 RPM", recent)
        self.assertIn("MOCK_WRITE_COUNT=1", text[-2000:])

    def test_node_and_bitrate_enter_confirmation_cancel_then_stage(self):
        process, master, slave, before, output = self.start(self.setup, no_mouse=True)
        self.setup.wait_for(process, master, output, "读取：成功")
        self.send(master, "n")
        self.setup.wait_for(process, master, output, "请输入十进制数值")
        self.send(master, "2\n")
        self.setup.wait_for(process, master, output, "YZ-AIM / 操作确认")
        self.send(master, "q")
        self.setup.wait_for(process, master, output, "已取消：节点")
        self.send(master, "n")
        self.setup.wait_for(process, master, output, "请输入十进制数值")
        self.send(master, "2\n")
        self.setup.wait_for(process, master, output, "YZ-AIM / 操作确认")
        self.send(master, "\n")
        self.setup.wait_for(process, master, output, "节点配置：成功")
        self.send(master, "b")
        self.setup.wait_for(process, master, output, "请输入十进制数值")
        self.send(master, "250000\n")
        self.setup.wait_for(process, master, output, "YZ-AIM / 操作确认")
        self.send(master, "\x1b")
        self.setup.wait_for(process, master, output, "已取消：波特率")
        self.send(master, "b")
        self.setup.wait_for(process, master, output, "请输入十进制数值")
        self.send(master, "250000\n")
        self.setup.wait_for(process, master, output, "YZ-AIM / 操作确认")
        self.send(master, "\n")
        self.setup.wait_for(process, master, output, "波特率配置：成功")
        self.read_for(process, master, output, 0.15)
        self.send(master, "q")
        rc, text = self.setup.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("节点=1", latest_frame(output))
        self.assertIn("MOCK_WRITE_COUNT=6", text[-2000:])

    def test_space_stop_is_separate_from_output_disable(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, " ")
        self.test.wait_for(process, master, output, "停止请求：成功")
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_STOP_010F_WRITES=0", text[-2000:])
        self.assertIn("MOCK_OUTPUT_DISABLE_WRITES=0", text[-2000:])

    def test_action_failure_remains_visible_after_periodic_read(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "m")
        self.test.wait_for(process, master, output, "相对运动：失败：请先按 e 使能")
        self.read_for(process, master, output, 2.5)
        frame = latest_frame(output)
        self.assertIn("相对运动：失败：请先按 e 使能", frame, frame[-2000:])
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])

    def test_setup_escape_cancel_does_not_write(self):
        process, master, slave, before, output = self.start(self.setup, no_mouse=True)
        self.setup.wait_for(process, master, output, "读取：成功")
        self.send(master, "a")
        self.setup.wait_for(process, master, output, "请输入十进制数值")
        self.send(master, "\x1b")
        self.setup.wait_for(process, master, output, "输入已取消或超出范围")
        self.send(master, "q")
        rc, text = self.setup.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])

    def test_enabled_idle_services_heartbeat(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.read_for(process, master, output, 1.1)
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        diagnostic = text[-2000:]
        self.assertIn("MOCK_OUTPUT_DISABLE_WRITES=1", diagnostic)
        match = re.search(r"MOCK_HEARTBEAT_COUNT=(\d+)", diagnostic)
        self.assertIsNotNone(match, diagnostic)
        self.assertGreaterEqual(int(match.group(1)), 4, diagnostic)

    def test_single_key_enable_and_disable_confirm_output_state(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.read_for(process, master, output, 0.15)
        self.assertIn("已使能（原始值 0x0007）", latest_frame(output))
        self.send(master, "d")
        self.test.wait_for(process, master, output, "输出失能：成功")
        self.read_for(process, master, output, 0.15)
        self.assertIn("已失能（原始值 0x0006）", latest_frame(output))
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_OUTPUT_DISABLE_WRITES=1", text[-2000:])

    def test_short_jog_then_space_stop_restores_terminal(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.send(master, "j")
        self.test.wait_for(process, master, output, "点动：成功")
        self.send(master, " ")
        self.test.wait_for(process, master, output, "停止请求：成功")
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_OUTPUT_DISABLE_WRITES=1", text[-2000:])

    def test_positive_and_negative_moves_return_to_start(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.send(master, "m")
        self.test.wait_for(process, master, output, "相对运动：成功")
        self.read_for(process, master, output, 0.15)
        self.assertIn("24411 电机脉冲", latest_frame(output))
        self.send(master, "M")
        self.test.wait_for(process, master, output, "相对运动：成功")
        self.read_for(process, master, output, 0.15)
        self.assertIn("1200 电机脉冲", latest_frame(output))
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_OUTPUT_DISABLE_WRITES=1", text[-2000:])

    def test_absolute_position_and_readonly_reboot_inspection(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "p")
        self.test.wait_for(process, master, output, "输入带符号十进制绝对目标")
        self.send(master, "999999999999\n")
        self.test.wait_for(process, master, output, "请输入有符号32位十进制位置")
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])

        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.send(master, "p")
        self.test.wait_for(process, master, output, "输入带符号十进制绝对目标")
        self.send(master, "1500\n")
        self.test.wait_for(process, master, output, "绝对位置：成功")
        self.read_for(process, master, output, 0.15)
        self.assertIn("1500 电机脉冲", latest_frame(output))
        self.send(master, "d")
        self.test.wait_for(process, master, output, "输出失能：成功")
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_OUTPUT_DISABLE_WRITES=1", text[-2000:])

    def test_absolute_prompt_uses_current_value_and_valid_target_interval(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "p")
        self.test.wait_for(process, master, output, "输入带符号十进制绝对目标")
        frame = latest_frame(output)
        for hint in ("当前 1200", "p 是绝对目标", "24411", "[-1966080000, 1966080000]"):
            self.assertIn(hint, frame, frame[-2000:])
        self.send(master, "1200\n")
        self.test.wait_for(process, master, output, "绝对目标须在 [-1966080000, 1966080000] 电机脉冲内且不等于当前")
        self.assertNotIn("首个故障", latest_frame(output))
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])
        self.assertNotIn("YZ-AIM 首个故障：", text[-2000:])

    def test_main_panel_speed_and_help_are_read_only(self):
        process, master, slave, before, output = self.start(
            self.test, cols=80, rows=24, no_mouse=True
        )
        self.test.wait_for(process, master, output, "读取：成功")
        frame = latest_frame(output)
        for label in ("[e] 使能", "[v] 速度", "[r] 相对", "[a] 绝对",
                      "[d] 失能", "[空格] 停", "[?] 帮助", "选定速度", "位置"):
            self.assertIn(label, frame, frame)
        self.send(master, "v")
        self.test.wait_for(process, master, output, "输入 1~3000 电机 RPM")
        self.send(master, "6\n")
        self.test.wait_for(process, master, output, "下次运动 6 电机 RPM")
        self.assertIn("6 电机 RPM", latest_frame(output))
        self.send(master, "?")
        self.test.wait_for(process, master, output, "高级：m/M")
        self.send(master, "?")
        self.test.wait_for(process, master, output, "选定速度")
        self.test.save_capture("80x24-main", output)
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])
        self.assertIn("MOCK_PROFILE_RPM_WRITES=", text[-2000:])

    def test_selected_speed_live_actual_stop_and_resume(self):
        process, master, slave, before, output = self.start(
            self.test, cols=80, rows=24, no_mouse=True
        )
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "v")
        self.test.wait_for(process, master, output, "输入 1~3000 电机 RPM")
        self.send(master, "6\n")
        self.test.wait_for(process, master, output, "下次运动 6 电机 RPM")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.send(master, "r")
        self.test.wait_for(process, master, output, "输入相对输出角度")
        self.send(master, "360\n")
        self.test.wait_for(process, master, output, "本次目标")
        self.read_for(process, master, output, 0.42)
        before_stop = bytes(output)
        self.test.save_capture("80x24-moving-360deg", before_stop)
        frames = []
        for candidate in before_stop.split(b"\x1b[?2026l"):
            if b"\x1b[?2026h" in candidate:
                frames.append(ANSI_CSI.sub(b"", candidate.split(b"\x1b[?2026h")[-1]).decode(
                    "utf-8", "replace"
                ))
        midframes = [f for f in frames if "本次目标" in f and "进行中" in f]
        self.assertGreaterEqual(len(midframes), 2, "no in-motion UI frames")
        self.assertTrue(any("目标" in f and "1672368" in f for f in midframes))
        self.assertTrue(any(all(key in f for key in ("[v] 速度", "[r] 相对", "[a] 绝对"))
                            for f in midframes), "80x24 motion lost primary controls")
        self.assertTrue(any("位置" in f and "1200 电机脉冲" not in f for f in midframes),
                        "actual position did not update before completion")
        self.send(master, " ")
        self.test.wait_for(process, master, output, "已确认停止，位置动作已取消")
        self.send(master, "r")
        self.test.wait_for(process, master, output, "输入相对输出角度")
        self.send(master, "-1\n")
        self.test.wait_for(process, master, output, "相对运动：成功", timeout=8)
        self.send(master, "d")
        self.test.wait_for(process, master, output, "输出失能：成功")
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_PROFILE_RPM_WRITES=6,6", text[-2000:])
        self.assertNotIn("YZ-AIM 首个故障", text[-2000:])

    def test_absolute_output_angle_input_and_range(self):
        process, master, slave, before, output = self.start(self.test, no_mouse=True)
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "a")
        self.test.wait_for(process, master, output, "输入绝对输出角度")
        self.send(master, "423530\n")
        self.test.wait_for(process, master, output, "目标超出手册总轴位")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.send(master, "a")
        self.test.wait_for(process, master, output, "输入绝对输出角度")
        self.send(master, "1\n")
        self.test.wait_for(process, master, output, "绝对位置：成功")
        self.assertIn("4642 电机脉冲", latest_frame(output))
        self.send(master, "d")
        self.test.wait_for(process, master, output, "输出失能：成功")
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_PROFILE_RPM_WRITES=30", text[-2000:])

    def test_selected_3000_and_out_of_range_speed_have_no_writes(self):
        process, master, slave, before, output = self.start(
            self.test, no_mouse=True, cols=80, rows=24
        )
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "v")
        self.test.wait_for(process, master, output, "输入 1~3000 电机 RPM")
        self.send(master, "3000\n")
        self.test.wait_for(process, master, output, "下次运动 3000 电机 RPM")
        self.send(master, "v")
        self.test.wait_for(process, master, output, "输入 1~3000 电机 RPM")
        self.send(master, "3001\n")
        self.test.wait_for(process, master, output, "输入已取消或超出范围")
        self.assertIn("3000 电机 RPM", latest_frame(output))
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])

    def test_360_degree_relative_absolute_targets_and_3000_rpm(self):
        process, master, slave, before, output = self.start(
            self.test, no_mouse=True, cols=80, rows=24
        )
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "v")
        self.test.wait_for(process, master, output, "输入 1~3000 电机 RPM")
        self.send(master, "3000\n")
        self.test.wait_for(process, master, output, "下次运动 3000 电机 RPM")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "安全检查并使能：成功")
        self.send(master, "r")
        self.test.wait_for(process, master, output, "输入相对输出角度")
        self.send(master, "360\n")
        self.test.wait_for(process, master, output, "本次目标")
        frame = latest_frame(output)
        for marker in ("3000 电机 RPM", "[v] 速度", "[r] 相对", "[a] 绝对", "1672368"):
            self.assertIn(marker, frame)
        self.test.save_capture("80x24-3000rpm-360deg-moving", output)
        self.test.wait_for(process, master, output, "相对运动：成功", timeout=15)
        self.assertIn("1672368 电机脉冲", latest_frame(output))
        self.send(master, "a")
        self.test.wait_for(process, master, output, "输入绝对输出角度")
        self.send(master, "360\n")
        self.test.wait_for(process, master, output, "绝对位置：成功", timeout=15)
        self.assertIn("1671168 电机脉冲", latest_frame(output))
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_PROFILE_RPM_WRITES=3000,3000", text[-2000:])
        self.assertIn("MOCK_POSITION_TARGET_WRITES=1200,1671168,1672368,1671168", text[-2000:])

    def test_manual_absolute_endpoints_and_out_of_range_no_writes(self):
        for position, outside in ((1966080000, 1966080001), (-1966080000, -1966080001)):
            with self.subTest(position=position):
                process, master, slave, before, output = self.start(self.test, no_mouse=True)
                self.test.wait_for(process, master, output, "读取：成功")
                self.send(master, "p")
                self.test.wait_for(process, master, output, "输入带符号十进制绝对目标")
                self.send(master, f"{outside}\n")
                self.test.wait_for(process, master, output, "绝对目标须在 [-1966080000, 1966080000]")
                self.send(master, "q")
                rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
                self.assertEqual(rc, 0, text[-2000:])
                self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])

                process, master, slave, before, output = self.start(self.test, no_mouse=True)
                self.test.wait_for(process, master, output, "读取：成功")
                self.send(master, "e")
                self.test.wait_for(process, master, output, "安全检查并使能：成功")
                self.send(master, "p")
                self.test.wait_for(process, master, output, "输入带符号十进制绝对目标")
                self.send(master, f"{position}\n")
                self.test.wait_for(process, master, output, "绝对位置：成功", timeout=20)
                self.assertIn(f"{position} 电机脉冲", latest_frame(output))
                self.send(master, "q")
                rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
                self.assertEqual(rc, 0, text[-2000:])
                self.assertIn(f"MOCK_POSITION_TARGET_WRITES=1200,1200,{position}", text[-2000:])

    def test_relative_manual_endpoint_is_checked_before_scheduling(self):
        process, master, slave, before, output = self.start(
            self.test, no_mouse=True, env_override={"YZ_AIM_MOCK_POSITION": "1966080000"}
        )
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "r")
        self.test.wait_for(process, master, output, "输入相对输出角度")
        self.assertIn("当前允许增量脉冲", latest_frame(output))
        self.send(master, "1\n")
        self.test.wait_for(process, master, output, "目标超出手册总轴位")
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertEqual(rc, 0, text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])

    def test_first_transport_abort_survives_refresh_and_exit(self):
        process, master, slave, before, output = self.start(
            self.test, no_mouse=True, env_override={"YZ_AIM_MOCK_ABORT_ENABLE": "1"}
        )
        self.test.wait_for(process, master, output, "读取：成功")
        self.send(master, "e")
        self.test.wait_for(process, master, output, "首个故障：安全检查并使能：失败：从站拒绝 SDO")
        self.send(master, "R")
        self.test.wait_for(process, master, output, "读取：成功")
        frame = latest_frame(output)
        self.assertIn("首个故障：安全检查并使能：失败：从站拒绝 SDO", frame)
        self.send(master, "q")
        rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
        self.assertIn(rc, (0, 6), text[-2000:])
        self.assertIn("YZ-AIM 首个故障：安全检查并使能：失败：从站拒绝 SDO", text[-2000:])
        self.assertIn("MOCK_WRITE_COUNT=", text[-2000:])

    def test_sigint_and_sigterm_restore_terminal(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            process, master, slave, before, output = self.start(self.test, no_mouse=True)
            self.test.wait_for(process, master, output, "读取：成功")
            process.send_signal(signum)
            rc, text = self.test.finish(process, master, slave, before, output, send_q=False)
            self.assertEqual(rc, 0, text[-500:])
            self.assertIn("MOCK_WRITE_COUNT=0", text[-2000:])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True, help="AArch64 yz_aim_setup binary")
    parser.add_argument("--test", required=True, help="AArch64 yz_aim_test binary")
    parser.add_argument("--capture-dir", help="optional directory for representative ANSI PTY captures")
    global ARGS
    ARGS = parser.parse_args()
    unittest.main(argv=[__file__], verbosity=2)


if __name__ == "__main__":
    main()
