#!/usr/bin/env python3
"""Read HID scanner tags and append them to a JSONL file.

Example:
python3 scripts/test_hid_scanner_save.py \
  --device /dev/input/by-id/usb-STMicroelectronics_STM32_Custm_HID_0123456789AB-event-kbd \
  --output /home/wheeltec/wheeltec_ros2/rfid_scan_logs/test_hid_scanner_records.jsonl \
  --timeout 30 \
  --grab
"""

import argparse
import array
import errno
import fcntl
import json
import os
import select
import signal
import struct
import sys
import time
from datetime import datetime, timezone


DEFAULT_DEVICE = (
    "/dev/input/by-id/"
    "usb-STMicroelectronics_STM32_Custm_HID_0123456789AB-event-kbd"
)
DEFAULT_OUTPUT = "/home/wheeltec/wheeltec_ros2/rfid_scan_logs/test_hid_scanner_records.jsonl"

EV_KEY = 0x01
KEY_ENTER = 28
KEY_1 = 2
KEY_9 = 10
KEY_0 = 11
KEY_BACKSPACE = 14
KEY_LEFTSHIFT = 42
KEY_RIGHTSHIFT = 54
KEY_A = 30
KEY_Z = 44
KEY_KPENTER = 96
KEY_KP1 = 79
KEY_KP9 = 73
KEY_KP0 = 82

# _IOW("E", 0x90, int) from linux/input.h. The ioctl payload is one int.
EVIOCGRAB = 0x40044590

INPUT_EVENT = struct.Struct("llHHi")

LETTER_KEYS = {
    30: "a",
    48: "b",
    46: "c",
    32: "d",
    18: "e",
    33: "f",
    34: "g",
    35: "h",
    23: "i",
    36: "j",
    37: "k",
    38: "l",
    50: "m",
    49: "n",
    24: "o",
    25: "p",
    16: "q",
    19: "r",
    31: "s",
    20: "t",
    22: "u",
    47: "v",
    17: "w",
    45: "x",
    21: "y",
    44: "z",
}

DIGIT_KEYS = {
    2: "1",
    3: "2",
    4: "3",
    5: "4",
    6: "5",
    7: "6",
    8: "7",
    9: "8",
    10: "9",
    11: "0",
}

SHIFT_DIGIT_KEYS = {
    2: "!",
    3: "@",
    4: "#",
    5: "$",
    6: "%",
    7: "^",
    8: "&",
    9: "*",
    10: "(",
    11: ")",
}

KEYPAD_DIGIT_KEYS = {
    82: "0",
    79: "1",
    80: "2",
    81: "3",
    75: "4",
    76: "5",
    77: "6",
    71: "7",
    72: "8",
    73: "9",
}


class HidScannerSaveTool:
    def __init__(self, args):
        self.args = args
        self.fd = None
        self.output = None
        self.grabbed = False
        self.shift_for_next_key = False
        self.current_tag = []
        self.last_key_time = None
        self.tags_written = 0
        self.stop_requested = False

    def run(self):
        self._install_signal_handlers()
        self._open_device()
        self._open_output()
        if self.args.grab:
            self._grab_device()

        print(
            "HID scanner save test started. "
            f"device={self.args.device} output={self.args.output} "
            f"timeout={self.args.timeout}s max_tags={self.args.max_tags} "
            f"grab={self.args.grab}"
        )
        print("Scan tags now. Press Ctrl+C to exit.")

        deadline = time.monotonic() + self.args.timeout if self.args.timeout > 0 else None
        try:
            while not self.stop_requested:
                if self.args.max_tags > 0 and self.tags_written >= self.args.max_tags:
                    break
                if deadline is not None and time.monotonic() >= deadline:
                    break

                self._finish_pending_tag_after_quiet()
                timeout = self._poll_timeout(deadline)
                readable, _, _ = select.select([self.fd], [], [], timeout)
                if not readable:
                    self._finish_pending_tag_after_quiet()
                    continue
                self._read_available_events()
        finally:
            self._finish_pending_tag()
            self.close()

        print(f"Exit. saved_tags={self.tags_written}")

    def close(self):
        if self.fd is not None:
            if self.grabbed:
                try:
                    fcntl.ioctl(self.fd, EVIOCGRAB, array.array("i", [0]))
                except OSError:
                    pass
                self.grabbed = False
            try:
                self.fd.close()
            except OSError:
                pass
            self.fd = None
        if self.output is not None:
            self.output.close()
            self.output = None

    def _install_signal_handlers(self):
        def handle_signal(signum, frame):
            del signum, frame
            self.stop_requested = True

        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)

    def _open_device(self):
        if not os.path.exists(self.args.device):
            print(f"设备不存在: {self.args.device}", file=sys.stderr)
            print("请检查扫码枪设备路径，例如执行:", file=sys.stderr)
            print("  ls -l /dev/input/by-id/", file=sys.stderr)
            print("  ls -l /dev/input/by-path/", file=sys.stderr)
            raise SystemExit(2)

        try:
            self.fd = open(self.args.device, "rb", buffering=0)
        except PermissionError as exc:
            print(f"打开设备权限不足: {self.args.device}", file=sys.stderr)
            print(
                "当前用户可能不在 input 组，请执行 "
                "sudo usermod -aG input wheeltec 后重新登录。",
                file=sys.stderr,
            )
            raise SystemExit(2) from exc
        except OSError as exc:
            print(f"打开设备失败: {self.args.device}: {exc}", file=sys.stderr)
            raise SystemExit(2) from exc

        os.set_blocking(self.fd.fileno(), False)

    def _open_output(self):
        parent = os.path.dirname(os.path.abspath(self.args.output))
        if parent:
            os.makedirs(parent, exist_ok=True)
        self.output = open(self.args.output, "a", encoding="utf-8")

    def _grab_device(self):
        try:
            fcntl.ioctl(self.fd, EVIOCGRAB, array.array("i", [1]))
            self.grabbed = True
        except PermissionError as exc:
            print(f"EVIOCGRAB 权限不足: {self.args.device}", file=sys.stderr)
            print(
                "当前用户可能不在 input 组，请执行 "
                "sudo usermod -aG input wheeltec 后重新登录。",
                file=sys.stderr,
            )
            raise SystemExit(2) from exc
        except OSError as exc:
            print(f"EVIOCGRAB 失败: {self.args.device}: {exc}", file=sys.stderr)
            raise SystemExit(2) from exc

    def _poll_timeout(self, deadline):
        timeouts = [0.05]
        if self.current_tag and self.last_key_time is not None:
            quiet_sec = self.args.inter_char_timeout_ms / 1000.0
            elapsed = time.monotonic() - self.last_key_time
            timeouts.append(max(0.0, quiet_sec - elapsed))
        if deadline is not None:
            timeouts.append(max(0.0, deadline - time.monotonic()))
        return min(timeouts)

    def _read_available_events(self):
        while not self.stop_requested:
            try:
                data = self.fd.read(INPUT_EVENT.size)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                    break
                print(f"读取 input_event 失败: {exc}", file=sys.stderr)
                self.stop_requested = True
                break

            if not data:
                break
            if len(data) != INPUT_EVENT.size:
                print(f"input_event 长度异常: {len(data)}", file=sys.stderr)
                break

            _, _, event_type, code, value = INPUT_EVENT.unpack(data)
            if event_type == EV_KEY:
                self._process_key_event(code, value)

    def _process_key_event(self, code, value):
        if value != 1:
            return

        if code in (KEY_LEFTSHIFT, KEY_RIGHTSHIFT):
            self.shift_for_next_key = True
            return

        if code in (KEY_ENTER, KEY_KPENTER):
            self._finish_pending_tag()
            self.shift_for_next_key = False
            return

        if code == KEY_BACKSPACE:
            if self.current_tag:
                self.current_tag.pop()
                self.last_key_time = time.monotonic()
            self.shift_for_next_key = False
            return

        char = self._key_code_to_char(code)
        if char is None:
            self.shift_for_next_key = False
            return

        self.current_tag.append(char)
        self.last_key_time = time.monotonic()
        self.shift_for_next_key = False

    def _key_code_to_char(self, code):
        if code in LETTER_KEYS:
            char = LETTER_KEYS[code]
            return char.upper() if self.shift_for_next_key else char
        if code in DIGIT_KEYS:
            if self.shift_for_next_key:
                return SHIFT_DIGIT_KEYS.get(code, DIGIT_KEYS[code])
            return DIGIT_KEYS[code]
        if code in KEYPAD_DIGIT_KEYS:
            return KEYPAD_DIGIT_KEYS[code]
        return None

    def _finish_pending_tag_after_quiet(self):
        if not self.current_tag or self.last_key_time is None:
            return
        quiet_sec = self.args.inter_char_timeout_ms / 1000.0
        if time.monotonic() - self.last_key_time >= quiet_sec:
            self._finish_pending_tag()

    def _finish_pending_tag(self):
        if not self.current_tag:
            self.last_key_time = None
            return

        tag = "".join(self.current_tag)
        self.current_tag = []
        self.last_key_time = None
        if not tag:
            return

        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
                "+00:00", "Z"
            ),
            "event": "hid_scanner_tag_saved",
            "device": self.args.device,
            "tag": tag,
            "tag_index": self.tags_written + 1,
            "source": "hid_keyboard",
            "grab": bool(self.args.grab),
        }
        self.output.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.output.flush()
        self.tags_written += 1
        print(f"[{self.tags_written}] {tag}")


def parse_args():
    parser = argparse.ArgumentParser(description="Test HID scanner reading and JSONL saving.")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="Linux input event device path")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="JSONL output file path")
    parser.add_argument("--timeout", type=float, default=30.0, help="Total read timeout in seconds")
    parser.add_argument(
        "--inter-char-timeout-ms",
        type=int,
        default=200,
        help="Finish a tag after this quiet interval if Enter is not received",
    )
    parser.add_argument("--max-tags", type=int, default=0, help="Stop after this many tags; 0 means unlimited")
    parser.add_argument("--grab", action="store_true", help="Use EVIOCGRAB to exclusively grab the device")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.inter_char_timeout_ms < 1:
        print("--inter-char-timeout-ms 必须 >= 1", file=sys.stderr)
        return 2
    tool = HidScannerSaveTool(args)
    try:
        tool.run()
    except KeyboardInterrupt:
        tool.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
