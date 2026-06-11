#!/usr/bin/env python3
"""UHFReader188 RFID reader protocol client (answer mode / command-response).

Supports:
  - CRC16 calculation (polynomial 0x8408, init 0xFFFF)
  - Command building (Len + Addr + Cmd + Data + CRC_L + CRC_H)
  - Response reading with timeout
  - Response parsing (Len, Addr, reCmd, Status, Data, CRC)
  - 0x01 short-format multi-tag inventory (QValue + Session only)
  - 0x0F single-tag inventory
  - Reader info query (0x21)
  - Multi-frame collection for status=0x03 continuation
  - Per-cell multi-round inventory with deduplication

Tested on UHFReader188 with Prolific USB-Serial adapter at 57600 baud.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    import serial  # pyserial
except ImportError:
    serial = None  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Status codes
STATUS_OK = 0x00
STATUS_INVENTORY_OK = 0x01
STATUS_INVENTORY_TIMEOUT = 0x02
STATUS_FOLLOW_FRAMES = 0x03
STATUS_TAG_CACHE_FULL = 0x04
STATUS_ANTENNA_ERROR = 0xF8
STATUS_COMMAND_ERROR = 0xF9
STATUS_COMM_UNSTABLE = 0xFA
STATUS_NO_TAG = 0xFB
STATUS_LENGTH_ERROR = 0xFD
STATUS_CRC_ERROR = 0xFE
STATUS_PARAM_ERROR = 0xFF

STATUS_DESCRIPTIONS: Dict[int, str] = {
    0x00: "普通命令成功",
    0x01: "询查正常返回",
    0x02: "询查时间溢出，可解析已有Data",
    0x03: "本条消息之后还有消息",
    0x04: "标签太多或缓存满",
    0xF8: "天线连接检测错误",
    0xF9: "命令执行出错",
    0xFA: "有标签但通信不畅",
    0xFB: "无电子标签可操作",
    0xFD: "命令长度错误",
    0xFE: "非法命令或CRC错误",
    0xFF: "参数错误",
}

# Commands
CMD_INVENTORY_MULTIPLE = 0x01
CMD_INVENTORY_SINGLE = 0x0F
CMD_READER_INFO = 0x21

# Reader type mapping
READER_TYPES: Dict[int, str] = {
    0x06: "UHFReader188",
    0x09: "UHFReader102",
    0x0D: "UHFReader188",
}


# ---------------------------------------------------------------------------
# CRC16
# ---------------------------------------------------------------------------

def crc16_uhf(data: bytes) -> int:
    """Calculate CRC16 for UHFReader188 protocol.

    Initial value: 0xFFFF
    Polynomial: 0x8408 (bit-reversed form of 0x1021)
    Returns 16-bit CRC with low byte first in frame.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# Command building
# ---------------------------------------------------------------------------

def build_cmd(addr: int, cmd: int, data: bytes = b"") -> bytes:
    """Build a command frame for UHFReader188.

    Frame format: Len Addr Cmd Data CRC_L CRC_H
    - Len does NOT include itself
    - Len = 4 + len(Data)
    - CRC is calculated over [Len, Addr, Cmd, Data]

    Args:
        addr: Reader address (0x00 for broadcast)
        cmd: Command byte
        data: Command data bytes

    Returns:
        Complete frame bytes ready to send
    """
    length = 4 + len(data)
    frame = bytes([length, addr, cmd]) + data
    crc = crc16_uhf(frame)
    crc_l = crc & 0xFF
    crc_h = (crc >> 8) & 0xFF
    return frame + bytes([crc_l, crc_h])


def build_inventory_short(addr: int, q_value: int = 2, session: int = 0) -> bytes:
    """Build 0x01 short-format multi-tag inventory command.

    Short format only includes QValue + Session (no AdrTID, no LenTID).

    Args:
        addr: Reader address
        q_value: Q value (0-8), controls number of slots
        session: Session bank (0-3, i.e. S0-S3)

    Returns:
        Complete frame bytes
    """
    data = bytes([q_value & 0x07, session & 0x03])
    return build_cmd(addr, CMD_INVENTORY_MULTIPLE, data)


def build_inventory_single(addr: int) -> bytes:
    """Build 0x0F single-tag inventory command (no data bytes)."""
    return build_cmd(addr, CMD_INVENTORY_SINGLE)


def build_reader_info_cmd(addr: int) -> bytes:
    """Build 0x21 reader information query command."""
    return build_cmd(addr, CMD_READER_INFO)


# ---------------------------------------------------------------------------
# Frame reading
# ---------------------------------------------------------------------------

def read_frame(
    ser: "serial.Serial",
    timeout_sec: float = 2.5,
    verbose: bool = False,
) -> Tuple[bytes, str]:
    """Read one complete response frame from serial port.

    Reads 1 byte for Len, then reads Len bytes for the rest of the frame.
    Validates length and CRC.

    Args:
        ser: Open serial port
        timeout_sec: Maximum time to wait for a frame
        verbose: Print hex debug info

    Returns:
        (frame_bytes, error_string)
        error_string is empty on success.
    """
    if serial is None:
        return b"", "pyserial not installed"

    old_timeout = ser.timeout
    try:
        ser.timeout = timeout_sec

        # Read Len byte
        len_byte = ser.read(1)
        if len(len_byte) == 0:
            return b"", "timeout waiting for Len byte"

        if verbose:
            print(f"  [RX] Len=0x{len_byte[0]:02X} ({len_byte[0]})")

        frame_len = len_byte[0]
        if frame_len < 4:
            return b"", f"invalid frame length: {frame_len} (minimum is 4)"

        # Read remaining bytes
        remaining = ser.read(frame_len)
        if len(remaining) < frame_len:
            return b"", f"incomplete frame: got {len(remaining)} of {frame_len} bytes"

        frame = len_byte + remaining

        if verbose:
            hex_str = " ".join(f"{b:02X}" for b in frame)
            print(f"  [RX] Full frame: {hex_str}")

        # Validate CRC
        calc_crc = crc16_uhf(frame[:-2])
        recv_crc = frame[-2] | (frame[-1] << 8)
        if calc_crc != recv_crc:
            return frame, (
                f"CRC mismatch: calc=0x{calc_crc:04X} recv=0x{recv_crc:04X}"
            )

        return frame, ""

    finally:
        ser.timeout = old_timeout


# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------

class ParsedResponse:
    """Parsed UHFReader188 response frame."""

    __slots__ = ("length", "addr", "re_cmd", "status", "data", "crc_ok", "raw")

    def __init__(
        self,
        length: int = 0,
        addr: int = 0,
        re_cmd: int = 0,
        status: int = 0,
        data: bytes = b"",
        crc_ok: bool = True,
        raw: bytes = b"",
    ) -> None:
        self.length = length
        self.addr = addr
        self.re_cmd = re_cmd
        self.status = status
        self.data = data
        self.crc_ok = crc_ok
        self.raw = raw

    def status_ok(self) -> bool:
        """Return True if status indicates success (0x00, 0x01, 0x02, 0x03, 0x04)."""
        return self.status in (0x00, 0x01, 0x02, 0x03, 0x04)

    def has_more_frames(self) -> bool:
        """Return True if status=0x03 indicates continuation frames."""
        return self.status == STATUS_FOLLOW_FRAMES

    def status_description(self) -> str:
        return STATUS_DESCRIPTIONS.get(self.status, f"未知状态 0x{self.status:02X}")


def parse_response(frame: bytes) -> ParsedResponse:
    """Parse a UHFReader188 response frame.

    Frame format: Len Addr reCmd Status Data... CRC_L CRC_H

    Args:
        frame: Complete frame bytes including Len and CRC

    Returns:
        ParsedResponse object
    """
    if len(frame) < 5:
        return ParsedResponse(crc_ok=False, raw=frame)

    length = frame[0]
    addr = frame[1]
    re_cmd = frame[2]
    status = frame[3]
    data = frame[4:-2]

    # CRC already validated in read_frame, but parse it
    calc_crc = crc16_uhf(frame[:-2])
    recv_crc = frame[-2] | (frame[-1] << 8)
    crc_ok = calc_crc == recv_crc

    return ParsedResponse(
        length=length,
        addr=addr,
        re_cmd=re_cmd,
        status=status,
        data=data,
        crc_ok=crc_ok,
        raw=frame,
    )


# ---------------------------------------------------------------------------
# Tag parsing
# ---------------------------------------------------------------------------

class TagInfo:
    """Parsed RFID tag information."""

    __slots__ = ("epc", "rssi", "antenna")

    def __init__(self, epc: str, rssi: int = 0, antenna: int = 0) -> None:
        self.epc = epc
        self.rssi = rssi
        self.antenna = antenna

    def __repr__(self) -> str:
        return f"TagInfo(epc={self.epc!r}, rssi={self.rssi}, antenna={self.antenna})"

    def __eq__(self, other: object) -> bool:
        if isinstance(other, TagInfo):
            return self.epc == other.epc
        return NotImplemented

    def __hash__(self) -> int:
        return hash(self.epc)


def parse_inventory_tags(data: bytes) -> List[TagInfo]:
    """Parse tag data from inventory response.

    Supports both 0x01 and 0x0F inventory responses.

    Standard format:  Num + EPC_Len + EPC + RSSI
    Possible variant: Num + Ant + EPC_Len + EPC + RSSI

    The parser auto-detects the format by checking if the second byte
    looks like a valid EPC length (even number, 4-32 typical EPC bytes).

    Args:
        data: Response data bytes (after status byte)

    Returns:
        List of TagInfo objects
    """
    tags: List[TagInfo] = []
    offset = 0

    if offset >= len(data):
        return tags

    # First byte is Num (number of tags in this frame)
    num_tags = data[offset]
    offset += 1

    for _ in range(num_tags):
        if offset >= len(data):
            break

        # Check if next byte is antenna or EPC length
        # Heuristic: EPC length is even and in range [4, 32]
        # Antenna is typically 1-8
        candidate_epc_len = data[offset]
        next_after_epc_len = offset + 1 + candidate_epc_len

        if (
            candidate_epc_len >= 4
            and candidate_epc_len <= 32
            and candidate_epc_len % 2 == 0
            and next_after_epc_len <= len(data)
        ):
            # Standard format: EPC_Len + EPC + RSSI
            antenna = 0
            epc_len = candidate_epc_len
            offset += 1  # skip EPC_Len
        elif (
            offset + 1 < len(data)
        ):
            # Variant format: Ant + EPC_Len + EPC + RSSI
            antenna = candidate_epc_len
            offset += 1  # skip Ant
            if offset >= len(data):
                break
            epc_len = data[offset]
            offset += 1  # skip EPC_Len
            if epc_len < 4 or epc_len > 32 or epc_len % 2 != 0:
                # Not a valid EPC length, skip this tag
                break
            if offset + epc_len + 1 > len(data):
                break
        else:
            break

        # Read EPC
        if offset + epc_len > len(data):
            break
        epc_bytes = data[offset:offset + epc_len]
        epc = "".join(f"{b:02X}" for b in epc_bytes)
        offset += epc_len

        # Read RSSI (1 byte)
        rssi = 0
        if offset < len(data):
            rssi = data[offset]
            offset += 1

        tags.append(TagInfo(epc=epc, rssi=rssi, antenna=antenna))

    return tags


# ---------------------------------------------------------------------------
# Reader info parsing
# ---------------------------------------------------------------------------

class ReaderInfo:
    """Parsed UHFReader188 reader information."""

    __slots__ = (
        "reader_type", "reader_type_name", "tr_type",
        "power", "scan_time", "version", "dmaxfre", "dminfre",
        "raw_data",
    )

    def __init__(self) -> None:
        self.reader_type: int = 0
        self.reader_type_name: str = ""
        self.tr_type: int = 0
        self.power: int = 0
        self.scan_time: int = 0
        self.version: str = ""
        self.dmaxfre: int = 0
        self.dminfre: int = 0
        self.raw_data: bytes = b""

    def __repr__(self) -> str:
        return (
            f"ReaderInfo(type={self.reader_type_name!r} (0x{self.reader_type:02X}), "
            f"tr_type=0x{self.tr_type:02X}, power={self.power}, "
            f"scan_time={self.scan_time}, dmaxfre=0x{self.dmaxfre:02X}, "
            f"dminfre=0x{self.dminfre:02X}, version={self.version!r})"
        )


def parse_reader_info(data: bytes) -> ReaderInfo:
    """Parse reader info response data.

    Data format for UHFReader188 (reCmd=0x21), 10 bytes total:
        Version[2] Type[1] Tr_Type[1] dmaxfre[1] dminfre[1] Power[1] Scntm[1] Reserved[2]

    Example raw data: 06 46 0D 02 31 80 21 0A 00 00
        Version  = 06 46
        Type     = 0x0D (UHFReader188)
        Tr_Type  = 0x02 (EPC C1G2 / ISO18000-6C)
        dmaxfre  = 0x31
        dminfre  = 0x80
        Power    = 0x21 = 33
        Scntm    = 0x0A = 10 (1000ms)
        Reserved = 00 00

    Args:
        data: Response data bytes (after status byte)

    Returns:
        ReaderInfo object
    """
    info = ReaderInfo()
    info.raw_data = data

    if len(data) < 10:
        return info

    # Version: 2 bytes
    version_bytes = data[0:2]
    info.version = "".join(f"{b:02X}" for b in version_bytes)

    # Type: 1 byte at offset 2
    info.reader_type = data[2]
    info.reader_type_name = READER_TYPES.get(
        info.reader_type, f"Unknown(0x{info.reader_type:02X})")

    # Tr_Type: 1 byte at offset 3
    info.tr_type = data[3]

    # dmaxfre: 1 byte at offset 4
    info.dmaxfre = data[4]

    # dminfre: 1 byte at offset 5
    info.dminfre = data[5]

    # Power: 1 byte at offset 6
    info.power = data[6]

    # Scan time (Scntm): 1 byte at offset 7
    info.scan_time = data[7]

    return info


# ---------------------------------------------------------------------------
# High-level inventory operations
# ---------------------------------------------------------------------------

class UHFReader188:
    """High-level interface to UHFReader188 in answer mode."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        address: int = 0x00,
        timeout_sec: float = 2.5,
        verbose: bool = False,
    ) -> None:
        """Initialize the reader connection parameters.

        Args:
            port: Serial port device path
            baudrate: Serial baud rate (default 57600 for UHFReader188)
            address: Reader address (default 0x00)
            timeout_sec: Frame read timeout in seconds
            verbose: Enable hex debug logging
        """
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.timeout_sec = timeout_sec
        self.verbose = verbose
        self._serial: Optional["serial.Serial"] = None

    def open(self) -> str:
        """Open serial connection. Returns error message or empty string on success."""
        if serial is None:
            return "pyserial is not installed. Run: pip install pyserial"
        self.close()
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout_sec,
                write_timeout=self.timeout_sec,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            return ""
        except Exception as e:
            return f"Failed to open {self.port}: {e}"

    def close(self) -> None:
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def _send_and_receive(self, cmd_bytes: bytes) -> Tuple[Optional[ParsedResponse], str]:
        """Send command and read response.

        Returns:
            (ParsedResponse or None, error_string)
        """
        if not self.is_open:
            return None, "serial port not open"

        assert self._serial is not None
        try:
            self._serial.reset_input_buffer()
            written = self._serial.write(cmd_bytes)
            if written is None or written < len(cmd_bytes):
                return None, f"incomplete write: {written}/{len(cmd_bytes)}"

            if self.verbose:
                hex_str = " ".join(f"{b:02X}" for b in cmd_bytes)
                print(f"  [TX] {hex_str}")

            frame, err = read_frame(self._serial, self.timeout_sec, self.verbose)
            if err:
                return None, err

            parsed = parse_response(frame)
            if not parsed.crc_ok:
                return parsed, "CRC verification failed"

            return parsed, ""

        except Exception as e:
            return None, f"serial I/O error: {e}"

    def read_reader_info(self) -> Tuple[Optional[ReaderInfo], str]:
        """Query reader information.

        Returns:
            (ReaderInfo or None, error_string)
        """
        cmd = build_reader_info_cmd(self.address)
        resp, err = self._send_and_receive(cmd)
        if err:
            return None, err
        assert resp is not None
        if resp.status != STATUS_OK:
            return None, f"reader info failed: status=0x{resp.status:02X} {resp.status_description()}"
        info = parse_reader_info(resp.data)
        return info, ""

    def inventory_single(self) -> Tuple[List[TagInfo], int, str]:
        """Perform single-tag inventory (0x0F).

        Returns:
            (tags_list, status_code, error_string)
        """
        cmd = build_inventory_single(self.address)
        resp, err = self._send_and_receive(cmd)
        if err:
            return [], -1, err
        assert resp is not None

        tags = []
        if resp.status in (STATUS_INVENTORY_OK, STATUS_INVENTORY_TIMEOUT, STATUS_FOLLOW_FRAMES):
            tags = parse_inventory_tags(resp.data)

        return tags, resp.status, ""

    def inventory_once(
        self,
        q_value: int = 2,
        session: int = 0,
        collect_follow_frames: bool = True,
        max_follow_frames: int = 10,
    ) -> Tuple[List[TagInfo], int, str]:
        """Perform one multi-tag inventory round using 0x01 short format.

        Sends: 0x01 short format with QValue + Session only.
        Collects all continuation frames if status=0x03.

        Args:
            q_value: Q value (0-8)
            session: Session bank (0-3)
            collect_follow_frames: Whether to read continuation frames
            max_follow_frames: Maximum continuation frames to read

        Returns:
            (deduplicated_tags_list, final_status, error_string)
        """
        cmd = build_inventory_short(self.address, q_value, session)
        resp, err = self._send_and_receive(cmd)
        if err:
            return [], -1, err
        assert resp is not None

        all_tags: List[TagInfo] = []
        seen_epcs: set = set()
        current_resp = resp

        while True:
            if current_resp.status in (
                STATUS_INVENTORY_OK,
                STATUS_INVENTORY_TIMEOUT,
                STATUS_TAG_CACHE_FULL,
                STATUS_FOLLOW_FRAMES,
            ):
                tags = parse_inventory_tags(current_resp.data)
                for tag in tags:
                    if tag.epc not in seen_epcs:
                        seen_epcs.add(tag.epc)
                        all_tags.append(tag)

            if (
                collect_follow_frames
                and current_resp.has_more_frames()
                and max_follow_frames > 0
            ):
                frame, frame_err = read_frame(self._serial, self.timeout_sec, self.verbose)
                if frame_err:
                    return all_tags, current_resp.status, f"follow frame error: {frame_err}"
                current_resp = parse_response(frame)
                max_follow_frames -= 1
            else:
                break

        return all_tags, current_resp.status, ""

    def inventory_cell(
        self,
        scan_rounds: int = 5,
        scan_interval: float = 0.5,
        q_value: int = 2,
        session: int = 0,
        fallback_single: bool = True,
        collect_follow_frames: bool = True,
        max_follow_frames: int = 10,
    ) -> Dict:
        """Perform inventory for one cell (multiple rounds with deduplication).

        Args:
            scan_rounds: Number of inventory rounds
            scan_interval: Sleep between rounds in seconds
            q_value: Q value for 0x01 inventory
            session: Session bank for 0x01 inventory
            fallback_single: Try 0x0F if 0x01 yields no tags
            collect_follow_frames: Collect continuation frames
            max_follow_frames: Max continuation frames per round

        Returns:
            Dict with keys: epcs, rssi_map, rounds_completed, status, error
        """
        result: Dict = {
            "epcs": [],
            "rssi_map": {},
            "rounds_completed": 0,
            "status": "ok",
            "error": "",
            "reader_mode": "uhf_reader188_answer_serial",
            "source": "answer_mode_serial",
        }

        all_epcs: set = set()
        rssi_map: Dict[str, int] = {}

        for i in range(scan_rounds):
            tags, status, err = self.inventory_once(
                q_value=q_value,
                session=session,
                collect_follow_frames=collect_follow_frames,
                max_follow_frames=max_follow_frames,
            )

            result["rounds_completed"] = i + 1

            if err:
                if status == STATUS_ANTENNA_ERROR:
                    result["status"] = "hardware_error"
                    result["error"] = f"antenna error: {err}"
                    return result
                if status == STATUS_CRC_ERROR:
                    result["status"] = "crc_error"
                    result["error"] = f"CRC error: {err}"
                    continue
                # Other errors: log and continue
                if self.verbose:
                    print(f"  Round {i + 1} error: {err}")
                continue

            for tag in tags:
                if tag.epc not in all_epcs:
                    all_epcs.add(tag.epc)
                    rssi_map[tag.epc] = tag.rssi

            if self.verbose:
                print(f"  Round {i + 1}: {len(tags)} tags, total unique: {len(all_epcs)}")

            if i < scan_rounds - 1:
                time.sleep(scan_interval)

        # Fallback: if no tags found, try single-tag inventory
        if len(all_epcs) == 0 and fallback_single:
            if self.verbose:
                print("  No tags from 0x01, trying 0x0F single-tag fallback")
            tags, status, err = self.inventory_single()
            if not err:
                for tag in tags:
                    if tag.epc not in all_epcs:
                        all_epcs.add(tag.epc)
                        rssi_map[tag.epc] = tag.rssi

        result["epcs"] = sorted(all_epcs)
        result["rssi_map"] = rssi_map

        if len(all_epcs) > 0:
            result["status"] = "ok"
            result["source"] = "answer_mode_serial"
        else:
            result["status"] = "no_tag"
            result["source"] = "answer_mode_serial_no_tag"

        return result

    def inventory_cell_from_params(
        self,
        q_value: int = 2,
        session: int = 0,
        scan_rounds: int = 5,
        scan_interval: float = 0.5,
        fallback_single: bool = True,
        collect_follow_frames: bool = True,
        max_follow_frames: int = 10,
    ) -> Dict:
        """Open port, perform cell inventory, and close port.

        This is a convenience method that handles the full lifecycle.

        Returns:
            Dict with inventory results
        """
        err = self.open()
        if err:
            return {
                "epcs": [],
                "rssi_map": {},
                "rounds_completed": 0,
                "status": "serial_error",
                "error": err,
                "reader_mode": "uhf_reader188_answer_serial",
                "source": "answer_mode_serial_failed",
            }

        try:
            return self.inventory_cell(
                scan_rounds=scan_rounds,
                scan_interval=scan_interval,
                q_value=q_value,
                session=session,
                fallback_single=fallback_single,
                collect_follow_frames=collect_follow_frames,
                max_follow_frames=max_follow_frames,
            )
        finally:
            self.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _hex_str(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="UHFReader188 RFID reader test tool (answer mode)",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port path")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x00, help="Reader address")
    parser.add_argument("--q", type=int, default=2, help="Q value (0-8)")
    parser.add_argument("--session", type=int, default=0, help="Session (0-3)")
    parser.add_argument("--rounds", type=int, default=5, help="Number of inventory rounds per cell")
    parser.add_argument("--interval", type=float, default=0.5, help="Interval between rounds (sec)")
    parser.add_argument("--timeout", type=float, default=2.5, help="Frame read timeout (sec)")
    parser.add_argument("--info", action="store_true", help="Query and display reader info")
    parser.add_argument("--inventory", action="store_true", help="Run multi-round cell inventory")
    parser.add_argument("--single", action="store_true", help="Run single-tag inventory (0x0F)")
    parser.add_argument("--verbose", action="store_true", help="Print hex frames")
    args = parser.parse_args()

    if serial is None:
        print("ERROR: pyserial is not installed. Run: pip install pyserial")
        return 1

    reader = UHFReader188(
        port=args.port,
        baudrate=args.baud,
        address=args.addr,
        timeout_sec=args.timeout,
        verbose=args.verbose,
    )

    err = reader.open()
    if err:
        print(f"ERROR: {err}")
        return 1

    try:
        if args.info:
            info, info_err = reader.read_reader_info()
            if info_err:
                print(f"ERROR reading reader info: {info_err}")
                return 1
            print(f"Reader info: {info}")
            if args.verbose and info.raw_data:
                print(f"  Raw data: {_hex_str(info.raw_data)}")
            return 0

        if args.single:
            tags, status, inv_err = reader.inventory_single()
            if inv_err:
                print(f"ERROR: {inv_err}")
                return 1
            print(f"Single-tag inventory (0x0F): status=0x{status:02X} {STATUS_DESCRIPTIONS.get(status, '?')}")
            if tags:
                for tag in tags:
                    print(f"  EPC={tag.epc}  RSSI={tag.rssi}  Ant={tag.antenna}")
            else:
                print("  No tags found")
            return 0

        if args.inventory:
            result = reader.inventory_cell(
                scan_rounds=args.rounds,
                scan_interval=args.interval,
                q_value=args.q,
                session=args.session,
                fallback_single=True,
            )
            print(f"\nInventory result:")
            print(f"  Status: {result['status']}")
            print(f"  Source: {result['source']}")
            print(f"  Rounds completed: {result['rounds_completed']}")
            print(f"  Unique EPCs: {len(result['epcs'])}")
            for epc in result["epcs"]:
                rssi = result["rssi_map"].get(epc, 0)
                print(f"    EPC={epc}  RSSI={rssi}")
            if result["error"]:
                print(f"  Error: {result['error']}")
            return 0

        # Default: show info then a quick inventory
        print("=== Reader Info ===")
        info, info_err = reader.read_reader_info()
        if info_err:
            print(f"  ERROR: {info_err}")
        else:
            print(f"  {info}")

        print("\n=== Single Inventory (0x0F) ===")
        tags, status, inv_err = reader.inventory_single()
        if inv_err:
            print(f"  ERROR: {inv_err}")
        else:
            print(f"  Status: 0x{status:02X} {STATUS_DESCRIPTIONS.get(status, '?')}")
            for tag in tags:
                print(f"  EPC={tag.epc}  RSSI={tag.rssi}")

        print("\n=== Multi-tag Inventory (0x01 short, Q=2, S0) ===")
        tags, status, inv_err = reader.inventory_once(q_value=2, session=0)
        if inv_err:
            print(f"  ERROR: {inv_err}")
        else:
            print(f"  Status: 0x{status:02X} {STATUS_DESCRIPTIONS.get(status, '?')}")
            for tag in tags:
                print(f"  EPC={tag.epc}  RSSI={tag.rssi}")
            if not tags:
                print("  No tags found")

        return 0

    finally:
        reader.close()


if __name__ == "__main__":
    sys.exit(main())