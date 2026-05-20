#!/usr/bin/env python3
"""Manual Modbus RTU relay test tool for a two-wire lift actuator."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional


FUNC_WRITE_SINGLE_COIL = 0x05
COIL_ON = 0xFF00
COIL_OFF = 0x0000


def calc_crc16(data: bytes) -> int:
    """Return Modbus RTU CRC16 as an integer, low byte transmitted first."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_write_coil_frame(slave: int, coil_addr: int, on: bool) -> bytes:
    if not 1 <= slave <= 247:
        raise ValueError(f"slave address out of range: {slave}")
    if not 0 <= coil_addr <= 0xFFFF:
        raise ValueError(f"coil address out of range: {coil_addr}")

    value = COIL_ON if on else COIL_OFF
    payload = bytes(
        [
            slave,
            FUNC_WRITE_SINGLE_COIL,
            (coil_addr >> 8) & 0xFF,
            coil_addr & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]
    )
    crc = calc_crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


class RelayModbusClient:
    def __init__(
        self,
        port: str,
        baud: int,
        slave: int,
        *,
        dry_run: bool = False,
        verbose: bool = False,
        timeout: float = 1.0,
    ) -> None:
        self.port = port
        self.baud = baud
        self.slave = slave
        self.dry_run = dry_run
        self.verbose = verbose
        self.timeout = timeout
        self._serial = None

    def open(self) -> None:
        if self.dry_run:
            print(f"DRY-RUN: skip opening serial port {self.port}")
            return

        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required. Install it with: python3 -m pip install pyserial"
            ) from exc

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def write_coil(self, channel: int, on: bool) -> None:
        if channel not in (1, 2, 3, 4):
            raise ValueError(f"relay channel must be 1..4, got {channel}")

        coil_addr = channel - 1
        frame = build_write_coil_frame(self.slave, coil_addr, on)
        action = "ON" if on else "OFF"

        if self.dry_run:
            print(f"DRY-RUN: Y{channel} {action}  TX {hex_bytes(frame)}")
            return

        if self._serial is None:
            raise RuntimeError("serial port is not open")

        self._serial.reset_input_buffer()
        if self.verbose:
            print(f"TX: {hex_bytes(frame)}")
        self._serial.write(frame)
        self._serial.flush()

        response = self._serial.read(8)
        if self.verbose:
            print(f"RX: {hex_bytes(response)}")
        self._check_write_coil_response(response, coil_addr, on)

    def set_y1_y2(self, y1_on: bool, y2_on: bool) -> None:
        self.write_coil(1, y1_on)
        self.write_coil(2, y2_on)

    def stop(self) -> None:
        self.write_coil(3, False)

    def all_off(self) -> None:
        self.write_coil(3, False)
        self.write_coil(1, False)
        self.write_coil(2, False)
        self.write_coil(4, False)

    def pulse_direction(self, direction: str, seconds: float, settle: float) -> None:
        direction = direction.lower()
        if direction == "a":
            y1_on = False
            y2_on = False
        elif direction == "b":
            y1_on = True
            y2_on = True
        else:
            raise ValueError("direction must be 'a' or 'b'")

        print(f"Pulse direction {direction.upper()} for {seconds:.3f}s")
        self.stop()
        time.sleep(settle)
        self.set_y1_y2(y1_on, y2_on)
        time.sleep(settle)

        try:
            self.write_coil(3, True)
            time.sleep(seconds)
        finally:
            self.stop()

    def _check_write_coil_response(
        self, response: bytes, expected_addr: int, expected_on: bool
    ) -> None:
        if len(response) != 8:
            raise RuntimeError(
                f"incomplete Modbus response: expected 8 bytes, got {len(response)}"
            )

        payload = response[:-2]
        received_crc = response[-2] | (response[-1] << 8)
        calculated_crc = calc_crc16(payload)
        if received_crc != calculated_crc:
            raise RuntimeError(
                "Modbus CRC mismatch: "
                f"received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}"
            )

        if response[0] != self.slave:
            raise RuntimeError(
                f"unexpected slave address: expected {self.slave}, got {response[0]}"
            )
        if response[1] != FUNC_WRITE_SINGLE_COIL:
            raise RuntimeError(
                f"unexpected function code: expected 0x05, got 0x{response[1]:02X}"
            )

        response_addr = (response[2] << 8) | response[3]
        expected_value = COIL_ON if expected_on else COIL_OFF
        response_value = (response[4] << 8) | response[5]

        if response_addr != expected_addr:
            raise RuntimeError(
                f"unexpected coil address: expected 0x{expected_addr:04X}, "
                f"got 0x{response_addr:04X}"
            )
        if response_value != expected_value:
            raise RuntimeError(
                f"unexpected coil value: expected 0x{expected_value:04X}, "
                f"got 0x{response_value:04X}"
            )


def finite_positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("value must be a positive finite number")
    return number


def finite_nonnegative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError("value must be a non-negative finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manual Modbus RTU test tool for lift relay point movement."
    )
    parser.add_argument("--port", required=True, help="Serial device, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=38400, help="Serial baud rate")
    parser.add_argument("--slave", type=int, default=1, help="Modbus RTU slave address")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print actions and frames without opening the serial port",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print transmitted and received Modbus RTU frames",
    )

    motion_options = argparse.ArgumentParser(add_help=False)
    motion_options.add_argument(
        "--seconds",
        type=finite_positive_float,
        default=0.5,
        help="Pulse run time in seconds",
    )
    motion_options.add_argument(
        "--settle",
        type=finite_nonnegative_float,
        default=0.2,
        help="Delay after stopping and after direction switching",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("all_off", help="Turn off Y1, Y2, Y3, and Y4")
    subparsers.add_parser("stop", help="Turn off Y3 motor power")
    subparsers.add_parser("dir_a", parents=[motion_options], help="Pulse direction A")
    subparsers.add_parser("dir_b", parents=[motion_options], help="Pulse direction B")

    pulse_parser = subparsers.add_parser(
        "pulse", parents=[motion_options], help="Pulse the selected direction"
    )
    pulse_parser.add_argument(
        "--direction",
        choices=("a", "b"),
        required=True,
        help="Pulse direction",
    )

    return parser


def safe_shutdown(client: RelayModbusClient) -> None:
    try:
        client.stop()
    except Exception as exc:
        print(f"WARNING: failed to stop Y3: {exc}", file=sys.stderr)

    try:
        client.all_off()
    except Exception as exc:
        print(f"WARNING: failed to turn all channels off: {exc}", file=sys.stderr)


def run_command(args: argparse.Namespace, client: RelayModbusClient) -> None:
    if args.command == "all_off":
        client.all_off()
    elif args.command == "stop":
        client.stop()
    elif args.command == "dir_a":
        client.pulse_direction("a", args.seconds, args.settle)
    elif args.command == "dir_b":
        client.pulse_direction("b", args.seconds, args.settle)
    elif args.command == "pulse":
        client.pulse_direction(args.direction, args.seconds, args.settle)
    else:
        raise ValueError(f"unknown command: {args.command}")


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    client = RelayModbusClient(
        port=args.port,
        baud=args.baud,
        slave=args.slave,
        dry_run=args.dry_run,
        verbose=args.verbose,
    )

    try:
        client.open()
        run_command(args, client)
        print("OK")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; attempting to stop the lift relay.", file=sys.stderr)
        safe_shutdown(client)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        safe_shutdown(client)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
