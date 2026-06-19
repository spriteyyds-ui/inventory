# Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
#!/usr/bin/env python3
"""Standalone test script for UHFReader188 RFID reader in answer mode.

Usage examples:

# Query reader info:
python3 scripts/test_uhf_reader188_answer.py \
  --port /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTA4b2A7N11-if00-port0 \
  --baud 57600 --info --verbose

# Set power to 15 and verify with reader info:
python3 scripts/test_uhf_reader188_answer.py \
  --port /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTA4b2A7N11-if00-port0 \
  --baud 57600 --set-power 15 --info --verbose

# Multi-round cell inventory:
python3 scripts/test_uhf_reader188_answer.py \
  --port /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTA4b2A7N11-if00-port0 \
  --baud 57600 --inventory --q 2 --session 0 --rounds 30 --interval 0.5 --verbose

# Single-tag inventory (0x0F):
python3 scripts/test_uhf_reader188_answer.py \
  --port /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTA4b2A7N11-if00-port0 \
  --baud 57600 --single --verbose

# Run self-check only (no hardware needed):
python3 scripts/test_uhf_reader188_answer.py --selfcheck
"""

from __future__ import annotations

import argparse
import sys
import time

# Allow running from project root or scripts directory
try:
    from uhf_reader188_client import UHFReader188, STATUS_DESCRIPTIONS, parse_reader_info
except ImportError:
    try:
        import os
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from uhf_reader188_client import UHFReader188, STATUS_DESCRIPTIONS, parse_reader_info  # type: ignore
    except ImportError as exc:
        print(f"ERROR: Cannot import uhf_reader188_client: {exc}")
        print("Make sure scripts/uhf_reader188_client.py exists and pyserial is installed.")
        sys.exit(1)


# ---------------------------------------------------------------------------
# Self-check (no hardware needed)
# ---------------------------------------------------------------------------

def run_self_check() -> bool:
    """Run built-in self-check using known raw data from hardware test.

    Raw data (after Status byte): 06 46 0D 02 31 80 21 0A 00 00
    Expected parsed values:
        Version  = 0646
        Type     = 0x0D (UHFReader188)
        Tr_Type  = 0x02 (EPC C1G2 / ISO18000-6C)
        dmaxfre  = 0x31
        dminfre  = 0x80
        Power    = 0x21 = 33
        Scntm    = 0x0A = 10 (1000ms)
    """
    raw = bytes([
        0x06, 0x46, 0x0D, 0x02, 0x31, 0x80, 0x21, 0x0A, 0x00, 0x00
    ])
    info = parse_reader_info(raw)

    print("=" * 60)
    print("Self-check: parse_reader_info() with known raw data")
    print("=" * 60)
    print(f"  Raw hex:  {' '.join(f'{b:02X}' for b in raw)}")
    print(f"  Version:  {info.version}")
    print(f"  Type:     0x{info.reader_type:02X} ({info.reader_type_name})")
    print(f"  Tr_Type:  0x{info.tr_type:02X}")
    print(f"  dmaxfre:  0x{info.dmaxfre:02X}")
    print(f"  dminfre:  0x{info.dminfre:02X}")
    print(f"  Power:    {info.power} (0x{info.power:02X})")
    print(f"  Scntm:    {info.scan_time} ({info.scan_time * 100} ms)")
    print()

    errors = []
    if info.version != "0646":
        errors.append(f"version expected='0646' got='{info.version}'")
    if info.reader_type != 0x0D:
        errors.append(f"reader_type expected=0x0D got=0x{info.reader_type:02X}")
    if info.reader_type_name != "UHFReader188":
        errors.append(
            f"reader_type_name expected='UHFReader188' got='{info.reader_type_name}'")
    if info.tr_type != 0x02:
        errors.append(f"tr_type expected=0x02 got=0x{info.tr_type:02X}")
    if info.dmaxfre != 0x31:
        errors.append(f"dmaxfre expected=0x31 got=0x{info.dmaxfre:02X}")
    if info.dminfre != 0x80:
        errors.append(f"dminfre expected=0x80 got=0x{info.dminfre:02X}")
    if info.power != 0x21:
        errors.append(f"power expected=0x21 got=0x{info.power:02X}")
    if info.scan_time != 0x0A:
        errors.append(f"scan_time expected=0x0A got=0x{info.scan_time:02X}")

    if errors:
        print("FAIL:")
        for e in errors:
            print(f"  {e}")
        return False

    print("PASS: All fields match expected values.")
    return True


# ---------------------------------------------------------------------------
# Hardware test functions
# ---------------------------------------------------------------------------

def test_reader_info(reader: UHFReader188, verbose: bool) -> int:
    """Query and display reader information."""
    print("=" * 60)
    print("UHFReader188 Reader Info Query")
    print("=" * 60)
    print(f"Port: {reader.port}")
    print(f"Baud: {reader.baudrate}")
    print(f"Address: 0x{reader.address:02X}")
    print()

    err = reader.open()
    if err:
        print(f"ERROR opening port: {err}")
        return 1

    try:
        info, info_err = reader.read_reader_info()
        if info_err:
            print(f"ERROR: {info_err}")
            return 1

        print(f"Reader Type: {info.reader_type_name} (0x{info.reader_type:02X})")
        print(f"Tr_Type: 0x{info.tr_type:02X} ", end="")
        if info.tr_type & 0x02:
            print("(EPC C1G2 / ISO18000-6C)", end="")
        print()
        print(f"dmaxfre: 0x{info.dmaxfre:02X}")
        print(f"dminfre: 0x{info.dminfre:02X}")
        print(f"Power: {info.power} (0x{info.power:02X})")
        print(f"Scan Time: {info.scan_time} ({info.scan_time * 100} ms)")
        print(f"Version: {info.version}")

        if verbose and info.raw_data:
            hex_str = " ".join(f"{b:02X}" for b in info.raw_data)
            print(f"\nRaw data: {hex_str}")

        return 0
    finally:
        reader.close()


def test_set_power(reader: UHFReader188, power: int, verbose: bool) -> int:
    """Set reader transmit power and optionally verify with reader info."""
    print("=" * 60)
    print("UHFReader188 Set Power")
    print("=" * 60)
    print(f"Port: {reader.port}")
    print(f"Baud: {reader.baudrate}")
    print(f"Address: 0x{reader.address:02X}")
    print(f"Target power: {power}")
    print()

    err = reader.open()
    if err:
        print(f"ERROR opening port: {err}")
        return 1

    try:
        ok, set_err = reader.set_power(power)
        if set_err:
            print(f"ERROR setting power: {set_err}")
            return 1

        print(f"Set power OK: {power}")

        # Verify by reading reader info
        info, info_err = reader.read_reader_info()
        if info_err:
            print(f"WARNING: Could not verify power via reader info: {info_err}")
        else:
            print(f"Verified Power: {info.power}")
            if info.power != power:
                print(f"WARNING: Expected {power}, got {info.power}")
            else:
                print("Power verified OK.")

        if verbose:
            if info:
                print(f"\nFull reader info: {info}")

        return 0
    finally:
        reader.close()


def test_single_inventory(reader: UHFReader188, verbose: bool) -> int:
    """Run single-tag inventory (0x0F)."""
    print("=" * 60)
    print("UHFReader188 Single-Tag Inventory (0x0F)")
    print("=" * 60)
    print(f"Port: {reader.port}")
    print(f"Baud: {reader.baudrate}")
    print(f"Address: 0x{reader.address:02X}")
    print()

    err = reader.open()
    if err:
        print(f"ERROR opening port: {err}")
        return 1

    try:
        tags, status, inv_err = reader.inventory_single()
        if inv_err:
            print(f"ERROR: {inv_err}")
            return 1

        status_desc = STATUS_DESCRIPTIONS.get(status, "未知")
        print(f"Status: 0x{status:02X} ({status_desc})")

        if tags:
            print(f"Tags found: {len(tags)}")
            for i, tag in enumerate(tags, 1):
                print(f"  [{i}] EPC={tag.epc}  RSSI={tag.rssi}  Ant={tag.antenna}")
        else:
            print("No tags found")

        return 0
    finally:
        reader.close()


def test_multi_inventory(
    reader: UHFReader188,
    q_value: int,
    session: int,
    rounds: int,
    interval: float,
    verbose: bool,
) -> int:
    """Run multi-round cell inventory with 0x01 short format."""
    print("=" * 60)
    print("UHFReader188 Multi-Tag Inventory (0x01 Short Format)")
    print("=" * 60)
    print(f"Port: {reader.port}")
    print(f"Baud: {reader.baudrate}")
    print(f"Address: 0x{reader.address:02X}")
    print(f"Q Value: {q_value}")
    print(f"Session: S{session}")
    print(f"Rounds: {rounds}")
    print(f"Interval: {interval:.2f} sec")
    print()

    err = reader.open()
    if err:
        print(f"ERROR opening port: {err}")
        return 1

    try:
        print("Starting inventory rounds...\n")

        all_epcs: set = set()
        rssi_map = {}
        total_errors = 0
        start_time = time.monotonic()

        for i in range(rounds):
            round_start = time.monotonic()
            tags, status, inv_err = reader.inventory_once(
                q_value=q_value,
                session=session,
                collect_follow_frames=True,
                max_follow_frames=10,
            )
            round_elapsed = time.monotonic() - round_start

            new_count = 0
            for tag in tags:
                if tag.epc not in all_epcs:
                    all_epcs.add(tag.epc)
                    rssi_map[tag.epc] = tag.rssi
                    new_count += 1

            if inv_err:
                total_errors += 1
                if verbose:
                    print(
                        f"  Round {i + 1:3d}/{rounds}: ERROR status=0x{status:02X} "
                        f"err={inv_err} time={round_elapsed:.3f}s"
                    )
            else:
                marker = " *" if new_count > 0 else ""
                if verbose or new_count > 0:
                    print(
                        f"  Round {i + 1:3d}/{rounds}: status=0x{status:02X} "
                        f"tags={len(tags)} new={new_count} unique={len(all_epcs)} "
                        f"time={round_elapsed:.3f}s{marker}"
                    )

            if i < rounds - 1:
                remaining = interval - (time.monotonic() - round_start)
                if remaining > 0:
                    time.sleep(remaining)

        total_elapsed = time.monotonic() - start_time
        print(f"\n{'=' * 60}")
        print(f"Inventory Complete")
        print(f"{'=' * 60}")
        print(f"Total rounds: {rounds}")
        print(f"Total errors: {total_errors}")
        print(f"Total time: {total_elapsed:.2f} sec")
        print(f"Unique EPCs: {len(all_epcs)}")

        if all_epcs:
            print(f"\nTag List:")
            for epc in sorted(all_epcs):
                rssi = rssi_map.get(epc, 0)
                print(f"  EPC={epc}  RSSI={rssi}")
        else:
            print("\nNo tags detected in any round")

        return 0
    finally:
        reader.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="UHFReader188 RFID reader test tool (answer mode)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run self-check only (no hardware needed)
  %(prog)s --selfcheck

  # Query reader info
  %(prog)s --port /dev/ttyUSB0 --baud 57600 --info --verbose

  # Set power to 15 and verify
  %(prog)s --port /dev/ttyUSB0 --baud 57600 --set-power 15 --info --verbose

  # Multi-round inventory with 30 rounds
  %(prog)s --port /dev/ttyUSB0 --baud 57600 --inventory --q 2 --session 0 --rounds 30 --interval 0.5 --verbose

  # Single-tag inventory
  %(prog)s --port /dev/ttyUSB0 --baud 57600 --single --verbose

  # Quick test (info + single + one multi-tag)
  %(prog)s --port /dev/ttyUSB0 --baud 57600 --verbose
""",
    )
    parser.add_argument(
        "--selfcheck",
        action="store_true",
        help="Run built-in self-check (no hardware needed)",
    )
    parser.add_argument(
        "--port",
        default=(
            "/dev/serial/by-id/"
            "usb-Prolific_Technology_Inc._USB-Serial_Controller_CTA4b2A7N11-if00-port0"
        ),
        help="Serial port device path (default: %(default)s)",
    )
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate (default: 57600)")
    parser.add_argument(
        "--addr",
        type=lambda x: int(x, 0),
        default=0x00,
        help="Reader address (default: 0x00)",
    )
    parser.add_argument("--q", type=int, default=2, help="Q value 0-8 (default: 2)")
    parser.add_argument("--session", type=int, default=0, help="Session 0-3 (default: 0)")
    parser.add_argument(
        "--rounds",
        type=int,
        default=5,
        help="Number of inventory rounds (default: 5)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Interval between rounds in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.5,
        help="Frame read timeout in seconds (default: 2.5)",
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Query and display reader info",
    )
    parser.add_argument(
        "--inventory",
        action="store_true",
        help="Run multi-round cell inventory",
    )
    parser.add_argument(
        "--single",
        action="store_true",
        help="Run single-tag inventory (0x0F)",
    )
    parser.add_argument(
        "--set-power",
        type=int,
        default=None,
        metavar="POWER",
        help="Set reader transmit power (0-33)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print hex frames and detailed round info",
    )
    args = parser.parse_args()

    # Self-check mode (no hardware)
    if args.selfcheck:
        ok = run_self_check()
        return 0 if ok else 1

    # Always run self-check before hardware tests
    check_ok = run_self_check()
    print()
    if not check_ok:
        print("WARNING: Self-check FAILED. Parser may produce incorrect results.")
        return 1

    print("Self-check passed.\n")

    # Validate parameters
    if args.q < 0 or args.q > 8:
        print("ERROR: Q value must be 0-8")
        return 1
    if args.session < 0 or args.session > 3:
        print("ERROR: Session must be 0-3")
        return 1
    if args.rounds < 1:
        print("ERROR: Rounds must be >= 1")
        return 1

    reader = UHFReader188(
        port=args.port,
        baudrate=args.baud,
        address=args.addr,
        timeout_sec=args.timeout,
        verbose=args.verbose,
    )

    # Determine what to run
    run_info = args.info
    run_single = args.single
    run_inventory = args.inventory
    run_set_power = args.set_power is not None

    # If no specific action requested, run quick test (info + single + one inventory)
    if not run_info and not run_single and not run_inventory and not run_set_power:
        run_info = True
        run_single = True
        run_inventory = True
        print("No specific action requested, running quick test...\n")

    exit_code = 0

    if run_set_power:
        code = test_set_power(reader, args.set_power, args.verbose)
        if code != 0:
            exit_code = code
        print()

    if run_info:
        code = test_reader_info(reader, args.verbose)
        if code != 0:
            exit_code = code
        print()

    if run_single:
        code = test_single_inventory(reader, args.verbose)
        if code != 0:
            exit_code = code
        print()

    if run_inventory:
        code = test_multi_inventory(
            reader,
            q_value=args.q,
            session=args.session,
            rounds=args.rounds,
            interval=args.interval,
            verbose=args.verbose,
        )
        if code != 0:
            exit_code = code

    return exit_code


if __name__ == "__main__":
    sys.exit(main())