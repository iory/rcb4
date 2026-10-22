#!/usr/bin/env python
"""Read, back up, restore and flatten the ROM of an RCB-4 / RCB-4mini board.

    rcb4-rom show
    rcb4-rom backup rom.bin
    rcb4-rom flat-startup            # backs the ROM up first, then writes
    rcb4-rom restore rom.bin

``flat-startup`` makes the board power up without commanding any servo:
the HeartToHeart4 startup routine stops right after it writes the config
word, before it holds the servos, moves them home or plays the startup
motion, and every servo record starts free with no trim and no mixing.
Software then sees the same board whatever project is in its ROM.
"""

import argparse
from datetime import datetime
import sys

import serial

from rcb4.rcb4interface import com_baudrates
from rcb4.rcb4interface import ConfigData
from rcb4.rcb4interface import ics_slot_count
from rcb4.rcb4interface import ics_unassigned
from rcb4.rcb4interface import mixing_no_source
from rcb4.rcb4interface import RCB4Interface
from rcb4.rcb4interface import RomAddr
from rcb4.rcb4interface import servo_mixing_offsets
from rcb4.rcb4interface import servo_record_size
from rcb4.rcb4interface import ServoOnOffValues


def open_board(port):
    """Connect without touching any servo (no flattening, no assignment)."""
    interface = RCB4Interface()
    if port is None:
        if interface.auto_open(servos=False):
            return interface
    else:
        for baudrate in com_baudrates:
            try:
                if interface.open(port=port, baudrate=baudrate, servos=False):
                    return interface
            except serial.SerialException:
                pass
            interface.close()
    print("No RCB-4 board answered.", file=sys.stderr)
    sys.exit(1)


def show(interface):
    config = int.from_bytes(
        interface.read_rom(RomAddr.ConfigDataRomAddress.value, 2), "little")
    flags = {
        "ROM program": ConfigData.EnableRunEeprom.value,
        "vector jumps": ConfigData.EnableReferenceTable.value,
        "interpolation-end message": ConfigData.EnableServoResponse.value,
        "ICS switch": ConfigData.EnableSio.value,
        "green LED": ConfigData.GreenLED.value,
    }
    print(f"startup config {config:#06x}: " + ", ".join(
        f"{name} {'on' if config & bit else 'off'}" for name, bit in flags.items()))
    table = interface.read_rom(RomAddr.ServoDataRamAddress.value, 2 * ics_slot_count)
    assigned = [n for n in range(ics_slot_count)
                if int.from_bytes(table[2 * n:2 * n + 2], "little") != ics_unassigned]
    print(f"ICS numbers assigned at startup: {assigned if assigned else 'none'}")
    data = interface.read_rom(RomAddr.ServoDataRomAddress.value,
                              servo_record_size * ics_slot_count)
    not_flat = []
    for n in range(ics_slot_count):
        record = data[servo_record_size * n:servo_record_size * (n + 1)]
        trim = int.from_bytes(record[2:4], "little", signed=True)
        reference = int.from_bytes(record[6:8], "little")
        mixings = [(int.from_bytes(record[o:o + 2], "little"), record[o + 2])
                   for o in servo_mixing_offsets]
        mixings = [(s, r) for s, r in mixings if s != mixing_no_source and r != 0]
        if trim or reference != ServoOnOffValues.OFF.value or mixings:
            not_flat.append(n)
            print(f"  ICS {n:2d}: trim {trim:+d}, startup reference "
                  + ("free" if reference == ServoOnOffValues.OFF.value else str(reference))
                  + (f", mixing {[(hex(s), r) for s, r in mixings]}" if mixings else ""))
    blinks = interface.rom_startup_blinks()
    if config & ConfigData.EnableRunEeprom.value and blinks is not None:
        print(f"startup blinks the green LED {blinks} times, then stops the ROM program")
    stops = not config & ConfigData.EnableRunEeprom.value or blinks is not None
    flat = (stops and not config & (ConfigData.EnableReferenceTable.value
                                    | ConfigData.EnableServoResponse.value)
            and not not_flat)
    print("startup is flat" if flat else
          "startup is NOT flat (rcb4-rom flat-startup would change it)")


def main():
    parser = argparse.ArgumentParser(
        description="Read, back up, restore and flatten the ROM of an RCB-4 board.")
    parser.add_argument("--port", default=None,
                        help="Serial port or ftdi:// URL (default: the first Dual USB Adapter)")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("show", help="Print what the board does at power-up")
    backup = sub.add_parser("backup", help="Save the whole ROM to a file")
    backup.add_argument("path")
    restore = sub.add_parser("restore", help="Write a backup back into the ROM")
    restore.add_argument("path")
    flat = sub.add_parser(
        "flat-startup",
        help="Make the board power up without commanding any servo (writes ROM)")
    flat.add_argument(
        "--no-led", action="store_true",
        help="Leave the green LED off at power-up (default: on)")
    flat.add_argument(
        "--blink", type=int, default=0, metavar="N",
        help="Blink the green LED N times at power-up (1-3), so a flat board "
        + "can be told by eye")
    flat.add_argument(
        "--backup", default=None,
        help="Where to save the ROM first (default: rcb4-rom-<date>.bin here)")
    args = parser.parse_args()

    interface = open_board(args.port)
    try:
        if args.command == "show":
            show(interface)
        elif args.command == "backup":
            data = interface.backup_rom(args.path)
            print(f"Saved {len(data)} bytes of ROM to {args.path}")
        elif args.command == "restore":
            written = interface.restore_rom(args.path)
            print(f"Restored {args.path}: {written} bytes differed and were written")
        elif args.command == "flat-startup":
            path = args.backup or datetime.now().strftime("rcb4-rom-%Y%m%d-%H%M%S.bin")
            data = interface.backup_rom(path)
            print(f"Saved {len(data)} bytes of ROM to {path} "
                  + f"(undo with: rcb4-rom restore {path})")
            changed = interface.flatten_rom_startup(led=not args.no_led, blinks=args.blink)
            if changed["config"] is not None:
                old, new = changed["config"]
                print(f"Startup config {old:#06x} -> {new:#06x}")
            print(f"Servo data: {changed['servo_bytes']} bytes written")
            if args.blink:
                print(f"Blink program: {changed['program_bytes']} bytes written")
            if (changed["config"] is None and changed["servo_bytes"] == 0
                    and changed["program_bytes"] == 0):
                print("The ROM already starts up flat; nothing written")
            else:
                print("Takes effect at the next power-up")
    finally:
        interface.close()


if __name__ == "__main__":
    main()
