#!/usr/bin/env python

import argparse
from pathlib import Path
import shutil
import subprocess
import sys

from rcb4.data import kondoh7_elf
from rcb4.data import stlink


def check_dependencies():
    objcopy = shutil.which("arm-none-eabi-objcopy")
    if objcopy is None:
        print("Please install arm-none-eabi-objcopy.")
        print("sudo apt install -y binutils-arm-none-eabi")
        sys.exit(1)


def convert_elf_to_bin(elf_path, bin_path):
    cmd = ["arm-none-eabi-objcopy", "-O", "binary", elf_path, bin_path]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error converting ELF to BIN: {result.stderr}")
        sys.exit(2)


def flash_bin_to_device(st_flash_path, bin_path):
    cmd = [st_flash_path, "write", bin_path, "0x08000000"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error flashing BIN to device: {result.stderr}")
        sys.exit(3)


def main():
    parser = argparse.ArgumentParser(description="Flash firmware to a device.")
    parser.add_argument(
        "--firmware",
        type=str,
        help="Path to the firmware ELF file. If not provided, the latest kondoh7 ELF file will be used.",
        default=None
    )
    args = parser.parse_args()

    print("Checking dependencies...")
    check_dependencies()
    print("Dependencies are satisfied.")

    print("Locating ST-Link path...")
    st_flash_path = stlink()
    if not st_flash_path:
        print("Error: ST-Link path not found. Please check your installation of stlink.")
        sys.exit(4)
    print(f"ST-Link path found: {st_flash_path}")

    print("Retrieving ELF file...")
    elf_path = args.firmware if args.firmware else kondoh7_elf('latest')
    if not elf_path or not Path(elf_path).is_file():
        print(f"Error: ELF file not found: {elf_path}")
        print("Please check the path or filename.")
        sys.exit(5)
    print(f"ELF file found: {elf_path}")

    bin_path = Path(elf_path).with_suffix(".bin")
    print(f"Converting ELF to BIN: {bin_path}")

    convert_elf_to_bin(elf_path, bin_path)
    print("ELF successfully converted to BIN.")

    print("Flashing BIN to device...")
    flash_bin_to_device(st_flash_path, bin_path)
    print("BIN successfully flashed to device.")


if __name__ == "__main__":
    main()
