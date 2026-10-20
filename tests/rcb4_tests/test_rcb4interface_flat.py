import unittest

import numpy as np

from rcb4.rcb4interface import blink_startup_program
from rcb4.rcb4interface import ConfigData
from rcb4.rcb4interface import ics_unassigned
from rcb4.rcb4interface import RamAddr
from rcb4.rcb4interface import RCB4Interface
from rcb4.rcb4interface import rom_timer_wait
from rcb4.rcb4interface import ServoOnOffValues
from rcb4.rcb4interface import startup_timer_wait_bytes


class FakeRamRCB4Interface(RCB4Interface):
    """RCB4Interface whose RAM reads and writes go to a bytearray."""

    def __init__(self, config):
        super().__init__()
        self.ram = bytearray(0x490)
        self.ram[0:2] = config.to_bytes(2, "little")
        self.writes = []

    def write_move_ram_to_com_command(self, src_addr, src_data_size):
        return list(self.ram[src_addr:src_addr + src_data_size])

    def write_move_com_to_ram_command(self, dest_addr, dest_data):
        self.writes.append((dest_addr, list(dest_data)))
        self.ram[dest_addr:dest_addr + len(dest_data)] = bytes(dest_data)
        return [0x06]

    def word(self, address, signed=False):
        return int.from_bytes(self.ram[address:address + 2], "little", signed=signed)


def record(ics_number):
    return RamAddr.ServoRamAddress.value + 20 * ics_number


class TestFlattenBoard(unittest.TestCase):

    def test_project_settings_are_undone(self):
        # ROM program, vector jumps, interpolation-end message, 25 ms cycle.
        interface = FakeRamRCB4Interface(0xC2BF)
        interface.ram[record(5) + 2:record(5) + 4] = (-2700).to_bytes(2, "little", signed=True)
        interface.ram[record(5) + 6:record(5) + 9] = bytes([0x4C, 0x1D, 3])  # held, moving
        interface.ram[record(7) + 17:record(7) + 20] = bytes([0x22, 0x40, 0x05])

        changed = interface.flatten_board()

        config = interface.word(RamAddr.ConfigRamAddress.value)
        self.assertEqual(config, 0xC291)
        self.assertEqual(changed["config"], (0xC2BF, 0xC291))
        self.assertEqual(interface.word(record(5) + 2, signed=True), 0)
        self.assertEqual(interface.word(record(5) + 6), ServoOnOffValues.OFF.value)
        self.assertEqual(interface.ram[record(5) + 8], 0)
        self.assertEqual(interface.ram[record(7) + 19], 0)
        self.assertEqual(changed["trim"], {5: -2700})
        self.assertEqual(changed["mixing"], {7: [(0x4022, 0x05)]})
        self.assertFalse(config & ConfigData.EnableRunEeprom.value)

    def test_a_flat_board_is_not_written(self):
        interface = FakeRamRCB4Interface(0xC291)
        for ics_number in range(35):
            # What every record of a board with a project read: no source, ratio 1.
            for offset in (14, 17):
                interface.ram[record(ics_number) + offset:record(ics_number) + offset + 3] = \
                    bytes([0xFF, 0xFF, 0x01])

        changed = interface.flatten_board()

        self.assertEqual(interface.writes, [])
        self.assertEqual(changed, {"config": None, "trim": {}, "mixing": {}})

    def test_not_flat_leaves_a_project_assignment_alone(self):
        interface = FakeRamRCB4Interface(0xC291)
        table = np.full(35, ics_unassigned, dtype="<u2")
        table[3] = record(3)
        interface.write_ics_assign_table(table)
        interface.writes = []

        self.assertEqual(len(interface.assign_ics_servos(flat=False)), 0)
        self.assertEqual(interface.writes, [])


class FakeRomRCB4Interface(FakeRamRCB4Interface):
    """Adds a ROM, read and written through MOV packets on serial_write."""

    def __init__(self, config, rom):
        super().__init__(config)
        self.rom = bytearray(rom)
        self.rom_writes = []

    def serial_write(self, byte_list, timeout=10):
        kind = byte_list[2]
        if kind == 0x23:  # ROM -> COM
            address = byte_list[6] | byte_list[7] << 8 | byte_list[8] << 16
            return [0x00] + list(self.rom[address:address + byte_list[9]])
        if kind == 0x32:  # literal -> ROM
            address = byte_list[3] | byte_list[4] << 8 | byte_list[5] << 16
            data = bytes(byte_list[6:-1])
            self.rom_writes.append((address, data))
            self.rom[address:address + len(data)] = data
            return [0x00, 0x06]
        raise AssertionError(f"unexpected packet {bytes(byte_list).hex()}")


def project_rom():
    """The project area of the board measured on 2026-10-20 (HeartToHeart4)."""
    rom = bytearray(0x094B)
    rom[0x0333:0x0333 + 11] = bytes.fromhex("0b0003000000dc050002f1")
    rom[0x033E:0x033E + 3] = bytes.fromhex("4f10ff")  # ServoCmdFree
    rom[0x038D:0x038D + 25] = startup_timer_wait_bytes
    rom[0x05DC:0x05DE] = (0x001B).to_bytes(2, "little")
    rom[0x05DE:0x05DE + 72] = b"\xff" * 72
    for n in range(35):
        record = 0x0626 + 20 * n
        rom[record + 1] = n // 2
        rom[record + 6:record + 8] = (7500).to_bytes(2, "little")
        rom[record + 14:record + 20] = bytes.fromhex("ffff01ffff01")
    return rom


class TestRom(unittest.TestCase):

    def test_flatten_rom_startup_writes_config_and_free_references(self):
        rom = project_rom()
        rom[0x0626 + 20 * 5 + 2:0x0626 + 20 * 5 + 4] = (-2700).to_bytes(2, "little", signed=True)
        interface = FakeRomRCB4Interface(0x8393, rom)

        changed = interface.flatten_rom_startup()

        self.assertEqual(changed["config"], (0x001B, 0x8011))
        self.assertEqual(int.from_bytes(interface.rom[0x05DC:0x05DE], "little"), 0x8011)
        for n in range(35):
            record = 0x0626 + 20 * n
            self.assertEqual(interface.rom[record + 6:record + 8], b"\x00\x80")
            self.assertEqual(interface.rom[record + 2:record + 4], b"\x00\x00")
            self.assertEqual(interface.rom[record + 1], n // 2)  # ICS id kept
        # The startup program itself is not touched.
        self.assertEqual(interface.rom[0x0333:0x0333 + 11], project_rom()[0x0333:0x0333 + 11])
        # A second run writes nothing.
        interface.rom_writes = []
        changed = interface.flatten_rom_startup()
        self.assertEqual(changed, {"config": None, "servo_bytes": 0, "program_bytes": 0})
        self.assertEqual(interface.rom_writes, [])

    def test_an_unknown_rom_is_not_written(self):
        rom = project_rom()
        rom[0x0333] = 0x09
        interface = FakeRomRCB4Interface(0x8393, rom)

        with self.assertRaises(RuntimeError):
            interface.flatten_rom_startup()
        self.assertEqual(interface.rom_writes, [])

    def test_backup_and_restore(self):
        import os
        import tempfile

        interface = FakeRomRCB4Interface(0x8393, project_rom())
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "rom.bin")
            saved = interface.backup_rom(path, size=len(interface.rom))
            interface.flatten_rom_startup()
            self.assertNotEqual(bytes(interface.rom), saved)
            interface.restore_rom(path)
        self.assertEqual(bytes(interface.rom), saved)

    def test_led_can_be_left_off(self):
        interface = FakeRomRCB4Interface(0x8393, project_rom())

        changed = interface.flatten_rom_startup(led=False)

        self.assertEqual(changed["config"], (0x001B, 0x0011))


class TestBlinkProgram(unittest.TestCase):

    def test_timer_wait_is_the_startup_routines_own(self):
        # The 500 ms wait a HeartToHeart4 project holds at 0x038D.
        self.assertEqual(rom_timer_wait(0x038D, 5), startup_timer_wait_bytes)

    def test_three_blinks_fit_and_four_do_not(self):
        self.assertEqual(len(blink_startup_program(0x4083, 0xC081, 3, 3)), 3 * 68 + 9 + 7)
        with self.assertRaises(ValueError):
            blink_startup_program(0x4083, 0xC081, 4, 3)

    def test_flatten_with_blinks(self):
        interface = FakeRomRCB4Interface(0x8393, project_rom())

        changed = interface.flatten_rom_startup(blinks=3)

        # The ROM program runs through the config write; the program at
        # 0x033E stops it after blinking, LED left on.
        self.assertEqual(changed["config"], (0x001B, 0x0013))
        program = blink_startup_program(0x0013, 0x8011, 3, 3)
        self.assertEqual(bytes(interface.rom[0x033E:0x033E + len(program)]), program)
        # Only the bytes that differed are written.
        self.assertTrue(0 < changed["program_bytes"] <= len(program))
        self.assertEqual(interface.rom_startup_blinks(), 3)
        # Back to no blinks: the config stops the program at the config write.
        changed = interface.flatten_rom_startup()
        self.assertEqual(changed["config"], (0x0013, 0x8011))

    def test_hth4_code_is_not_a_blink_program(self):
        interface = FakeRomRCB4Interface(0x8393, project_rom())
        self.assertIsNone(interface.rom_startup_blinks())
