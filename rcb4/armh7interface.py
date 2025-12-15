import numbers
from pathlib import Path
import platform
import select
import struct
from threading import Lock
import time

from colorama import Fore
from colorama import Style
from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection
import numpy as np
import serial
import serial.tools.list_ports

from rcb4.asm import encode_servo_ids_to_5bytes_bin
from rcb4.asm import encode_servo_positions_to_bytes
from rcb4.asm import encode_servo_velocity_and_position_to_bytes
from rcb4.asm import four_bit_to_num
from rcb4.asm import rcb4_checksum
from rcb4.asm import rcb4_servo_svector
from rcb4.asm import rcb4_velocity
from rcb4.ctype_utils import c_type_to_numpy_format
from rcb4.ctype_utils import c_type_to_size
from rcb4.data import kondoh7_elf
from rcb4.rcb4interface import CommandTypes
from rcb4.rcb4interface import deg_to_servovector
from rcb4.rcb4interface import rcb4_dof
from rcb4.rcb4interface import RCB4Interface
from rcb4.rcb4interface import ServoOnOffValues
from rcb4.rcb4interface import ServoParams
from rcb4.struct_header import c_vector
from rcb4.struct_header import ControlStruct
from rcb4.struct_header import DataAddress
from rcb4.struct_header import GPIOStruct
from rcb4.struct_header import Madgwick
from rcb4.struct_header import max_sensor_num
from rcb4.struct_header import max_SIO_num
from rcb4.struct_header import sensor_sidx
from rcb4.struct_header import SensorbaseStruct
from rcb4.struct_header import ServoStruct
from rcb4.struct_header import SystemStruct
from rcb4.struct_header import WormmoduleStruct
from rcb4.temperature import get_setting_value_from_temperatures
from rcb4.temperature import setting_value_to_temperature
from rcb4.units import convert_data
from rcb4.usb_utils import reset_serial_port

# ICS channel mapping (corresponds to actual UART mapping from euslisp)
# huart1:J6, huart2:J1, huart3:J5, huart4:J4, huart5:J3, huart7:J2
ICS_CHANNELS = {
    "J1": 1,
    "J2": 5,
    "J3": 4,
    "J4": 3,
    "J5": 2,
    "J6": 0,
    "ALL": 6
}

# ICS baudrate mapping (from euslisp)
ICS_BAUDRATES = {
    115200: 0,
    625000: 1,
    1250000: 2
}

armh7_variable_list = [
    "walking_command",
    "walking_mode",
    "rom_to_flash",
    "flash_to_rom",
    "Mfilter",
    "set_sidata_command",
    "set_sdata_command",
    "set_edata_command",
    "sidata_to_dataram",
    "dataflash_to_dataram",
    "servo_vector",
    "dataram_to_dataflash",
    "Control",
    "_sidata",
    "_sdata",
    "uwTickPrio",
    "__fdlib_version",
    "_impure_ptr",
    "_edata",
    "_sbss",
    "_ebss",
    "servo_idmode_scan",
    "buzzer_init_sound",
    "servo_idmode_scan_single",
    "imu_data_",
    "Sensor_vector",
    "Worm_vector",
    "SysB",
    "data_address",
    "pump_switch",
    "valve_switch",
    "gpio_cmd",
    "GPIO_vector",
]


servo_eeprom_params64 = {
    "fix_header": (1, 2),
    "stretch_gain": (3, 4),
    "speed": (5, 6),
    "punch": (7, 8),
    "dead_band": (9, 10),
    "dumping": (11, 12),
    "safe_timer": (13, 14),
    "mode_flag_b7slave_b4rotation_b3pwm_b1free_b0reverse": (15, 16),
    "pulse_max_limit": (17, 18, 19, 20),
    "pulse_min_limit": (21, 22, 23, 24),
    "fix_dont_change_25": (25, 26),
    "ics_baud_rate_10_115200_00_1250000": (27, 28),
    "temperature_limit": (29, 30),
    "current_limit": (31, 32),
    "fix_dont_change_33": (33, 34),
    "fix_dont_change_35": (35, 36),
    "fix_dont_change_37": (37, 38),
    "fix_dont_change_39": (39, 40),
    "fix_dont_change_41": (41, 42),
    "fix_dont_change_43": (43, 44),
    "fix_dont_change_45": (45, 46),
    "fix_dont_change_47": (47, 48),
    "fix_dont_change_49": (49, 50),
    "response": (51, 52),
    "user_offset": (53, 54),
    "fix_dont_change_55": (55, 56),
    "servo_id": (57, 58),
    "stretch_1": (59, 60),
    "stretch_2": (61, 62),
    "stretch_3": (63, 64),
}


def padding_bytearray(byte_array, n):
    padding_length = n - len(byte_array)
    if padding_length > 0:
        return bytearray(padding_length) + byte_array
    else:
        return byte_array


def is_running_on_wsl():
    """Check if the code is running on Windows Subsystem for Linux (WSL).

    Returns
    -------
    bool
        True if running on WSL, False otherwise.
    """
    try:
        with open('/proc/version', encoding='utf-8') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except (FileNotFoundError, PermissionError):
        return False


class ARMH7Interface:
    def __init__(self, timeout=0.1):
        self.lock = Lock()
        self.serial = None
        self.id_vector = None
        self.servo_sorted_ids = None
        self.worm_sorted_ids = None
        self._armh7_address = None
        self._servo_id_to_worm_id = None
        self._worm_id_to_servo_id = None
        self._joint_to_actuator_matrix = None
        self._actuator_to_joint_matrix = None
        self._worm_ref_angle = None
        self._default_timeout = timeout

    def __del__(self):
        self.close()

    def _handle_usb_permission_error(self):
        """Print USB permission error message with instructions in red color."""
        print(f"\n{Fore.RED}{'=' * 60}")
        print(f"{Fore.RED}USB PERMISSION ERROR")
        print(f"{Fore.RED}{'=' * 60}")
        print(f"{Fore.RED}\nYou don't have permission to access the USB device.")
        print(f"{Fore.RED}\nTo fix this issue, please run the following commands:")
        print(f"{Fore.RED}\n1. Add udev rules:")
        print(f"{Fore.RED}   curl -fsSL https://raw.githubusercontent.com/iory/rcb4/main/rcb4/assets/system/99-rcb4-udev.rules | sudo tee /etc/udev/rules.d/99-rcb4-udev.rules")
        print(f"{Fore.RED}   sudo udevadm control --reload-rules")
        print(f"{Fore.RED}   sudo udevadm trigger")
        print(f"{Fore.RED}\n2. Add your user to the required groups:")
        print(f"{Fore.RED}   sudo usermod -a -G dialout $USER")
        print(f"{Fore.RED}   sudo usermod -a -G plugdev $USER")
        print(f"{Fore.RED}\n3. Log out and log back in for the group changes to take effect.")
        print(f"{Fore.RED}\nFor more details, see the README.md file.")
        print(f"{Fore.RED}{'=' * 60}{Style.RESET_ALL}")

    def open(self, port="/dev/ttyUSB1", baudrate=1000000, timeout=0.01, max_attempts=3):
        """Opens a serial connection to the ARMH7 device.

        Parameters
        ----------
        port : str
            The port name to connect to.
        baudrate : int
            The baud rate for the serial connection.
        timeout : float, optional
            The timeout for the serial connection in seconds (default is 0.01).
        max_attempts : int, optional
            Maximum number of connection attempts (default is 3).

        Returns
        -------
        None

        Raises
        ------
        serial.SerialException
            If there is an error opening the serial port.
        """
        connected = False
        last_exception = None

        for attempt in range(max_attempts):
            try:
                # USB reset only on retry (not on first attempt) and not on WSL
                if attempt > 0 and not is_running_on_wsl():
                    try:
                        print(f"Attempt {attempt + 1}: Performing USB reset...")
                        reset_serial_port(port)
                        time.sleep(2.0)
                    except ValueError as e:
                        print(f"Skipping reset for non-USB serial port: {e}")
                    except Exception as e:
                        if "LIBUSB_ERROR_ACCESS" in str(e):
                            self._handle_usb_permission_error()
                            raise
                        print(f"USB reset failed: {e}")

                # Open serial port
                self.serial = serial.Serial(port, baudrate, timeout=timeout)
                print(f"Attempt {attempt + 1}: Opened {port} at {baudrate} baud")

                # Clear any garbage data from buffers
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                time.sleep(0.1)

                # After powering on, the ACK value becomes unstable for some reason,
                # so the process is repeated several times.
                for ack_attempt in range(10):
                    try:
                        ack = self.check_ack()
                        if ack is True:
                            connected = True
                            print(f"ACK confirmed on attempt {ack_attempt + 1}")
                            break
                        time.sleep(0.1)
                    except Exception as e:
                        if ack_attempt == 9:
                            print(f"ACK check failed after 10 attempts: {e}")
                        time.sleep(0.05)

                if connected:
                    break
                else:
                    print(f"Attempt {attempt + 1}: Failed to receive ACK")
                    self.serial.close()
                    self.serial = None
                    if attempt < max_attempts - 1:
                        time.sleep(1.0)

            except serial.SerialException as e:
                last_exception = e
                print(f"Attempt {attempt + 1}: Serial connection failed - {e}")
                if self.serial:
                    self.serial.close()
                    self.serial = None
                if attempt < max_attempts - 1:
                    time.sleep(1.0)

        if not connected:
            error_msg = f"Failed to connect to {port} after {max_attempts} attempts"
            if last_exception:
                error_msg += f": {last_exception}"
            raise serial.SerialException(error_msg)

        self.check_firmware_version()
        self.copy_worm_params_from_flash()
        self.search_worm_ids()
        # Set a baseline threshold value
        # before sending joint angle commands with ref_angle in the WormModule.
        self.write_cstruct_slot_v(
            WormmoduleStruct, "thleshold", 30 * np.ones(max_sensor_num)
        )
        self.write_cstruct_slot_v(
            WormmoduleStruct, "thleshold_scale", np.ones(max_sensor_num)
        )
        self.search_servo_ids()
        self.all_jointbase_sensors()
        return True

    @staticmethod
    def from_port(port=None):
        ports = serial.tools.list_ports.comports()
        if port is not None:
            for cand in ports:
                if cand.device == port:
                    if cand.vid == 0x165C and cand.pid == 0x0008:
                        interface = RCB4Interface()
                        interface.open(port)
                        return interface
            interface = ARMH7Interface()
            interface.open(port)
            return interface

        interface = ARMH7Interface()
        ret = interface.auto_open()
        if ret is True:
            return interface

        interface = RCB4Interface()
        ret = interface.auto_open()
        if ret is True:
            return interface

    def auto_open(self, max_attempts=3):
        """Automatically find and open the ARMH7 device.

        Parameters
        ----------
        max_attempts : int, optional
            Maximum number of connection attempts per port (default is 3).

        Returns
        -------
        bool
            True if connection successful, False otherwise.
        """
        system_info = platform.uname()
        if "amlogic" in system_info.version and system_info.machine == "aarch64":
            # for radxa specific.
            try:
                return self.open("/dev/ttyAML1", max_attempts=max_attempts)
            except Exception as e:
                print(f"Failed to open /dev/ttyAML1: {e}")
                return False

        vendor_id = 0x0403
        product_id = 0x6001
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vendor_id and port.pid == product_id:
                try:
                    return self.open(port=port.device, max_attempts=max_attempts)
                except Exception as e:
                    print(f"Failed to open {port.device}: {e}")
                    continue
        return False

    def close(self):
        if self.serial:
            self.serial.close()
        self.serial = None

    def is_opened(self):
        return self.serial is not None

    def serial_write(self, byte_list):
        if self.serial is None:
            raise RuntimeError("Serial is not opened.")

        data_to_send = bytes(byte_list)
        with self.lock:
            try:
                self.serial.write(data_to_send)
            except serial.SerialException as e:
                print(f"Error sending data: {e}")
            ret = self.serial_read()
        return ret

    def serial_read(self, timeout=None):
        if self.serial is None:
            raise RuntimeError("Serial is not opened.")
        if timeout is None:
            timeout = self._default_timeout

        start_time = time.time()
        read_data = b""
        while True:
            remain_time = timeout - (time.time() - start_time)
            if remain_time <= 0:
                raise serial.SerialException("Timeout: No data received.")
            ready, _, _ = select.select([self.serial], [], [], remain_time)
            if not ready:
                continue

            chunk = self.serial.read(self.serial.in_waiting or 1)
            if not chunk:
                raise serial.SerialException("Timeout: Incomplete data received.")
            read_data += chunk
            if len(read_data) > 0 and read_data[0] == len(read_data):
                return read_data[1 : len(read_data) - 1]

    def get_version(self):
        byte_list = [0x03, CommandTypes.Version.value, 0x00]
        return self.serial_write(byte_list).decode("utf-8")

    def get_ack(self):
        byte_list = [0x04, CommandTypes.AckCheck.value, 0x06, 0x08]
        return self.serial_write(byte_list)

    def check_ack(self):
        ack_byte_list = self.get_ack()
        if ack_byte_list is not None and len(ack_byte_list) > 1:
            return ack_byte_list[1] == 0x06
        else:
            return False

    def check_firmware_version(self):
        version = self.get_version()
        try:
            kondoh7_elf(version)
            return True
        except RuntimeError as e:
            print(Fore.RED + str(e) + f". Current firmware version is {version}")
            print("Please run following commands.")
            print("<" * 100)
            print("sudo apt install -y binutils-arm-none-eabi")
            from rcb4.data import stlink

            st_flash_path = stlink()
            bin_path = Path(kondoh7_elf()).with_suffix(".bin")
            print(f"arm-none-eabi-objcopy -O binary {kondoh7_elf()} {bin_path}")
            print(f"{st_flash_path} write {bin_path} 0x08000000")
            print(">" * 100 + Style.RESET_ALL)
            self.close()
        raise RuntimeError("The firmware version is inconsistent.")

    def memory_read_aux(self, addr, length):
        """Memory Read Aux function"""
        cnt = 1
        e_size = length
        skip_size = 0
        n = 11
        byte_list = [0] * n
        byte_list[0] = n
        byte_list[1] = 0xFB  # MREADV_OP
        byte_list[2] = addr & 0xFF
        byte_list[3] = (addr >> 8) & 0xFF
        byte_list[4] = (addr >> 16) & 0xFF
        byte_list[5] = (addr >> 24) & 0xFF
        byte_list[6] = cnt
        byte_list[7] = e_size
        byte_list[8] = skip_size & 0xFF
        byte_list[9] = (skip_size >> 8) & 0xFF
        byte_list[10] = rcb4_checksum(byte_list)
        return self.serial_write(byte_list)

    def memory_read(self, addr, length):
        limit = 250
        if length < limit:
            return self.memory_read_aux(addr, length)
        elif length < 2 * limit:
            return self.memory_read_aux(addr, limit) + self.memory_read_aux(
                addr + limit, length - limit
            )
        else:
            return (
                self.memory_read_aux(addr, limit)
                + self.memory_read_aux(addr + limit, limit)
                + self.memory_read_aux(addr + 2 * limit, length - 2 * limit)
            )

    def memory_cstruct(self, cls, v_idx=0, addr=None, size=None):
        if addr is None:
            addr = self.armh7_address[cls.__name__]
        if size is None:
            size = cls.size
        buf = self.memory_read((size * v_idx) + addr, size)
        return cls(buf)

    def memory_write(self, addr, length, data):
        addr = int(addr)  # for numpy
        cnt = 1
        e_size = length
        skip_size = 0
        n = length + 11
        byte_list = bytearray(n)
        byte_list[0] = n
        byte_list[1] = 0xFC  # MWRITEV_OP
        byte_list[2:6] = addr.to_bytes(4, byteorder="little")
        byte_list[6] = cnt
        byte_list[7] = e_size
        byte_list[8:10] = skip_size.to_bytes(2, byteorder="little")
        for i in range(length):
            byte_list[10 + i] = data[i]
        byte_list[10 + length] = rcb4_checksum(byte_list[0 : n - 1])
        return self.serial_write(byte_list)

    def cfunc_call(self, func_string, *args):
        addr = self.armh7_address[func_string]
        argc = len(args)
        n = 8 + 4 * argc
        byte_list = bytearray(n)
        byte_list[0] = n
        byte_list[1] = 0xFA
        byte_list[2:6] = int(addr).to_bytes(4, byteorder="little")
        byte_list[6] = argc
        for i in range(argc):
            byte_list[7 + i * 4 : 7 + (i + 1) * 4] = int(args[i]).to_bytes(
                4, byteorder="little"
            )
        byte_list[n - 1] = rcb4_checksum(byte_list[0 : n - 1])
        return self.serial_write(byte_list)

    def write_cls_alist(self, cls, idx, slot_name, vec):
        baseaddr = self.armh7_address[cls.__name__]
        cls_size = cls.size
        typ = cls.__fields_types__[slot_name].c_type
        slot_size = c_type_to_size(typ)
        slot_offset = cls.__fields_types__[slot_name].offset
        cnt = 1
        addr = baseaddr + (idx * cls_size) + slot_offset
        data = bytearray(slot_size)

        if cnt == 1:
            v = None
            if isinstance(vec, list):
                v = vec[0]
            else:
                v = vec
            if typ == "float":
                byte_data = struct.pack("f", v)
                byte_data = struct.unpack("4B", byte_data)
            elif typ == "uint16":
                byte_data = struct.pack("H", v)
                byte_data = struct.unpack("BB", byte_data)
            elif typ == "uint8":
                # Unpacking a single byte.
                byte_data = struct.pack("B", v)
                byte_data = struct.unpack("B", byte_data)
            else:
                raise RuntimeError(f"not implemented typ {typ}")
            data[0 : len(byte_data)] = byte_data
        else:
            raise NotImplementedError
        return self.memory_write(addr, slot_size, data)

    def read_cstruct_slot_vector(self, cls, slot_name):
        vcnt = c_vector[cls.__name__] or 1
        addr = self.armh7_address[cls.__name__]
        cls_size = cls.size
        field_type = cls.__fields_types__[slot_name]
        slot_offset = field_type.offset
        c_type = field_type.c_type
        vlen = getattr(field_type, "vlen", 1)
        element_size = c_type_to_size(c_type) * vlen
        n = 11
        byte_list = np.zeros(n, dtype=np.uint8)
        byte_list[0] = n
        byte_list[1] = 0xFB  # MREADV_OP
        byte_list[2:6] = np.array(
            [(addr + slot_offset) >> (i * 8) & 0xFF for i in range(4)], dtype=np.uint8
        )
        byte_list[6] = vcnt
        byte_list[7] = element_size
        if cls_size > 0xFFFF:
            raise ValueError(f"skip_size={cls_size} is too large for 2 bytes.")
        byte_list[8] = cls_size & 0xFF
        byte_list[9] = (cls_size >> 8) & 0xFF

        byte_list[n - 1] = rcb4_checksum(byte_list)
        b = self.serial_write(byte_list)
        return np.frombuffer(b, dtype=c_type_to_numpy_format(c_type))

    def read_jointbase_sensor_ids(self):
        if self.id_vector is None:
            ret = []
            for i in range(max_sensor_num):
                sensor = self.memory_cstruct(SensorbaseStruct, i)
                port = sensor.port
                id = sensor.id
                if port > 0 and id == (i + sensor_sidx) // 2:
                    ret.append(i + sensor_sidx)
            self.id_vector = sorted(ret)
        return self.id_vector

    def reference_angle_vector(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.zeros(shape=0)
        ref_angles = self.read_cstruct_slot_vector(ServoStruct, slot_name="ref_angle")[
            servo_ids
        ]
        return ref_angles

    def servo_error(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.zeros(shape=0)
        error_angles = self.read_cstruct_slot_vector(
            ServoStruct, slot_name="error_angle"
        )[servo_ids]
        return error_angles

    def adjust_angle_vector(self, servo_ids=None, error_threshold=None):
        """Stop servo motor when joint error become large.

        This function stops servo motor when the error between reference angle
        vector and current angle vector exceeds error threshold.

        Parameters
        ----------
        servo_ids : array_like
            Array of error check target servo IDs. Each ID corresponds to a
            specific servo.
        error_threshold : None or numbers.Number or numpy.ndarray
            Error threshold angle [deg].

        Returns
        -------
        bool
            Return True when adjustment occurs.
        """
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        servo_ids = np.array(servo_ids)
        # Ignore free servo
        servo_ids = servo_ids[self.reference_angle_vector(servo_ids=servo_ids) != ServoOnOffValues.OFF.value]
        if len(servo_ids) == 0:
            return False

        # Calculate error threshold[deg]
        if error_threshold is None:
            error_threshold = 5
        if isinstance(error_threshold, numbers.Number):
            error_threshold = np.full(len(servo_ids), error_threshold, dtype=np.float32)
        assert isinstance(
            error_threshold, np.ndarray
        ), "error_threshold must be np.ndarray"
        assert len(servo_ids) == len(
            error_threshold
        ), "length of servo_ids and error_threshold must be equal"

        # Find servo whose angle exceeds error threshold
        current_av = self.angle_vector(servo_ids=servo_ids)
        reference_servo_av = self.reference_angle_vector(servo_ids=servo_ids)
        reference_av = self.servo_angle_vector_to_angle_vector(
            reference_servo_av, servo_ids
        )
        error_av = current_av - reference_av
        error_indices = abs(error_av) > error_threshold
        error_ids = servo_ids[error_indices]

        # Stop motion
        if len(error_ids) > 0:
            print(
                f"Servo {error_ids} error {error_av[error_indices]}[deg]"
                + f" exceeds threshold {error_threshold[error_indices]}[deg]."
            )
            print(
                "Stop motion by overriding reference angle vector with"
                + " current angle vector."
            )
            all_servo_ids = self.search_servo_ids()
            self.servo_angle_vector(
                all_servo_ids, servo_vector=self._angle_vector()[all_servo_ids]
            )
            return True
        return False

    def servo_id_to_index(self, servo_id):
        if self.valid_servo_ids([servo_id]):
            return self.sequentialized_servo_ids([servo_id])[0]

    def sequentialized_servo_ids(self, servo_ids):
        if len(servo_ids) == 0:
            return np.zeros(shape=0, dtype=np.uint8)
        return self._servo_id_to_sequentialized_servo_id[np.array(servo_ids)].astype(
            np.uint8
        )

    def _angle_vector(self):
        return self.read_cstruct_slot_vector(ServoStruct, slot_name="current_angle")

    def _send_angle_vector(self, av, servo_ids=None, velocity=127):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(av) != len(servo_ids):
            raise ValueError("Length of servo_ids and angle_vector must be the same.")
        av = np.array(av)
        worm_av = []
        worm_indices = []
        for i, (angle, servo_id) in enumerate(zip(av, servo_ids)):
            if servo_id in self._servo_id_to_worm_id:
                worm_av.append(angle)
                worm_indices.append(self._servo_id_to_worm_id[servo_id])
                av[i] = 135
        if len(worm_indices) > 0:
            if self._worm_ref_angle is None:
                self._worm_ref_angle = np.array(
                    self.read_cstruct_slot_vector(WormmoduleStruct, "ref_angle"),
                    dtype=np.float32,
                )
            self._worm_ref_angle[np.array(worm_indices)] = np.array(worm_av)
            self.write_cstruct_slot_v(WormmoduleStruct, "ref_angle", self._worm_ref_angle)
        svs = self.angle_vector_to_servo_angle_vector(av, servo_ids)
        return self.servo_angle_vector(servo_ids, svs, velocity=velocity)

    def angle_vector(self, av=None, servo_ids=None, velocity=127):
        if av is not None:
            return self._send_angle_vector(av, servo_ids, velocity)
        all_servo_ids = self.search_servo_ids()
        if len(all_servo_ids) == 0:
            return np.zeros(shape=0)
        av = self.servo_angle_vector_to_angle_vector(
            self._angle_vector()[all_servo_ids], all_servo_ids
        )
        worm_av = self.read_cstruct_slot_vector(
            WormmoduleStruct, slot_name="present_angle"
        )
        for worm_idx in self.search_worm_ids():
            av[self.servo_id_to_index(self.worm_id_to_servo_id[worm_idx])] = worm_av[
                worm_idx
            ]
        if servo_ids is not None:
            if len(servo_ids) == 0:
                return np.zeros(shape=0)
            av = av[self.sequentialized_servo_ids(servo_ids)]
        return av

    def servo_angle_vector_to_angle_vector(self, servo_av, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(servo_av) != len(servo_ids):
            raise ValueError("Length of servo_ids and angle_vector must be the same.")
        if len(servo_ids) == 0:
            return np.zeros(shape=0)
        seq_indices = self.sequentialized_servo_ids(servo_ids)
        tmp_servo_av = np.append(np.zeros(len(self.servo_sorted_ids)), 1)
        tmp_servo_av[seq_indices] = np.array(servo_av)
        tmp_av = np.matmul(tmp_servo_av.T, self.actuator_to_joint_matrix.T)[:-1]
        return tmp_av[seq_indices]

    def angle_vector_to_servo_angle_vector(self, av, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(av) != len(servo_ids):
            raise ValueError("Length of servo_ids and angle_vector must be the same.")
        if len(servo_ids) == 0:
            return np.zeros(shape=0)
        seq_indices = self.sequentialized_servo_ids(servo_ids)
        tmp_av = np.append(np.zeros(len(self.servo_sorted_ids)), 1)
        tmp_av[seq_indices] = np.array(av)
        return np.matmul(self.joint_to_actuator_matrix, tmp_av)[seq_indices]

    def _trim_servo_vector(self):
        return np.array(
            self.read_cstruct_slot_vector(ServoStruct, "trim"), dtype=np.int16
        )

    def _set_trim_vector(self, trim_vector=None, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(trim_vector) != len(servo_ids):
            raise ValueError("Length of servo_ids and trim_vector must be the same.")
        if len(trim_vector) == 0:
            return
        current_trim_vector = self._trim_servo_vector()
        current_trim_vector[np.array(servo_ids)] = trim_vector
        return self.write_cstruct_slot_v(ServoStruct, "trim", current_trim_vector)

    def trim_vector(self, av=None, servo_ids=None):
        if av is not None:
            trim_av = self.angle_vector_to_servo_angle_vector(av, servo_ids) - 7500
            return self._set_trim_vector(trim_av, servo_ids)
        all_servo_ids = self.search_servo_ids()
        if len(all_servo_ids) == 0:
            return np.zeros(shape=0)
        trim = np.append(7500 + self._trim_servo_vector()[all_servo_ids], 1)
        trim_av = np.matmul(trim.T, self.actuator_to_joint_matrix.T)[:-1]
        if servo_ids is not None:
            if len(servo_ids) == 0:
                return np.zeros(shape=0)
            trim_av = trim_av[self.sequentialized_servo_ids(servo_ids)]
        return trim_av

    def clear_trim_vector(self, write_to_flash=True):
        self.write_cstruct_slot_v(ServoStruct, "trim", np.zeros(rcb4_dof))
        if write_to_flash:
            self.write_to_flash()

    def ics_start(self):
        return self.set_cstruct_slot(SystemStruct, 0, "ics_comm_stop", [0, 0, 0, 0, 0, 0])

    def ics_stop(self):
        return self.set_cstruct_slot(SystemStruct, 0, "ics_comm_stop", [1, 1, 1, 1, 1, 1])

    def idmode_scan(self):
        self.ics_stop()
        self.cfunc_call("servo_idmode_scan")
        self.ics_start()

        self.servo_sorted_ids = None
        self.worm_sorted_ids = None
        self.search_worm_ids()
        self.search_servo_ids()

    def battery_voltage(self):
        return self.memory_cstruct(SystemStruct).battery_voltage

    def search_servo_ids(self):
        if self.servo_sorted_ids is not None:
            return self.servo_sorted_ids
        servo_indices = []
        wheel_indices = []
        for idx in range(rcb4_dof):
            servo = self.memory_cstruct(ServoStruct, idx)
            if servo.flag > 0:
                servo_indices.append(idx)
                if idx not in self._servo_id_to_worm_id:
                    # wheel
                    if servo.rotation > 0:
                        wheel_indices.append(idx)
                    if servo.feedback > 0:
                        self.set_cstruct_slot(ServoStruct, idx, "feedback", 0)
        servo_indices = np.array(servo_indices)
        self.wheel_servo_sorted_ids = sorted(wheel_indices)
        self.servo_sorted_ids = servo_indices

        self._servo_id_to_sequentialized_servo_id = np.nan * np.ones(rcb4_dof)
        servo_indices = np.array(servo_indices)
        if len(servo_indices):
            self._servo_id_to_sequentialized_servo_id[servo_indices] = np.arange(
                len(servo_indices)
            )
        servo_on_states = self.servo_states()
        self.servo_on_states_dict = {}
        for index in servo_indices:
            if index in servo_on_states:
                self.servo_on_states_dict[index] = True
            else:
                self.servo_on_states_dict[index] = False
        # internally call actuator_to_joint_matrix to initialize it.
        self.actuator_to_joint_matrix  # NOQA
        return servo_indices

    def search_air_board_ids(self):
        # air_boards and air_board_ids are in the same order
        air_boards = self.all_air_boards()
        air_board_ids = []
        for air_board in air_boards:
            for idx in self.id_vector:
                if air_board.id == idx // 2:
                    air_board_ids.append(idx)
        return np.array(air_board_ids)

    def valid_servo_ids(self, servo_ids):
        return np.isfinite(self._servo_id_to_sequentialized_servo_id[np.array(servo_ids)])

    def hold(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [ServoOnOffValues.ON.value] * len(servo_ids)
        return self.servo_angle_vector(servo_ids, servo_vector, velocity=1000)

    def free(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [ServoOnOffValues.OFF.value] * len(servo_ids)
        return self.servo_angle_vector(servo_ids, servo_vector, velocity=1000)

    def neutral(self, servo_ids=None, velocity=1000):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        av = [0] * len(servo_ids)
        return self.angle_vector(av, servo_ids, velocity=velocity)

    def servo_angle_vector(self, servo_ids, servo_vector, velocity=127):
        """Sends a command to control multiple servos.

        This function sorts the servo IDs and corresponding angles,
        constructs a command byte list, and sends it.
        The velocity parameter is clamped between 1 and 255.

        Parameters
        ----------
        servo_ids : array_like
            Array of servo IDs. Each ID corresponds to a specific servo.
        servo_vector : array_like
            Array of angles (in servo pulse) for the servos.
            Each angle corresponds to the servo ID at the same index
            in servo_ids.
        velocity : int, or array like optional
            Number of frames for the servo movement duration,
            clamped between 1 and 255. Default value is 127.
            Converted to time in milliseconds by the following equation:
                ms = velocity * FRAME_SCALE
            FRAME_SCALE is 10 by default.
            (https://github.com/jsk-ros-pkg/control_boards/blob/3440bec59c9229900d410a767edd405da3c8d194/projects/kondoh7/Core/Inc/main.h#L86C24-L86C64)  # NOQA

        Raises
        ------
        ValueError
            If the length of `servo_ids` does not match the length
            of `servo_vector`.

        Notes
        -----
        The function internally sorts `servo_ids` and `servo_vector`
        based on the servo IDs to maintain the correspondence
        between each servo ID and its angle. This sorted order is
        used for constructing the command byte list.
        """
        if len(servo_ids) != len(servo_vector):
            raise ValueError("Length of servo_ids and servo_vector must be the same.")

        # Update servo on/off states based on 32767(Servo ON) and 32768(Servo OFF) values in servo_vector
        for servo_id, angle in zip(servo_ids, servo_vector):
            if angle == ServoOnOffValues.ON.value:
                self.servo_on_states_dict[servo_id] = True
            elif angle == ServoOnOffValues.OFF.value:
                self.servo_on_states_dict[servo_id] = False

        # Filter servo IDs based on their on state in servo_on_states_dict
        active_ids = []
        active_angles = []
        for servo_id, angle in zip(servo_ids, servo_vector):
            if self.servo_on_states_dict.get(servo_id, False) or angle in (
                ServoOnOffValues.ON.value,
                ServoOnOffValues.OFF.value,
            ):
                # Only include active servos
                active_ids.append(servo_id)
                active_angles.append(angle)

        # If no active servos, skip command sending
        if not active_ids:
            # print("[servo_angle_vector] No active servos to send commands.")
            return

        # Sort the servo vectors based on servo IDs
        sorted_indices = np.argsort(active_ids)
        sorted_servo_ids = np.array(active_ids)[sorted_indices]
        sorted_servo_vector = np.array(active_angles)[sorted_indices]

        # Prepare the command byte list
        if (
            isinstance(velocity, list)
            or isinstance(velocity, tuple)
            or isinstance(velocity, np.ndarray)
        ):
            sorted_servo_velocities = np.array(velocity)[sorted_indices]
            byte_list = (
                [CommandTypes.MultiServoMultiVelocities.value]
                + encode_servo_ids_to_5bytes_bin(sorted_servo_ids)
                + encode_servo_velocity_and_position_to_bytes(
                    sorted_servo_velocities, sorted_servo_vector
                )
            )
        else:
            byte_list = (
                [CommandTypes.MultiServoSingleVelocity.value]
                + encode_servo_ids_to_5bytes_bin(sorted_servo_ids)
                + [rcb4_velocity(velocity)]
                + encode_servo_positions_to_bytes(sorted_servo_vector)
            )

        # Add header (length) and checksum to the byte list
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))

        # send the command
        return self.serial_write(byte_list)

    def servo_param64(self, sid, param_names=None):
        v = self.memory_cstruct(ServoStruct, sid)
        param_values = {}
        if param_names is None:
            param_names = list(servo_eeprom_params64.keys())
        for param_name in param_names:
            indices = servo_eeprom_params64[param_name]
            param_value = four_bit_to_num(indices, v.params)
            param_values[param_name] = param_value
        return param_values

    def read_stretch(self, servo_ids=None):
        """Returns servo motor stretch value.

        'Stretch' refers to the holding force of the servo's output shaft.
        Increasing this value strengthens the holding force, while decreasing
        it weakens the force, allowing for a softer, more flexible state
        similar to human joints. Setting an appropriate value can help absorb
        shocks during activities like walking, maintaining balance by reducing
        impact when the foot contacts the ground.

        Parameters
        ----------
        servo_ids : List[int]
            List of servo ID whose stretch value is to be read.
            If None, read the stretch values of all connected servo motors.

        Returns
        -------
        List[int]
            List of stretch values corresponding to servo_ids.
        """
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        return [
            self.servo_param64(sid, ["stretch_gain"])["stretch_gain"] // 2
            for sid in servo_ids
        ]

    def send_stretch(self, value=127, servo_ids=None):
        """Write stretch value to servo motor.

        'Stretch' refers to the holding force of the servo's output shaft.
        Increasing this value strengthens the holding force, while decreasing
        it weakens the force, allowing for a softer, more flexible state
        similar to human joints. Setting an appropriate value can help absorb
        shocks during activities like walking, maintaining balance by reducing
        impact when the foot contacts the ground.

        Parameters
        ----------
        value : int
            Stretch value
        servo_ids : List[int]
            List of servo ID whose stretch value is to be sent.
            If None, write the stretch values to all connected servo motors.

        Returns
        -------
        str
            If b'\x12\x06' (which contains ACK) is returned, command succeed.
            If b'\x12\x15' (which contains NCK) is returned, command fail.
        """
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        if not isinstance(value, list) or not isinstance(value, tuple):
            value = [value] * len(servo_ids)
        byte_list = (
            [CommandTypes.ServoParam.value]
            + encode_servo_ids_to_5bytes_bin(servo_ids)
            + [ServoParams.Stretch.value]
            + rcb4_servo_svector(servo_ids, value)
        )
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))
        return self.serial_write(byte_list)

    def send_current_limit(self, current_limit_a=4.0, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        current_setting = int(current_limit_a * 10)
        if not isinstance(current_setting, (list, tuple)):
            current_setting = [current_setting] * len(servo_ids)
        byte_list = (
            [CommandTypes.ServoParam.value]
            + encode_servo_ids_to_5bytes_bin(servo_ids)
            + [ServoParams.CurrentLimit.value]
            + rcb4_servo_svector(servo_ids, current_setting)
        )
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))
        return self.serial_write(byte_list)

    def send_temperature_limit(self, temperature_limit_c=80, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        temperature_setting = get_setting_value_from_temperatures(temperature_limit_c)
        if not isinstance(temperature_setting, (list, tuple)):
            temperature_setting = [temperature_setting] * len(servo_ids)
        byte_list = (
            [CommandTypes.ServoParam.value]
            + encode_servo_ids_to_5bytes_bin(servo_ids)
            + [ServoParams.TemperatureLimit.value]
            + rcb4_servo_svector(servo_ids, temperature_setting)
        )
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))
        return self.serial_write(byte_list)

    def read_quaternion(self):
        cs = self.memory_cstruct(Madgwick, 0)
        return np.array([cs.q0, cs.q1, cs.q2, cs.q3], dtype=np.float32)

    def read_rpy(self):
        cs = self.memory_cstruct(Madgwick, 0)
        return [cs.roll, cs.pitch, cs.yaw]

    def read_imu_data(self):
        cs = self.memory_cstruct(Madgwick, 0)
        # MPU9250 acceleration measurement range is +-8g
        acc = convert_data(cs.acc, 8)
        g = 9.81
        acc = acc * g
        q_wxyz = np.array([cs.q0, cs.q1, cs.q2, cs.q3], dtype=np.float32)
        norm_q = np.sqrt(np.dot(q_wxyz.T, q_wxyz))
        q_wxyz = q_wxyz / norm_q
        gyro = convert_data(cs.gyro, 2000)
        gyro = np.deg2rad(gyro)
        return q_wxyz, acc, gyro

    def gyro_norm_vector(self):
        g = self.memory_cstruct(Madgwick, 0).gyro
        n = np.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)
        return (n, g)

    def set_cstruct_slot(self, cls, idx, slot_name, v):
        if not isinstance(v, list) or isinstance(v, tuple) or isinstance(v, np.ndarray):
            v = [v]
        cnt = len(v)

        baseaddr = self.armh7_address[cls.__name__]
        cls_size = cls.size
        typ = cls.__fields_types__[slot_name].c_type
        slot_size = cnt * c_type_to_size(typ)
        slot_offset = cls.__fields_types__[slot_name].offset
        addr = baseaddr + (idx * cls_size) + slot_offset
        bytes = bytearray(slot_size)

        byte_data = b""
        for value in v:
            if typ == "float":
                tmp_byte_data = struct.pack("f", value)
                byte_data += struct.unpack("4B", tmp_byte_data)
            elif typ == "uint16":
                tmp_byte_data = struct.pack("H", value)
                byte_data += struct.unpack("BB", tmp_byte_data)
            elif typ == "uint8":
                byte_data += value.to_bytes(1, "little")
            else:
                raise RuntimeError(f"not implemented typ {typ}")
        bytes[0 : len(byte_data)] = byte_data
        return self.memory_write(addr, slot_size, bytes)

    def search_worm_ids(self):
        if self.worm_sorted_ids is not None:
            return self.worm_sorted_ids
        indices = []
        self._servo_id_to_worm_id = {}
        self._worm_id_to_servo_id = {}
        for idx in range(max_sensor_num):
            worm = self.memory_cstruct(WormmoduleStruct, idx)
            if worm.module_type == 1:
                servo = self.memory_cstruct(ServoStruct, worm.servo_id)
                if servo.rotation == 1:
                    indices.append(idx)
                    if servo.feedback > 0:
                        self.set_cstruct_slot(ServoStruct, worm.servo_id, "feedback", 0)
                self._servo_id_to_worm_id[worm.servo_id] = idx
                self._worm_id_to_servo_id[idx] = worm.servo_id
        self.worm_sorted_ids = indices
        return indices

    def calibrate_worm(self, worm_idx, servo_idx, sensor_idx, magenc_present=None):
        if magenc_present is None:
            input(
                f"Servo ID: {servo_idx}, Sensor ID: {sensor_idx}\n"
                + "Press Enter to confirm the current angle: "
            )
            worm = self.memory_cstruct(WormmoduleStruct, worm_idx)
            magenc_present = worm.magenc_present
        self.send_worm_calib_data(
            worm_idx, servo_idx, sensor_idx, module_type=1, magenc_offset=magenc_present
        )
        worm = self.memory_cstruct(WormmoduleStruct, worm_idx)
        print(
            f"Magnetic encoder offset for Servo ID {servo_idx} "
            + f"set to: {worm.magenc_init}"
        )
        return worm.magenc_init

    def read_worm_angle(self, idx=0):
        if not 0 <= idx < max_sensor_num:
            print(
                f"Error: The worm servo index {idx} is out of the valid range."
                + " Please provide an index between 0 and {max_sensor_num - 1}."
            )
        if self.worm_sorted_ids is None:
            self.worm_sorted_ids = self.search_worm_ids()
        if idx not in self.worm_sorted_ids:
            print(
                f"Error: The worm servo index {idx} is out of the valid range."
                + f" Valid indices are: {self.worm_sorted_ids}. "
                + "Please provide an index from this list."
            )
        return self.memory_cstruct(WormmoduleStruct, idx).present_angle

    def send_worm_angle_and_threshold(
        self, worm_idx=0, angle=0, threshold=30, threshold_scale=1.0, sign=1
    ):
        angle_lst = [angle]
        th_lst = [threshold]
        th_scale_lst = [threshold_scale]
        if not 0 <= worm_idx < max_sensor_num:
            print(f"invalid argument worm_idx={worm_idx}")
        else:
            worm = self.memory_cstruct(WormmoduleStruct, worm_idx)
            if worm.ref_angle != angle:
                self.write_cls_alist(WormmoduleStruct, worm_idx, "ref_angle", angle_lst)
            if worm.thleshold != threshold:
                self.write_cls_alist(WormmoduleStruct, worm_idx, "thleshold", th_lst)
            if worm.thleshold_scale != threshold_scale:
                self.write_cls_alist(
                    WormmoduleStruct, worm_idx, "thleshold_scale", th_scale_lst
                )

    def send_worm_calib_data(
        self,
        worm_idx,
        servo_idx=None,
        sensor_idx=None,
        module_type=None,
        magenc_offset=None,
        upper_limit=69.0,
        thleshold_scale=5.0,
        timeout_time_scale=1.25,
        gear_ratio=20.0,
    ):
        # Check for valid worm_idx
        if not 0 <= worm_idx < max_sensor_num:
            print(f"Invalid argument worm_idx={worm_idx}")
            return

        # Print the formatted information
        print(
            f"send worm_idx: {worm_idx}, module_type: {module_type}"
            + f", servo_idx: {servo_idx} sensor_idx: {sensor_idx}"
        )
        print(
            f"magenc_offset: {magenc_offset}, upper_limit: {upper_limit}, "
            + f"thleshold_scale: {thleshold_scale}, "
            + f"timeout_time_scale: {timeout_time_scale}"
            + f", gear_ratio: {gear_ratio}"
        )

        # Write the parameters to the wormmodule vector cstruct
        if module_type is not None:
            self.write_cls_alist(WormmoduleStruct, worm_idx, "module_type", [module_type])
        if servo_idx is not None:
            self.write_cls_alist(WormmoduleStruct, worm_idx, "servo_id", [servo_idx])
        if sensor_idx is not None:
            self.write_cls_alist(WormmoduleStruct, worm_idx, "sensor_id", [sensor_idx])
        self.write_cls_alist(WormmoduleStruct, worm_idx, "move_state", [0])
        if magenc_offset is not None:
            self.write_cls_alist(
                WormmoduleStruct, worm_idx, "magenc_init", [magenc_offset % (2**14)]
            )
        if upper_limit is not None:
            self.write_cls_alist(
                WormmoduleStruct, worm_idx, "linear_upper_limit", [upper_limit]
            )
        if thleshold_scale is not None:
            self.write_cls_alist(
                WormmoduleStruct, worm_idx, "thleshold_scale", [thleshold_scale]
            )
        if timeout_time_scale is not None:
            self.write_cls_alist(
                WormmoduleStruct, worm_idx, "timeout_time_scale", [timeout_time_scale]
            )
        if gear_ratio is not None:
            self.write_cls_alist(WormmoduleStruct, worm_idx, "gear_ratio", [gear_ratio])
        return self.memory_cstruct(WormmoduleStruct, worm_idx)

    def read_worm_calib_data(self, worm_idx):
        # Check for valid worm_idx
        if not 0 <= worm_idx < max_sensor_num:
            print(f"Invalid argument worm_idx={worm_idx}")
            return
        return self.memory_cstruct(WormmoduleStruct, worm_idx)

    def copy_worm_params_from_flash(self):
        for i in range(max_sensor_num):
            self.dataflash_to_dataram(WormmoduleStruct, i)
        self.write_cstruct_slot_v(
            WormmoduleStruct, "move_state", np.zeros(max_sensor_num)
        )

    def buzzer(self):
        return self.cfunc_call("buzzer_init_sound")

    def write_to_flash(self):
        # The operation write_to_flash is time-consuming,
        # hence the default_timeout is temporarily extended.
        default_timeout = self._default_timeout
        self._default_timeout = 5.0
        ret = self.cfunc_call("rom_to_flash")
        self._default_timeout = default_timeout
        return ret

    def set_data_address(self):
        self.set_sidata()
        self.set_sdata()
        self.set_edata()
        self.set_data_size()

    def dataflash_to_dataram(self, cls, idx=0):
        addr = self.armh7_address[cls.__name__] + idx * cls.size
        saddr = self.armh7_address["_sdata"]
        offset = addr - saddr
        faddr = self.cstruct_slot(DataAddress, "dataflash_address")
        self.cstruct_slot(DataAddress, "src_addr", float(faddr[0]) + offset)
        self.cstruct_slot(DataAddress, "dst_addr", addr)
        self.cstruct_slot(DataAddress, "copy_size", cls.size)
        return self.cfunc_call("dataflash_to_dataram")

    def databssram_to_dataflash(self):
        self.set_data_address()
        self.cstruct_slot(
            DataAddress,
            "data_size",
            self.armh7_address["_ebss"] - self.armh7_address["_sdata"],
        )

        # The operation databssram_to_dataflash is time-consuming,
        # hence the default_timeout is temporarily extended.
        default_timeout = self._default_timeout
        self._default_timeout = 5.0
        ret = self.cfunc_call("dataram_to_dataflash")
        self._default_timeout = default_timeout
        return ret

    def set_sidata(self, sidata=None):
        sidata = sidata or self.armh7_address["_sidata"]
        return self.cstruct_slot(DataAddress, "_sidata", sidata)

    def set_sdata(self, sdata=None):
        sdata = sdata or self.armh7_address["_sdata"]
        return self.cstruct_slot(DataAddress, "_sdata", sdata)

    def set_edata(self, edata=None):
        edata = edata or self.armh7_address["_edata"]
        return self.cstruct_slot(DataAddress, "_edata", edata)

    def set_data_size(self, sdata=None, edata=None):
        sdata = sdata or self.armh7_address["_sdata"]
        edata = edata or self.armh7_address["uwTickPrio"]
        return self.cstruct_slot(DataAddress, "data_size", edata - sdata)

    def cstruct_slot(self, cls, slot_name, v=None):
        if v is not None:
            return self.write_cstruct_slot_v(cls, slot_name, v)
        return self.read_cstruct_slot_vector(cls, slot_name)

    def write_cstruct_slot_v(self, cls, slot_name, vec):
        vcnt = c_vector[cls.__name__] or 1
        addr = self.armh7_address[cls.__name__]
        skip_size = cls.size
        field_type = cls.__fields_types__[slot_name]
        typ = field_type.c_type
        # Number of elements in array (e.g., 6 for ics_baudrate number of ports)
        cnt = getattr(field_type, "vlen", 1)
        esize = c_type_to_size(typ)
        offset = field_type.offset
        tsize = cnt * esize

        n = 11 + tsize * vcnt
        if n > 240:
            print(f"n={n} should be less than 240")
            return

        byte_list = bytearray(n)
        byte_list[0] = n
        byte_list[1] = 0xFC  # MWRITEV_OP
        struct.pack_into("<I", byte_list, 2, addr + offset)
        byte_list[6] = vcnt
        byte_list[7] = tsize
        struct.pack_into("<H", byte_list, 8, skip_size)

        for i in range(vcnt):
            # vec is a list of arrays, get the i-th array
            array_data = vec[i] if isinstance(vec, list) and len(vec) > i else vec

            if cnt == 1:
                # Single element
                if isinstance(array_data, (list, tuple, np.ndarray)) and len(array_data) > 0:
                    v = array_data[0]
                else:
                    v = array_data
                if not isinstance(v, (int, float)):
                    v = v[0] if hasattr(v, '__len__') and len(v) > 0 else v
                if typ == "uint8":
                    v = int(round(v))
                    struct.pack_into("<B", byte_list, 10 + i * esize, v)
                elif typ == "uint16":
                    v = int(round(v))
                    struct.pack_into("<H", byte_list, 10 + i * esize, v)
                elif typ == "int16":
                    v = int(round(v))
                    struct.pack_into("<h", byte_list, 10 + i * esize, v)
                elif typ == "uint32":
                    v = int(round(v))
                    struct.pack_into("<I", byte_list, 10 + i * esize, v)
                elif typ in ("float", "double"):
                    struct.pack_into("<f", byte_list, 10 + i * esize, v)
                else:
                    raise RuntimeError(f"Not implemented case for typ {typ}")
            else:
                # Array of elements (like ics_baudrate with 6 elements)
                for j in range(cnt):
                    if isinstance(array_data, (list, tuple, np.ndarray)) and j < len(array_data):
                        v = array_data[j]
                    else:
                        v = 0  # Default value
                    if typ in ("uint8", "uint16", "uint32", "int16"):
                        v = int(round(v))
                    if typ == "uint8":
                        struct.pack_into("<B", byte_list, 10 + i * tsize + j * esize, v)
                    elif typ == "uint16":
                        struct.pack_into("<H", byte_list, 10 + i * tsize + j * esize, v)
                    elif typ == "int16":
                        struct.pack_into("<h", byte_list, 10 + i * tsize + j * esize, v)
                    elif typ == "uint32":
                        struct.pack_into("<I", byte_list, 10 + i * tsize + j * esize, v)
                    elif typ in ("float", "double"):
                        struct.pack_into("<f", byte_list, 10 + i * tsize + j * esize, v)
                    else:
                        raise RuntimeError(f"Not implemented case for typ {typ}")

        byte_list[n - 1] = rcb4_checksum(byte_list)
        s = self.serial_write(byte_list)
        s = padding_bytearray(s, tsize)
        return np.frombuffer(s, dtype=c_type_to_numpy_format(typ))

    def scan_ics_channels_per_port(self, timeout=False, raw_id=True):
        """Return the ICS or SIO ID for devices on each ICS channel (J1..J6).

        If `raw_id` is True, we return ICS ID (i.e., index // max_SIO_num).
        If `raw_id` is False, we return the raw index from read_device_port().
        If `timeout` is True and we see the 'timeout' bit, we set the ID to -1.

        Returns
        -------
        list of list of int
            A list with 6 elements (for J1..J6). Each element is a list of the
            IDs found on that ICS channel.
        """
        # Store the final result for each ICS channel (6 total).
        result = [[], [], [], [], [], []]
        tmp_device_port = self.read_device_port()
        ics_channel = [(None, 0), (None, 1), (None, 2), (None, 3), (None, 4), (None, 5)]

        for j in range(6):
            tmp_lst = []
            tmp_port = ics_channel[j][1] + 1
            for i, tmp_state in enumerate(tmp_device_port):
                if tmp_state >= 128:
                    tmp_state -= 128
                    timeout_flag = False
                    if tmp_state >= 64:
                        tmp_state -= 64
                        timeout_flag = True
                    if tmp_state == (2 ** (tmp_port - 1)):
                        tmp_id = i
                        if raw_id:
                            tmp_id = tmp_id // max_SIO_num
                        if timeout and timeout_flag:
                            tmp_id = -1
                        tmp_lst.append(tmp_id)
            result[j] = tmp_lst
        return result

    def read_device_port(self):
        return self.read_cstruct_slot_vector(ControlStruct, "device_port")

    def read_jb_cstruct(self, idx):
        return self.memory_cstruct(SensorbaseStruct, idx - sensor_sidx)

    def read_gpio_cstruct(self, idx):
        return self.memory_cstruct(GPIOStruct, idx - sensor_sidx)

    def all_jointbase_sensors(self):
        if self.id_vector is None:
            self.read_jointbase_sensor_ids()
        return [self.read_jb_cstruct(idx) for idx in self.id_vector]

    def all_air_boards(self):
        jointbase_sensors = self.all_jointbase_sensors()
        return [j for j in jointbase_sensors if j.board_revision == 3]

    def read_pressure_sensor(self, board_idx):
        """Returns pressure sensor value of air_relay board.

        Equation for calculating pressure value is following
        - v_diff[V]:
            (3.1395 * pressure[kPa] + 10.739) / 1000
            output of wheatstone bridge in pressure sensor
            when sensor input voltage is 5V, so need to convert for 4.14V.
            4.14V is determined by constant current circuit.
            -> See https://api.puiaudio.com/file/2ee7baab-a644-43d5-
               96e0-0b83ff2e8e64.pdf, p.3
        - v_amplified[V]:
            v_diff[V] * GAIN + V_OFFSET[V]
        - adc_raw:
            v_amplified[V] / 3.3[V] * 4095 (12bit)
        """
        V_OFFSET = 1.65
        GAIN = 7.4

        all_sensor_data = self.read_jb_cstruct(board_idx)
        adc_raw = all_sensor_data.adc[3]

        v_amplified = 3.3 * adc_raw / 4095
        v_diff = (v_amplified - V_OFFSET) / GAIN
        pressure = (v_diff * 1000 * 5.0 / 4.14 - 10.739) / 3.1395
        return pressure

    def start_pump(self):
        """Drive pump. There is supposed to be one pump for the entire system."""
        return self.cfunc_call("pump_switch", True)

    def stop_pump(self):
        """Stop driving pump. There should be one pump for the entire system."""
        return self.cfunc_call("pump_switch", False)

    def open_air_connect_valve(self):
        """Open valve to release air to atmosphere"""
        return self.cfunc_call("valve_switch", True)

    def close_air_connect_valve(self):
        """Close valve to shut off air from atmosphere"""
        return self.cfunc_call("valve_switch", False)

    def gpio_mode(self, board_idx):
        """Return current GPIO mode of air relay board

        0b: 1 0 0 1 GPIO3 GPIO2 GPIO1 GPIO0
        """
        all_gpio_data = self.read_gpio_cstruct(board_idx)
        gpio_mode = all_gpio_data.mode
        return gpio_mode

    def open_work_valve(self, board_idx):
        """Open work valve

        Set GPIO0 to 1
        """
        gpio_mode = self.gpio_mode(board_idx)
        command = gpio_mode | 0b00000001
        return self.cfunc_call("gpio_cmd", board_idx, command)

    def close_work_valve(self, board_idx):
        """Close work valve

        Set GPIO0 to 0
        """
        gpio_mode = self.gpio_mode(board_idx)
        command = gpio_mode & 0b11111110
        return self.cfunc_call("gpio_cmd", board_idx, command)

    def open_relay_valve(self, board_idx):
        """Open valve to relay air to next work

        Set GPIO1 to 1
        """
        gpio_mode = self.gpio_mode(board_idx)
        command = gpio_mode | 0b00000010
        return self.cfunc_call("gpio_cmd", board_idx, command)

    def close_relay_valve(self, board_idx):
        """Close valve to relay air to next work

        Set GPIO1 to 0
        """
        gpio_mode = self.gpio_mode(board_idx)
        command = gpio_mode & 0b11111101
        return self.cfunc_call("gpio_cmd", board_idx, command)

    @property
    def servo_id_to_worm_id(self):
        return self._servo_id_to_worm_id

    @property
    def worm_id_to_servo_id(self):
        return self._worm_id_to_servo_id

    @property
    def joint_to_actuator_matrix(self):
        if self._joint_to_actuator_matrix is None:
            servo_ids = self.search_servo_ids()
            servo_length = len(servo_ids)
            self._joint_to_actuator_matrix = np.zeros(
                (servo_length + 1, servo_length + 1), dtype=np.float32
            )
            self._joint_to_actuator_matrix[:, servo_length] = 7500
            self._joint_to_actuator_matrix[servo_length, servo_length] = 1
            for i in range(servo_length):
                self._joint_to_actuator_matrix[i, i] = deg_to_servovector
        return self._joint_to_actuator_matrix

    @property
    def actuator_to_joint_matrix(self):
        if self._actuator_to_joint_matrix is None:
            self._actuator_to_joint_matrix = np.linalg.inv(self.joint_to_actuator_matrix)
        return self._actuator_to_joint_matrix

    @property
    def armh7_address(self):
        if self._armh7_address is not None:
            return self._armh7_address
        self._armh7_address = dict.fromkeys(armh7_variable_list, "")
        version = self.get_version()
        with open(kondoh7_elf(version), "rb") as stream:
            elf = ELFFile(stream)
            for section in elf.iter_sections():
                if isinstance(section, SymbolTableSection):
                    for symbol in section.iter_symbols():
                        if symbol.name in self._armh7_address:
                            address = symbol.entry["st_value"]
                            self._armh7_address[symbol.name] = address
        return self._armh7_address

    def servo_states(self):
        servo_on_indices = np.where(self.reference_angle_vector() != ServoOnOffValues.OFF.value)[0]
        if len(servo_on_indices) > 0:
            servo_on_ids = self.servo_sorted_ids[servo_on_indices]
            # The worm module is always determined to be in the servo-off
            # state because it is erroneously recognized
            # as being in the servo-on state.
            if (
                self._servo_id_to_worm_id is not None
                and len(self._servo_id_to_worm_id) > 0
            ):
                mask = np.isin(servo_on_ids, list(self._servo_id_to_worm_id))
                servo_on_ids = servo_on_ids[~mask]
            return servo_on_ids
        return []

    def switch_reading_servo_current(self, enable=True):
        if enable:
            value = 1
        else:
            value = 0
        return self.write_cstruct_slot_v(SystemStruct, "servo_current_read_flag", [value])

    def read_servo_current(self):
        servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.zeros(shape=0)

        current_vector = self.read_cstruct_slot_vector(ServoStruct, "current")
        servo_current_vector = np.array(current_vector[servo_ids], dtype=np.float32)

        # Create a mask for non-zero values (i.e., values that are valid and need processing)
        inverted_servo_mask = servo_current_vector >= 64
        inverted_zero_servo_mask = servo_current_vector == 64
        if np.any(inverted_zero_servo_mask):
            servo_current_vector[inverted_zero_servo_mask] += 0.000001
        servo_current_vector[inverted_servo_mask] -= 64

        # Apply calculations only on non-zero values
        estimated_current_vector = servo_current_vector / 10.0
        estimated_current_vector[inverted_servo_mask] *= -1.0
        return estimated_current_vector

    def read_current_limit(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        current_vector = [
            self.servo_param64(sid, ["current_limit"])["current_limit"]
            for sid in servo_ids
        ]
        return current_vector / 10.0

    def switch_reading_servo_temperature(self, enable=True):
        if enable:
            value = 1
        else:
            value = 0
        return self.write_cstruct_slot_v(
            SystemStruct, "servo_temperature_read_flag", [value]
        )

    def read_servo_temperature(self):
        servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.zeros(shape=0)

        temperature_vector = self.read_cstruct_slot_vector(ServoStruct, "temperature")
        servo_temperatures = temperature_vector[servo_ids]

        # Create a mask for non-zero values (i.e., values that are valid and need processing)
        non_zero_mask = servo_temperatures != 0

        # Apply calculations only on non-zero values
        estimated_temperatures = np.zeros_like(servo_temperatures, dtype=np.float32)
        if np.any(non_zero_mask):
            estimated_temperatures[non_zero_mask] = setting_value_to_temperature[servo_temperatures[non_zero_mask]]
        return estimated_temperatures

    def read_temperature_limit(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        temperature_vector = [
            self.servo_param64(sid, ["temperature_limit"])["temperature_limit"]
            for sid in servo_ids
        ]
        return setting_value_to_temperature[temperature_vector]

    def read_ics_baudrate(self, ch="ALL"):
        """Read ICS baudrate configuration for specified channel(s).

        Parameters
        ----------
        ch : str
            ICS channel name ("J1"-"J6" or "ALL")

        Returns
        -------
        int or list
            Baudrate value(s) for the specified channel(s)
        """
        if ch not in ICS_CHANNELS:
            channel_list = list(ICS_CHANNELS.keys())
            raise ValueError(f"Invalid channel. Valid channels: {channel_list}")

        ri_baud_list = self.read_cstruct_slot_vector(SystemStruct, "ics_baudrate").copy()

        # Reverse mapping from value to baudrate
        value_to_baudrate = {v: k for k, v in ICS_BAUDRATES.items()}

        if ch == "ALL":
            results = []
            for channel_name in ["J1", "J2", "J3", "J4", "J5", "J6"]:
                channel_idx = ICS_CHANNELS[channel_name]
                baud_value = ri_baud_list[channel_idx]
                baudrate = value_to_baudrate.get(baud_value, 1250000)  # Default to 1250000
                results.append((channel_name, baudrate))
                print(f"{channel_name}: {baudrate}")
            return results
        else:
            channel_idx = ICS_CHANNELS[ch]
            baud_value = ri_baud_list[channel_idx]
            baudrate = value_to_baudrate.get(baud_value, 1250000)  # Default to 1250000
            print(f"{ch}: {baudrate}")
            return baudrate

    def send_ics_baudrate(self, baud, ch="ALL"):
        """Send ICS baudrate configuration to specified channel(s).

        Parameters
        ----------
        baud : int
            Baudrate value (1250000, 625000, or 115200)
        ch : str
            ICS channel name ("J1"-"J6" or "ALL")

        Returns
        -------
        numpy.ndarray
            Result from write operation
        """
        if baud not in ICS_BAUDRATES:
            baud_list = list(ICS_BAUDRATES.keys())
            raise ValueError(f"Invalid baudrate. Valid baudrates: {baud_list}")

        if ch not in ICS_CHANNELS:
            channel_list = list(ICS_CHANNELS.keys())
            raise ValueError(f"Invalid channel. Valid channels: {channel_list}")

        # Read current values as 2D array (like euslisp)
        ri_baud_list_2d = [self.read_cstruct_slot_vector(SystemStruct, "ics_baudrate").copy()]
        ri_baud_list = ri_baud_list_2d[0]  # Work with the first (and only) array
        baud_value = ICS_BAUDRATES[baud]

        print(f"Before: {ri_baud_list}")
        print(f"Setting baudrate value: {baud_value} for channel: {ch}")

        if ch == "ALL":
            # Set all channels to the same baudrate
            for i in range(6):  # J1-J6 channels
                ri_baud_list[i] = baud_value
        else:
            # Set specific channel
            channel_idx = ICS_CHANNELS[ch]
            ri_baud_list[channel_idx] = baud_value

        print(f"After modification: {ri_baud_list}")

        # Use write_cstruct_slot_v like in euslisp (pass 2D array structure)
        result = self.write_cstruct_slot_v(SystemStruct, "ics_baudrate", ri_baud_list_2d)

        # Verify the write by reading back
        verification = self.read_cstruct_slot_vector(SystemStruct, "ics_baudrate")
        print(f"Verification read: {verification}")

        return result


if __name__ == "__main__":
    interface = ARMH7Interface()
    try:
        print(interface.auto_open())
    except Exception as e:
        if "LIBUSB_ERROR_ACCESS" in str(e):
            # Error already handled and printed by the class method
            pass
        else:
            print(f"Error: {e}")
