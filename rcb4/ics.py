import platform
import sys
import threading
import time

from colorama import Fore
from colorama import Style
import readchar
import serial
import serial.tools.list_ports
import yaml

try:
    from pyftdi.ftdi import Ftdi
    from pyftdi.serialext import serial_for_url
    PYFTDI_AVAILABLE = True
except ImportError:
    PYFTDI_AVAILABLE = False

from rcb4.temperature import get_setting_value_from_temperatures

degree_to_pulse = 29.633


class KeyListener(threading.Thread):
    def __init__(self):
        super().__init__()
        self.key = None
        self.lock = threading.Lock()
        self.running = True  # Add a running flag

    def run(self):
        """Continuously read keys and store the latest key in self.key while running is True."""
        while self.running:
            try:
                key = readchar.readkey()
                if key == 'q':
                    with self.lock:
                        self.key = key
                    break
            except KeyboardInterrupt:
                self.stop()
                break
            with self.lock:
                self.key = key

    def get_key(self):
        """Return the latest key and reset it."""
        with self.lock:
            key = self.key
            self.key = None
        return key

    def stop(self):
        """Stop the listener thread."""
        self.running = False


def load_and_process_yaml(yaml_path):
    with open(yaml_path) as f:
        joint_data = yaml.safe_load(f)
    servo_candidates = []
    servo_id_to_rotation = {}

    for _joint_name, properties in joint_data.get("joint_name_to_servo_id", {}).items():
        candidate_servo_id = properties["id"] // 2
        rotation = False
        if properties.get("type") == "continuous":
            rotation = True
        servo_id_to_rotation[candidate_servo_id] = rotation
        servo_candidates.append(candidate_servo_id)
    servo_candidates = sorted(set(servo_candidates))
    return servo_candidates, servo_id_to_rotation


def format_baud(baud):
    """Convert baud rate to a more readable format."""
    if baud >= 1_000_000:
        return f"{baud / 1_000_000:.2f}M"
    elif baud >= 1_000:
        return f"{baud / 1_000:.0f}k"
    else:
        return str(baud)


class ICSServoController:
    def __init__(self, baudrate=1250000, yaml_path=None, timeout=0.1):
        if baudrate not in [1250000, 625000, 115200]:
            print(f";; baud={baudrate} is wrong.")
            print(";; baud should be one of 1250000, 625000, 115200")
            print("Use default baudrate 1250000")
            baudrate = 1250000
        if yaml_path is not None:
            self.servo_candidates, self.servo_id_to_rotation = load_and_process_yaml(
                yaml_path
            )
        else:
            self.servo_id_to_rotation = None
            self.servo_candidates = list(range(18))
        self.servo_id_index = 0
        self.servo_id = 0
        self.send_angle_pulse = None
        self.selected_index = 0
        self.baudrate = baudrate
        self.timeout = timeout
        self.ics = None
        self._use_pyftdi = False
        self.is_continuous_rotation_mode = None
        self.servo_eeprom_params64 = [
            ("fix-header", [1, 2]),
            ("stretch-gain", [3, 4]),
            ("speed", [5, 6]),
            ("punch", [7, 8]),
            ("dead-band", [9, 10]),
            ("dumping", [11, 12]),
            ("safe-timer", [13, 14]),
            ("mode-flag-b7slave-b4rotation-b3pwm-b1free-b0reverse", [15, 16]),
            ("pulse-max-limit", [17, 18, 19, 20]),
            ("pulse-min-limit", [21, 22, 23, 24]),
            ("fix-dont-change-25", [25, 26]),
            ("ics-baud-rate-10-115200-00-1250000", [27, 28]),
            ("temperature-limit", [29, 30]),
            ("current-limit", [31, 32]),
            ("fix-dont-change-33", [33, 34]),
            ("fix-dont-change-35", [35, 36]),
            ("fix-dont-change-37", [37, 38]),
            ("fix-dont-change-39", [39, 40]),
            ("fix-dont-change-41", [41, 42]),
            ("fix-dont-change-43", [43, 44]),
            ("fix-dont-change-45", [45, 46]),
            ("fix-dont-change-47", [47, 48]),
            ("fix-dont-change-49", [49, 50]),
            ("response", [51, 52]),
            ("user-offset", [53, 54]),
            ("fix-dont-change-55", [55, 56]),
            ("servo-id", [57, 58]),
            ("stretch-1", [59, 60]),
            ("stretch-2", [61, 62]),
            ("stretch-3", [63, 64]),
        ]

    def synchronize(self, tx_data, rx_length, timeout_multiplier=1):
        """Send data and wait for response.

        Parameters
        ----------
        tx_data : bytes or bytearray
            Data to transmit.
        rx_length : int
            Expected number of bytes to receive.
        timeout_multiplier : int, optional
            Multiplier for timeout (e.g., 50 for EEPROM operations).

        Returns
        -------
        bytes or None
            Received data if successful, None if timeout or error.
        """
        if self.ics is None or not self.ics.is_open:
            return None

        # Retry more for slow baudrates (115200 has intermittent issues)
        max_retries = 3 if self.ics.baudrate <= 115200 else 1

        adjusted_timeout = self.timeout * timeout_multiplier
        original_timeout = self.ics.timeout

        for _attempt in range(max_retries):
            self.ics.reset_input_buffer()
            self.ics.write(tx_data)
            self.ics.flush()

            # Wait for TX to complete physically (important for half-duplex ICS)
            # Calculate TX time: (bytes * 10 bits/byte) / baudrate + margin
            # Use actual serial baudrate, not target baudrate
            tx_time = (len(tx_data) * 10) / self.ics.baudrate
            time.sleep(tx_time + 0.002)  # Add 2ms margin

            # Use pyserial's built-in timeout for reliable reading
            self.ics.timeout = adjusted_timeout

            try:
                rx_buffer = self.ics.read(rx_length)
                if len(rx_buffer) >= rx_length:
                    return bytes(rx_buffer)
            finally:
                self.ics.timeout = original_timeout

        return None

    def _open_connection_pyserial(self):
        """Try to open connection using pyserial (Linux)."""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.vid == 0x165C and p.pid == 0x08:
                for baudrate in [115200, 625000, 1250000]:
                    try:
                        if self.ics and self.ics.is_open:
                            self.ics.close()
                        self.ics = serial.Serial(
                            f"/dev/{p.name}",
                            baudrate,
                            timeout=self.timeout,
                            parity=serial.PARITY_EVEN,
                        )
                        current_baudrate = self.baud()
                        if current_baudrate != self.baudrate:
                            self._change_servo_baudrate(self.baudrate)
                            self.ics.close()
                            self.ics = serial.Serial(
                                f"/dev/{p.name}",
                                self.baudrate,
                                timeout=self.timeout,
                                parity=serial.PARITY_EVEN,
                            )
                        try:
                            self.setup_rotation_mode()
                            self.set_speed(127)
                        except Exception:
                            pass
                        return True
                    except (IndexError, OSError):
                        if self.ics and self.ics.is_open:
                            self.ics.close()
                        continue
        return False

    def _open_connection_pyftdi(self):
        """Try to open connection using pyftdi (macOS)."""
        if not PYFTDI_AVAILABLE:
            return False
        try:
            Ftdi.add_custom_vendor(0x165c, 'Kondo')
            Ftdi.add_custom_product(0x165c, 0x0008, 'DUAL USB ADAPTER')
        except ValueError:
            # Already registered
            pass

        for baudrate in [115200, 625000, 1250000]:
            try:
                if self.ics is not None:
                    self.ics.close()
                self.ics = serial_for_url(
                    'ftdi://Kondo:DUAL USB ADAPTER/1',
                    baudrate=baudrate,
                    timeout=self.timeout,
                )
                self.ics.parity = 'E'  # ICS requires even parity
                self._use_pyftdi = True
                current_baudrate = self.baud()
                if current_baudrate != self.baudrate:
                    self._change_servo_baudrate(self.baudrate)
                    self.ics.close()
                    self.ics = serial_for_url(
                        'ftdi://Kondo:DUAL USB ADAPTER/1',
                        baudrate=self.baudrate,
                        timeout=self.timeout,
                    )
                    self.ics.parity = 'E'
                try:
                    self.setup_rotation_mode()
                    self.set_speed(127)
                except Exception:
                    pass
                return True
            except Exception:
                if self.ics is not None:
                    try:
                        self.ics.close()
                    except Exception:
                        pass
                    self.ics = None
                self._use_pyftdi = False
                continue
        return False

    def open_connection(self):
        # Try pyserial first (works on Linux where ftdi_sio driver is loaded)
        if self._open_connection_pyserial():
            return True

        # If pyserial fails, try pyftdi (works on macOS)
        if PYFTDI_AVAILABLE:
            if self._open_connection_pyftdi():
                return True

        # Print error message if all attempts fail
        print(f"{Fore.RED}No USB Found.{Style.RESET_ALL}")
        print(f"{Fore.RED}May Dual USB Adapter is wrong.{Style.RESET_ALL}")
        if platform.system() == 'Darwin' and not PYFTDI_AVAILABLE:
            print(f"{Fore.YELLOW}On macOS, install pyftdi: pip install pyftdi{Style.RESET_ALL}")
        return False

    def _change_servo_baudrate(self, baud, servo_id=None):
        """Change servo's EEPROM baud rate setting without reopening connection."""
        if baud not in [1250000, 625000, 115200]:
            return None
        if servo_id is None:
            servo_id = self.get_servo_id()
        ics_param64, _ = self.read_param(servo_id=servo_id)
        if baud == 1250000:
            ics_param64[27] = 0
        elif baud == 625000:
            ics_param64[27] = 1
        elif baud == 115200:
            ics_param64[27] = 10
        self.set_param(ics_param64, servo_id)
        return baud

    def set_default_eeprom_param(self):
        self.set_param(
            [5, 10, 11, 4, 7, 15, 0, 0, 0, 2,  # 0-9
             1, 14, 15, 10, 0, 6, 2, 12, 14, 12,  # 10-19
             0, 13, 10, 12, 0, 0, 0, 0, 0, 10,  # 20-29
             1, 14, 1, 2, 9, 8, 14, 13, 9, 13,  # 30-39
             6, 14, 9, 11, 10, 12, 9, 5, 0, 14,   # 40-49
             0, 1, 0, 0, 13, 2, 0, 0, 3, 12,  # 50-59
             7, 8, 15, 14],)  # 60-63

    def read_baud(self, servo_id=None):
        _, result = self.read_param(servo_id=servo_id)
        return result["baud"]

    def baud(self, baud=None, servo_id=None):
        if baud is None:
            return self.read_baud()
        if baud not in [1250000, 625000, 115200]:
            print(f";; baud={baud} is wrong.")
            print(";; baud should be one of 1250000, 625000, 115200")
            return None

        if servo_id is None:
            servo_id = self.get_servo_id()

        # Change servo's EEPROM baud setting
        self._change_servo_baudrate(baud, servo_id)

        # Reopen serial with new baudrate
        if self.ics and self.ics.is_open:
            if self._use_pyftdi:
                self.ics.close()
                self.ics = serial_for_url(
                    'ftdi://Kondo:DUAL USB ADAPTER/1',
                    baudrate=baud,
                    timeout=self.timeout,
                )
                self.ics.parity = 'E'
            else:
                port_name = self.ics.port
                self.ics.close()
                self.ics = serial.Serial(
                    port_name,
                    baud,
                    timeout=self.timeout,
                    parity=serial.PARITY_EVEN,
                )
        return self.read_baud(servo_id=servo_id)

    def get_servo_id(self):
        ret = self.synchronize(bytes([0xFF, 0x00, 0x00, 0x00]), 5,
                               timeout_multiplier=3)
        if ret is None or len(ret) < 5:
            raise OSError("Failed to get servo ID: timeout or insufficient data")
        servo_id = ret[4] & 0x1F
        return servo_id

    def set_servo_id(self, servo_id):
        ret = self.synchronize(
            bytes([0xE0 | (0x1F & servo_id), 0x01, 0x01, 0x01]), 5)
        if ret is None or len(ret) < 5:
            raise OSError("Failed to set servo ID: timeout or insufficient data")
        return 0x1F & ret[4]

    def set_speed(self, speed, servo_id=None):
        speed = max(1, min(127, speed))
        if servo_id is None:
            servo_id = self.get_servo_id()
        v = self.synchronize(bytes([0xC0 | (servo_id & 0x1F), 0x02, speed]), 6)
        if v is None or len(v) < 6:
            raise OSError("Failed to set speed: timeout or insufficient data")
        return v[5]

    def get_speed(self, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        v = self.synchronize(bytes([0xA0 | (servo_id & 0x1F), 0x02]), 5)
        if v is None or len(v) < 5:
            raise OSError("Failed to get speed: timeout or insufficient data")
        return v[4]

    def get_stretch(self, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        v = self.synchronize(bytes([0xA0 | (servo_id & 0x1F), 0x01]), 5)
        if v is None or len(v) < 5:
            raise OSError("Failed to get stretch: timeout or insufficient data")
        return v[4]

    def get_current(self, servo_id=None, interpolate=True):
        if servo_id is None:
            servo_id = self.get_servo_id()
        v = self.synchronize(bytes([0xA0 | (servo_id & 0x1F), 0x03]), 5)
        if v is None or len(v) < 5:
            raise OSError("Failed to get current: timeout or insufficient data")
        current = v[4]
        sign = 1
        if current >= 64:
            current -= 64
            sign = -1
        if interpolate:
            return sign * current / 10.0
        return sign * current

    def get_temperature(self, servo_id=None, interpolate=True):
        if servo_id is None:
            servo_id = self.get_servo_id()
        v = self.synchronize(bytes([0xA0 | (servo_id & 0x1F), 0x04]), 5)
        if v is None or len(v) < 5:
            raise OSError("Failed to get temperature: timeout or insufficient data")
        if interpolate:
            return get_setting_value_from_temperatures(v[4])
        return v[4]

    def set_response(self, value, servo_id=None):
        """
        Set the response parameter to the specified value.
        """
        if servo_id is None:
            servo_id = self.get_servo_id()

        # Read the current parameters
        ics_param64, _ = self.read_param(servo_id=servo_id)

        # Update the 'response' field
        indices = [51, 52]  # Indices for the 'response' parameter
        ics_param64[indices[0] - 1] = (value >> 4) & 0x0F  # High 4 bits
        ics_param64[indices[1] - 1] = value & 0x0F         # Low 4 bits

        # Write back the updated parameters
        self.set_param(ics_param64, servo_id=servo_id)

        # Confirm the change
        _, result = self.read_param(servo_id=servo_id)
        print(f"Response set to {result['response']}")

    def reset_servo_position(self):
        self.set_angle(7500)
        print(f"{Fore.YELLOW}Servo reset to zero position.{Fore.RESET}")

    def setup_rotation_mode(self):
        self.is_continuous_rotation_mode = self.read_rotation()
        if self.is_continuous_rotation_mode:
            self.send_angle_pulse = 7500

    def toggle_rotation_mode(self):
        rotation_mode = self.read_rotation()
        self.set_rotation(not rotation_mode)
        rotation_mode = self.read_rotation()
        mode_text = "Enabled" if rotation_mode else "Disabled"
        print(f"{Fore.CYAN}Rotation mode set to {mode_text}{Fore.RESET}")

    def set_free_mode(self):
        free_mode = self.read_free()
        self.set_free(not free_mode)
        time.sleep(0.1)
        free_mode = self.read_free()
        mode_text = "Enabled" if free_mode else "Disabled"
        print(f"{Fore.MAGENTA}Free mode set to {mode_text}{Fore.RESET}")

    def increase_angle(self):
        if self.is_continuous_rotation_mode:
            angle_pulse = self.send_angle_pulse
            angle_pulse = min(11500, angle_pulse + int(degree_to_pulse * 1))
        else:
            angle = self.read_angle()
            angle_pulse = min(11500, angle + degree_to_pulse * 15)
        self.set_angle(angle_pulse)
        print(f"{Fore.BLUE}Angle increased to {angle_pulse}{Fore.RESET}")

    def decrease_angle(self):
        if self.is_continuous_rotation_mode:
            angle_pulse = self.send_angle_pulse
            angle_pulse = max(3500, angle_pulse - int(degree_to_pulse * 1))
        else:
            angle = self.read_angle()
            angle_pulse = max(3500, angle - degree_to_pulse * 15)
        self.set_angle(angle_pulse)
        print(f"{Fore.RED}Angle decreased to {angle_pulse}{Fore.RESET}")

    def increase_speed(self):
        speed = self.get_speed()
        speed = min(speed + 10, 127)
        self.set_speed(speed)
        print(f"{Fore.BLUE}Speed increased to {speed}{Fore.RESET}")

    def decrease_speed(self):
        speed = self.get_speed()
        speed = max(1, speed - 10)
        self.set_speed(speed)
        print(f"{Fore.BLUE}Speed decreased to {speed}{Fore.RESET}")

    def increase_stretch(self):
        stretch = self.get_stretch()
        stretch = min(stretch + 10, 127)
        self.set_stretch(stretch)
        print(f"{Fore.BLUE}Stretch increased to {stretch}{Fore.RESET}")

    def decrease_stretch(self):
        stretch = self.get_stretch()
        stretch = max(1, stretch - 10)
        self.set_stretch(stretch)
        print(f"{Fore.BLUE}Stretch decreased to {stretch}{Fore.RESET}")

    def parse_param64_key_value(self, v):
        alist = {}
        for param in self.servo_eeprom_params64:
            param_name, indices = param[0], param[1]
            alist[param_name] = self._4bit2num(indices, v)

        baud_value = alist.get("ics-baud-rate-10-115200-00-1250000", 0) & 0x0F
        servo_type = alist.get("ics-baud-rate-10-115200-00-1250000", 0) & 0xF0
        baud_rate = {10: 115200, 1: 625000, 0: 1250000}.get(baud_value, None)
        mode_flag_value = alist.get(
            "mode-flag-b7slave-b4rotation-b3pwm-b1free-b0reverse", 0
        )
        alist.update(self.ics_flag_dict(mode_flag_value))
        alist.update({"servo-id": alist.get("servo-id", 0), "baud": baud_rate,
                      "ics-baud-rate-10-115200-00-1250000": baud_value,
                      "custom-servo-type": servo_type})
        # custom-servo-type is original parameter.
        return alist

    def _4bit2num(self, lst, v):
        sum_val = 0
        for i in lst:
            sum_val = (sum_val << 4) + (v[i - 1] & 0x0F)
        return sum_val

    def ics_flag_dict(self, v):
        return {
            "slave": (v & 0xF0) >> 4 & 0x8 == 0x8,
            "rotation": (v & 0xF0) >> 4 & 0x1 == 0x1,
            "free": v & 0xF & 0x2 == 0x2,
            "reverse": v & 0xF & 0x1 == 0x1,
            "serial": v & 0xF & 0x8 == 0x8,
        }

    def set_flag(self, flag_name, value, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        ics_param64, _ = self.read_param(servo_id=servo_id)

        # Calculate byte and bit for manipulation
        byte_idx = 14 if flag_name in ["slave", "rotation"] else 15
        bit_position = 3 if flag_name in ["slave", "serial"] else 0
        if flag_name == 'b2':
            bit_position = 2
        mask = 1 << bit_position

        # Set or clear the bit based on the `value` argument
        if value:
            ics_param64[byte_idx] |= mask  # Set the bit
        else:
            ics_param64[byte_idx] &= ~mask
            # Clear the bit
        # Set updated parameters to the servo
        self.set_param(ics_param64, servo_id=servo_id)

    def set_slave(self, slave=None, servo_id=None):
        return self.set_flag("slave", slave, servo_id=servo_id)

    def set_rotation(self, rotation=None, servo_id=None):
        if rotation:
            self.send_angle_pulse = 7500
        self.set_free(True, servo_id=servo_id)  # free for before mode change.
        return self.set_flag("rotation", rotation, servo_id=servo_id)

    def set_stretch(self, value, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        value = max(1, min(value, 127))
        v = self.synchronize(bytes([0xC0 | servo_id, 0x01, value]), 6)
        if v is None or len(v) < 6:
            raise OSError("Failed to set stretch: timeout or insufficient data")
        return v[2]

    def set_stretch_values(self, stretch_values, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()

        # Read the current parameters
        ics_param64, _ = self.read_param(servo_id=servo_id)

        # Update the 'stretch-1', 'stretch-2', 'stretch-3' fields
        for stretch_key, value in stretch_values.items():
            if stretch_key == "stretch-1":
                indices = [59, 60]
            elif stretch_key == "stretch-2":
                indices = [61, 62]
            elif stretch_key == "stretch-3":
                indices = [63, 64]
            else:
                raise ValueError(f"Unsupported stretch parameter: {stretch_key}")

            # Split the value into two 4-bit chunks and store them in the corresponding indices
            ics_param64[indices[0] - 1] = (value >> 4) & 0x0F  # High 4 bits
            ics_param64[indices[1] - 1] = value & 0x0F         # Low 4 bits

        # Write back the updated parameters
        self.set_param(ics_param64, servo_id=servo_id)

        # Confirm the change
        _, _result = self.read_param(servo_id=servo_id)
        print(f"Stretch parameters set to: {stretch_values}")

    def set_serial(self, serial=None, servo_id=None):
        return self.set_flag("serial", serial, servo_id=servo_id)

    def set_reverse(self, reverse=None, servo_id=None):
        return self.set_flag("reverse", reverse, servo_id=servo_id)

    def set_free(self, free, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        if free:
            v = self.synchronize(
                bytes([0x80 | (0x1F & servo_id), 0, 0]), 6)
            if v is None or len(v) < 6:
                raise OSError("Failed to set free: timeout or insufficient data")
            return ((v[3 + 1] << 7) & 0x3F80) | (v[3 + 2] & 0x007F)
        ics_param64, _ = self.read_param()
        if free is None or free == 0:
            ics_param64[15] = ics_param64[15] & 0xD
        else:
            ics_param64[15] = ics_param64[15] | 0x2
        self.set_param(ics_param64, servo_id)
        if servo_id != self.read_param()[1].get("servo-id"):
            return self.read_free(servo_id)

    def read_free(self, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        _, result = self.read_param()
        return result.get("free", None)

    def read_rotation(self, servo_id=None):
        _, result = self.read_param(servo_id=servo_id)
        self.is_continuous_rotation_mode = result["rotation"]
        return result["rotation"]

    def set_param(self, ics_param64, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        # EEPROM write takes longer, so use larger timeout multiplier
        ret = self.synchronize(
            bytes([0xC0 | servo_id, 0x00] + ics_param64), 68,
            timeout_multiplier=50)
        if ret is None or len(ret) < 68:
            raise OSError("Failed to set param: timeout or insufficient data")
        ret_ics_param64 = ret[4:]
        self.parse_param64_key_value(ret_ics_param64)

    def close_connection(self):
        if self.ics and self.ics.is_open:
            self.ics.close()
        self.ics = None

    def display_status(self):
        options = [
            "Current Servo ID",
            "Angle",
            "Speed",
            "Stretch",
            "Baud Rate",
            "Rotation Mode",
            "Slave Mode",
            "Reverse Mode",
            "Serial Mode",
            "Free",
        ]
        selectable_options = ["Current Servo ID", "Angle", "Speed", "Stretch"]

        key_listener = KeyListener()
        key_listener.daemon = True
        key_listener.start()
        try:
            use_previous_result = False
            previous_servo_id = None
            while key_listener.running:
                if not self.ics or not self.ics.is_open:
                    # Clear the previous output at the start of each loop
                    sys.stdout.write("\033[H\033[J")
                    sys.stdout.flush()
                    print(f"{Fore.RED}Connection is not open.{Style.RESET_ALL}")
                    print(f"{Fore.RED}Please check the following:")
                    print("1. Use Dual USB Adapter (https://kondo-robot.com/product/02116)")
                    print("2. Set the device to ICS mode.")
                    print(f"3. Connect only one servo correctly.{Style.RESET_ALL}")

                    print(
                        f"{Fore.RED}To establish the connection, please execute the following commands in Linux:{Style.RESET_ALL}"
                    )
                    print()
                    print(f"{Fore.RED}  $ sudo su{Style.RESET_ALL}")
                    print(f"{Fore.RED}  modprobe ftdi-sio{Style.RESET_ALL}")
                    print(
                        f"{Fore.RED}  echo 165C 0008 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id{Style.RESET_ALL}"
                    )
                    print(f"{Fore.RED}  exit{Style.RESET_ALL}")
                    try:
                        ret = self.open_connection()
                    except Exception:
                        continue
                    if ret is False:
                        time.sleep(1.0)
                        continue
                # Print servo status
                try:
                    servo_id = self.get_servo_id()
                    if servo_id != previous_servo_id:
                        use_previous_result = False
                    previous_servo_id = servo_id
                    if use_previous_result is False:
                        _, result = self.read_param()
                        print('======================================')
                        sys.stdout.write("\033[H\033[J")
                        sys.stdout.flush()
                        print("--- Servo Status ---")
                        for i, option in enumerate(options):
                            selected = i == self.selected_index
                            if selected:
                                prefix_str = ">> "
                            else:
                                prefix_str = "   "
                            try:
                                print_str = self.get_status(option, result, selected=selected)
                            except Exception as _:
                                print_str = 'No Data'
                            print(f"{prefix_str} {option}: {print_str}")

                        print("----------------------\n")
                        print(
                            "Use ↑↓ to navigate, ←→ to adjust Current Servo ID or Servo Angles"
                        )
                        print(f"Press 'Enter' when Current Servo ID is selected to save the currently highlighted ID in {Fore.GREEN}green{Style.RESET_ALL}.")
                        print("Press 'z' to reset servo position")
                        print(
                            "Press 'r' to toggle rotation mode (enables continuous wheel-like rotation)"
                        )
                        print("Press 'f' to set free mode")
                        print(f"Press 'd' to set default EEPROM parameters {Fore.RED}(WARNING: This action will overwrite the servo's EEPROM).{Style.RESET_ALL}\n")
                        print("'q' to quit.")

                    key = key_listener.get_key()
                    use_previous_result = False

                    # Perform actions based on key
                    if key == "z":
                        self.reset_servo_position()
                    elif key == "r":
                        self.toggle_rotation_mode()
                    elif key == "f":
                        self.set_free_mode()
                    elif key == "q":
                        print("Exiting...")
                        break
                    elif key == "d":
                        print(
                            f"{Fore.RED}WARNING: This will overwrite the servo's EEPROM with default values.{Style.RESET_ALL}"
                        )
                        print("Press 'y' to proceed or any other key to cancel.")
                        while key_listener.running:
                            confirm_key = key_listener.get_key()
                            if confirm_key == "y":
                                print(f"{Fore.RED}Setting default EEPROM parameters...{Style.RESET_ALL}")
                                self.set_default_eeprom_param()
                                break
                            elif confirm_key is not None:
                                print(f"{Fore.YELLOW}Action canceled.{Style.RESET_ALL}")
                                break
                    elif key == readchar.key.UP:
                        self.selected_index = (self.selected_index - 1) % len(
                            selectable_options
                        )
                    elif key == readchar.key.DOWN:
                        self.selected_index = (self.selected_index + 1) % len(
                            selectable_options
                        )
                    elif (
                        key == readchar.key.ENTER
                        and selectable_options[self.selected_index] == "Current Servo ID"
                    ):
                        self.set_servo_id(self.servo_id)
                        time.sleep(0.3)
                        if self.servo_id_to_rotation is not None:
                            self.set_rotation(self.servo_id_to_rotation[self.servo_id])
                            time.sleep(0.3)
                    elif (
                        key == readchar.key.LEFT
                        and selectable_options[self.selected_index] == "Current Servo ID"
                    ):
                        if self.servo_id_index == 0:
                            self.servo_id_index = len(self.servo_candidates) - 1
                        else:
                            self.servo_id_index = max(0, self.servo_id_index - 1)
                        self.servo_id = self.servo_candidates[self.servo_id_index]
                    elif (
                        key == readchar.key.RIGHT
                        and selectable_options[self.selected_index] == "Current Servo ID"
                    ):
                        if self.servo_id_index == len(self.servo_candidates) - 1:
                            self.servo_id_index = 0
                        else:
                            self.servo_id_index = min(
                                len(self.servo_candidates) - 1, self.servo_id_index + 1
                            )
                        self.servo_id = self.servo_candidates[self.servo_id_index]
                    elif (
                        key == readchar.key.LEFT
                        and selectable_options[self.selected_index] == "Angle"
                    ):
                        self.decrease_angle()
                    elif (
                        key == readchar.key.RIGHT
                        and selectable_options[self.selected_index] == "Angle"
                    ):
                        self.increase_angle()
                    elif (
                        key == readchar.key.LEFT
                        and selectable_options[self.selected_index] == "Speed"
                    ):
                        self.decrease_speed()
                    elif (
                        key == readchar.key.RIGHT
                        and selectable_options[self.selected_index] == "Speed"
                    ):
                        self.increase_speed()
                    elif (
                        key == readchar.key.LEFT
                        and selectable_options[self.selected_index] == "Stretch"
                    ):
                        self.decrease_stretch()
                    elif (
                        key == readchar.key.RIGHT
                        and selectable_options[self.selected_index] == "Stretch"
                    ):
                        self.increase_stretch()
                    else:
                        use_previous_result = True
                except Exception as e:
                    print(f'[ERROR] {e}')
                    use_previous_result = False
                    self.close_connection()
                    continue
                # Flush the output to ensure it displays correctly
                sys.stdout.flush()

                # Wait for a short period before updating again
                # time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nDisplay stopped by user.")
        key_listener.stop()

    def get_status(self, option, param=None, selected=False):
        """Return the status based on the selected option."""
        if option == "Current Servo ID":
            if param is not None:
                current_servo_id = param["servo-id"]
            else:
                current_servo_id = self.get_servo_id()

            s = f'{current_servo_id}'
            if selected:
                str = f"{Style.RESET_ALL}["
                for i, servo_id in enumerate(self.servo_candidates):
                    if i == self.servo_id_index:
                        str += f"{Fore.GREEN}{servo_id}{Style.RESET_ALL} "
                    else:
                        str += f"{servo_id} "
                str += "]"
                s += f' -> Next Servo ID: {str}'
            return s
        elif option == "Angle":
            if self.is_continuous_rotation_mode:
                angle = self.send_angle_pulse
            else:
                angle = self.read_angle()
            angle = int((angle - 7500) / degree_to_pulse)
            return f"{angle}"
        elif option == "Stretch":
            if param is not None:
                stretch = param["stretch-gain"] // 2
            else:
                stretch = self.get_stretch()
            return f"{stretch}"
        elif option == "Speed":
            if param is not None:
                speed = param["speed"]
            else:
                speed = self.get_speed()
            return f"{speed}"
        elif option == "Baud Rate":
            if param is not None:
                baudrate = param["baud"]
            else:
                baudrate = self.read_baud()
            return f"{format_baud(baudrate)}bps"
        elif option == "Rotation Mode":
            if param is not None:
                rotation = param["rotation"]
            else:
                rotation = self.read_rotation()
            return f"{Fore.GREEN if rotation else Fore.RED}{'Enabled' if rotation else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Slave Mode":
            if param is not None:
                slave = param["slave"]
            else:
                slave = self.read_slave()
            return f"{Fore.GREEN if slave else Fore.RED}{'Enabled' if slave else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Reverse Mode":
            if param is not None:
                reverse = param["reverse"]
            else:
                reverse = self.read_reverse()
            return f"{Fore.GREEN if reverse else Fore.RED}{'Enabled' if reverse else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Serial Mode":
            if param is not None:
                serial = param["serial"]
            else:
                serial = self.read_serial()
            return f"{Fore.GREEN if serial else Fore.RED}{'Enabled' if serial else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Free":
            if param is not None:
                free = param["free"]
            else:
                free = self.read_free()
            return f"{Fore.GREEN if free else Fore.RED}{'Enabled' if free else 'Disabled'}{Style.RESET_ALL}"

    def read_angle(self, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        v = self.synchronize(bytes([0xA0 | (servo_id & 0x1F), 5]), 6)
        if v is None or len(v) < 6:
            raise OSError("Failed to read angle: timeout or insufficient data")
        angle = ((v[4] & 0x7F) << 7) | (v[5] & 0x7F)
        return angle

    def set_angle(self, v=7500, servo_id=None):
        v = int(v)
        if servo_id is None:
            servo_id = self.get_servo_id()
        self.send_angle_pulse = v
        ret = self.synchronize(
            bytes([0x80 | (servo_id & 0x1F), (v >> 7) & 0x7F, v & 0x7F]), 6)
        if ret is None or len(ret) < 6:
            raise OSError("Failed to set angle: timeout or insufficient data")
        angle = ((ret[4] & 0x7F) << 7) | (ret[5] & 0x7F)
        return angle

    def read_param(self, servo_id=None, max_retries=3):
        if servo_id is None:
            servo_id = self.get_servo_id()
        for _attempt in range(max_retries):
            ret = self.synchronize(
                bytes([0xA0 | servo_id, 0x00]), 68, timeout_multiplier=10)
            if ret is not None and len(ret) >= 68:
                ics_param64 = ret[4:]
                result = self.parse_param64_key_value(list(ics_param64))
                return list(ics_param64), result
            # Small delay before retry (servo may need time to respond)
            time.sleep(0.05)
        raise OSError("Failed to read param: timeout or insufficient data")

    def _4bit2num(self, lst, v):
        sum_val = 0
        for i in lst:
            sum_val = (sum_val << 4) + (v[i - 1] & 0x0F)
        return sum_val


if __name__ == '__main__':
    servo_controller = ICSServoController()
    servo_controller.open_connection()
