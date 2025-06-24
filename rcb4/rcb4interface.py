from enum import Enum
import select
from threading import Lock
import time

import numpy as np
import serial
import serial.tools.list_ports

from rcb4.asm import encode_servo_ids_to_5bytes_bin
from rcb4.asm import encode_servo_positions_to_bytes
from rcb4.asm import encode_servo_velocity_and_position_to_bytes
from rcb4.asm import rcb4_checksum
from rcb4.asm import rcb4_velocity

# 4000.0 / 135
deg_to_servovector = 29.62962962962963


class CommandTypes(Enum):
    Move = 0x00
    Jump = 0x0B
    Call = 0x0C
    SingleServo = 0x0F
    MultiServoSingleVelocity = 0x10
    MultiServoMultiVelocities = 0x11
    ServoParam = 0x12
    Version = 0xFD
    AckCheck = 0xFE
    _None = 0xFF


class ServoOnOffValues(Enum):
    ON = 32767
    OFF = 32768


class ServoParams(Enum):
    Stretch = 0x01
    Speed = 0x02
    CurrentLimit = 0x03
    TemperatureLimit = 0x04


class SubMoveCmd(Enum):
    RamToCom = 0x20
    ComToRam = 0x02
    DeviceToCom = 0x21
    ComToDevice = 0x12


class RamAddr(Enum):
    # System configuration RAM address
    ConfigRamAddress = 0x0000
    # Program counter configuration address
    ProgramCounterRamAddress = 0x0002
    # Address of the value of ADC converter 0
    AdcRamAddress = 0x0022
    # PIO input/output configuration
    PioModeAddress = 0x0038
    # PIO port value
    PioAddress = 0x003A
    # Address where KRR button data is recorded
    KrrButtonDataAddress = 0x0350
    # PA1 data (followed by data in increments of 2 bytes)
    KrrPa1Address = 0x0352
    # (Data continues in increments of 2 bytes)
    CounterRamAddress = 0x0457
    # (Data continues in increments of 2 bytes)
    UserParameterRamAddress = 0x0462


class RomAddr(Enum):
    # Address of the command to play the startup motion
    StartupCmdRomAddress = 0x0444
    # The first address where the main loop starts running
    MainLoopCmd = 0x044B
    # The starting address where motion data is stored
    MotionRomAddress = 0x0B80


class ConfigData(Enum):
    EnableSio = 0x0001  # ICS switch
    EnableRunEeprom = 0x0002  # EEPROM program execution switch
    # Interpolation operation completion message switch
    EnableServoResponse = 0x0004
    EnableReferenceTable = 0x0008  # Vector jump switch
    Frame = 0x0030  # Output cycle register [4:5]
    Baudrate = 0x00C0  # COM baud rate register [6:7] (check for conflicts)
    ZeroFlag = 0x0100  # Zero flag
    CarrayFlag = 0x0200  # Carry flag
    ProgramError = 0x0400  # Program error flag
    RFU = 0x1800  # Reserved for future use [11:12]
    # ICS switch baud rate register [13:14] (check for conflicts)
    IcsBaudrate = 0x6000
    GreenLED = 0x8000  # LED register (green)


rcb4_dof = 36  # servo 35 + 1


class RCB4Interface:
    # The number of data bytes per motion
    MotionSingleDataCount = 2048

    # The maximum number of motions
    MaxMotionCount = 120

    # The maximum total byte count of motion data
    MotionDataCount = 2048 * 120

    def __init__(self):
        self.lock = Lock()
        self.serial = None
        self.servo_sorted_ids = None
        self.wheel_servo_sorted_ids = None
        self._joint_to_actuator_matrix = None
        self._actuator_to_joint_matrix = None

    def __del__(self):
        self.close()

    def open(self, port="/dev/ttyUSB0", baudrate=1250000, timeout=0.01):
        """Opens a serial connection to the RCB4 device.

        Parameters
        ----------
        port : str
            The port name to connect to.
        baudrate : int
            The baud rate for the serial connection.
        timeout : float, optional
            The timeout for the serial connection in seconds (default is 0.01).

        Returns
        -------
        None

        Raises
        ------
        serial.SerialException
            If there is an error opening the serial port.
        """
        try:
            self.serial = serial.Serial(
                port, baudrate, parity="E", stopbits=1, timeout=timeout
            )
            print(f"Opened {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise serial.SerialException(e)
        ack = self.check_ack()
        if ack is not True:
            return False
        self.check_firmware_version()
        self._config_data = self.get_config()
        self.search_servo_ids()
        return True

    def auto_open(self):
        vendor_id = 0x165C
        product_id = 0x0008
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vendor_id and port.pid == product_id:
                return self.open(port=port.device)

    def close(self):
        if self.serial:
            self.serial.close()
        self.serial = None

    def is_opened(self):
        return self.serial is not None

    def serial_write(self, byte_list, timeout=10):
        if self.serial is None:
            raise RuntimeError("Serial is not opened.")

        data_to_send = bytes(byte_list)
        with self.lock:
            try:
                self.serial.flushInput()
                self.serial.write(data_to_send)
            except serial.SerialException as e:
                self.close()
                print(f"Error sending data: {e}")
            ret = self.serial_read(timeout)
        return ret

    def serial_read(self, timeout=10):
        if self.serial is None:
            raise RuntimeError("Serial is not opened.")

        self.serial.flushInput()
        start_time = time.time()
        read_data = b""
        while True:
            ready, _, _ = select.select(
                [self.serial], [], [], timeout - (time.time() - start_time)
            )
            if not ready:
                self.close()
                raise serial.SerialException("Timeout: No data received.")

            chunk = self.serial.read(self.serial.in_waiting or 1)
            if not chunk:
                self.close()
                raise serial.SerialException("Timeout: Incomplete data received.")
            read_data += chunk
            if len(read_data) > 0 and read_data[0] == len(read_data):
                return read_data[1 : len(read_data) - 1]

    def get_version(self):
        byte_list = [0x03, CommandTypes.Version.value, 0x00]
        return self.serial_write(byte_list)

    def get_ack(self):
        byte_list = [0x04, CommandTypes.AckCheck.value, 0x06, 0x08]
        return self.serial_write(byte_list)

    def check_ack(self):
        ack_byte_list = self.get_ack()
        return ack_byte_list[1] == 0x06

    def check_firmware_version(self):
        version = self.get_version()
        if version != b"\xfdCB-4 V1.0      090715          \xc7":
            raise RuntimeError(
                "The firmware version is inconsistent. "
                + "Perhaps are you using not RCB-4mini?"
                + " https://kondo-robot.com/product/rcb4_mini"
            )

    @staticmethod
    def move_ram_to_com_command(scr_addr, src_data_size):
        """Create a command to transfer data from RAM to COM (RAM ==> COM).

        Parameters
        ----------
        scr_addr : int
            The starting address of the data to retrieve.
        src_data_size : int
            The number of bytes of data to retrieve.

        Returns
        -------
        return_data_size : int
            The number of bytes of received data.
        byte_list : list
            The command data array for transmission.

        Notes
        -----
        This method constructs a command to transfer data from RAM to COM.
        The `byte_list` list is initialized and filled with specific values to
        form the command. These include the command type and sub-command
        for RAM to COM transfer, the source address, and the size of the data
        to transfer. The checksum of the command is calculated and appended
        to `byte_list` as the last element. The `returnDataSize` is calculated
        by adding 3 to the `src_data_size`, accounting for additional bytes
        in the response.

        """
        byte_list = []
        byte_list.append(0x0A)
        byte_list.append(CommandTypes.Move.value)
        byte_list.append(SubMoveCmd.RamToCom.value)
        byte_list.append(0x00)
        byte_list.append(0x00)
        byte_list.append(0x00)
        byte_list.append(scr_addr & 0xFF)  # Source address low byte
        byte_list.append((scr_addr >> 8) & 0xFF)  # Source address high byte
        byte_list.append(src_data_size)  # Data size
        byte_list.append(rcb4_checksum(byte_list))
        return_data_size = src_data_size + 3  # Total bytes of received data
        return return_data_size, byte_list

    def write_move_ram_to_com_command(self, src_addr, src_data_size):
        _, byte_list = self.move_ram_to_com_command(src_addr, src_data_size)
        return self.serial_write(byte_list)[1:]

    @staticmethod
    def move_com_to_ram_command(dest_addr, dest_data):
        if len(dest_data) > 249:
            return 0, []
        txbuf = []
        txbuf.append((len(dest_data) + 7) & 0xFF)
        txbuf.append(CommandTypes.Move.value)
        txbuf.append(SubMoveCmd.ComToRam.value)
        txbuf.append(dest_addr & 0xFF)
        txbuf.append((dest_addr >> 8) & 0xFF)
        txbuf.append(0x00)
        if len(dest_data) == 1:
            txbuf.append(dest_data[0])
        else:
            for i in range(len(dest_data)):
                txbuf.append(dest_data[i])
        txbuf.append(rcb4_checksum(txbuf))
        return_data_size = 4  # return ACK
        return return_data_size, txbuf

    def write_move_com_to_ram_command(self, dest_addr, dest_data):
        _, byte_list = self.move_com_to_ram_command(dest_addr, dest_data)
        return self.serial_write(byte_list)[1:]

    def reference_angle_vector(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.zeros(shape=0)
        ref_angles = self._angle_vector("reference")[servo_ids]
        return ref_angles

    def servo_error(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.zeros(shape=0)
        error_angles = self._angle_vector("error")[servo_ids]
        return error_angles

    def servo_id_to_index(self, servo_id):
        if self.valid_servo_ids([servo_id]):
            return self.sequentialized_servo_ids([servo_id])[0]

    def sequentialized_servo_ids(self, servo_ids):
        if len(servo_ids) == 0:
            return np.zeros(shape=0, dtype=np.uint8)
        return self._servo_id_to_sequentialized_servo_id[np.array(servo_ids)].astype(
            np.uint8
        )

    def _angle_vector(self, slot="current"):
        rcb4_dof = 35
        offset = 2
        avs = np.zeros((rcb4_dof, 3))
        vi = 0
        size = 126
        for j in range(rcb4_dof // 7):
            ram = 0x90 + (20 * j * 7) + offset
            while True:
                _, byte_list = self.move_ram_to_com_command(ram, size)
                try:
                    byte_list = self.serial_write(byte_list, timeout=0.1)[1:]
                except serial.SerialException as _:
                    self.auto_open()
                    self.serial.flushInput()
                    continue
                if len(byte_list) == size:
                    break
            seg = np.frombuffer(byte_list, dtype="<u2")
            for i in range(7):
                avs[vi, 0] = seg[i * 10]
                avs[vi, 1] = seg[i * 10 + 1]
                avs[vi, 2] = seg[i * 10 + 2]
                vi += 1
        if slot == "current":
            return avs[:, 1]
        elif slot == "reference":
            return avs[:, 2]
        elif slot == "error":
            return avs[:, 0]
        else:
            raise ValueError(
                'slot should be ["current", "reference", "error"]' f" not {slot}"
            )

    def _send_angle_vector(self, av, servo_ids=None, velocity=127):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(av) != len(servo_ids):
            raise ValueError("Length of servo_ids and angle_vector must be the same.")
        av = np.array(av)
        svs = self.angle_vector_to_servo_angle_vector(av, servo_ids)
        return self.servo_angle_vector(servo_ids, svs, velocity=velocity)

    def angle_vector(self, av=None, servo_ids=None, velocity=127):
        if av is not None:
            return self._send_angle_vector(av, servo_ids, velocity)
        all_servo_ids = self.search_servo_ids()
        if len(all_servo_ids) == 0:
            return np.zeros(shape=0)
        av = np.append(self._angle_vector()[all_servo_ids], 1)
        av = np.matmul(av.T, self.actuator_to_joint_matrix.T)[:-1]
        if servo_ids is not None:
            if len(servo_ids) == 0:
                return np.zeros(shape=0)
            av = av[self.sequentialized_servo_ids(servo_ids)]
        return av

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

    def search_servo_ids(self):
        if self.servo_sorted_ids is not None:
            return self.servo_sorted_ids
        av = self._angle_vector()
        servo_indices = np.where(av > 0)
        if len(servo_indices) > 0:
            servo_indices = servo_indices[0]
        else:
            servo_indices = []

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
        return servo_indices

    def valid_servo_ids(self, servo_ids):
        return np.isfinite(self._servo_id_to_sequentialized_servo_id[np.array(servo_ids)])

    def hold(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [ServoOnOffValues.ON.value] * len(servo_ids)
        return self.servo_angle_vector(servo_ids, servo_vector, velocity=127)

    def free(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [ServoOnOffValues.OFF.value] * len(servo_ids)
        return self.servo_angle_vector(servo_ids, servo_vector, velocity=127)

    def neutral(self, servo_ids=None, velocity=127):
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
            Velocity for the servo movement, clamped between 1 and 255.
            Default value is 127.

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

        # Update servo on/off states based on 32767 (Servo ON) and 32768 (Servo OFF) values in servo_vector
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

    def set_servo_parameters_command(
        self, values, servo_ids=None, servo_param_type="stretch"
    ):
        """Generate a command to modify the parameters of servo motors.

        Parameters
        ----------
        values : list or array-like
            The new parameter values to be set for the servo motors.
        servo_ids : list or array-like, optional
            The IDs of the servo motors whose parameters are to be changed.
            If not provided, the method searches for servo IDs automatically.
        servo_param_type : str, default 'stretch'
            The type of parameter to be changed. Can be 'stretch' for stretch
            adjustment or 'speed' for speed adjustment.

        Returns
        -------
        tuple
            A tuple containing:
            - return_data_size (int): The size of the data expected to
              be returned.
            - byte_list (list): A list of bytes forming the command to send to
              the servos.

        Raises
        ------
        ValueError
            If the lengths of `servo_ids` and `values` do not match, or if an
            invalid `servo_param_type` is provided.

        Notes
        -----
        This method generates commands for altering servo motor parameters.
        It is designed to be used as a static method and can be accessed
        externally. The servo parameters can be modified to adjust either
        the stretch or speed of the servo motors based on the
        `servo_param_type` provided.

        Warnings
        --------
        The data in the provided `values` and potentially `servo_ids`
        could be modified. Exercise caution when using mutable objects.
        """
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if not servo_ids:
            return -1, []
        if len(values) != len(servo_ids):
            raise ValueError("Length of `servo_ids` and `values` " "must be the same.")

        # Sort the servo vectors based on servo IDs
        # for consistent command structure
        sorted_indices = np.argsort(servo_ids)
        sorted_servo_ids = np.array(servo_ids)[sorted_indices]
        sorted_values = np.array(values)[sorted_indices]

        byte_list = [CommandTypes.ServoParam.value]
        byte_list.extend(encode_servo_ids_to_5bytes_bin(sorted_servo_ids))

        if servo_param_type == "stretch":
            byte_list.append(ServoParams.Stretch.value)
        elif servo_param_type == "speed":
            byte_list.append(ServoParams.Speed.value)
        else:
            raise ValueError(f"Invalid `servo_param_type`: {servo_param_type}")

        # Clip the values to be within the valid range and convert to
        # unsigned 8-bit integers
        byte_list.extend(np.clip(sorted_values, 1, 127).astype(np.uint8).tolist())

        # Insert the length of the command at the beginning and
        # append the checksum at the end
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))
        return 4, byte_list

    def send_stretch(self, value=127, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        if not isinstance(value, list) or not isinstance(value, tuple):
            value = [value] * len(servo_ids)
        _, byte_list = self.set_servo_parameters_command(value, servo_ids)
        return self.serial_write(byte_list)

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

    def servo_states(self):
        servo_on_indices = np.where(self.reference_angle_vector() != ServoOnOffValues.OFF.value)[0]
        if len(servo_on_indices) > 0:
            servo_on_ids = self.servo_sorted_ids[servo_on_indices]
            return servo_on_ids
        return []

    def get_config(self):
        rxbuf = self.write_move_ram_to_com_command(RamAddr.ConfigRamAddress.value, 2)
        if len(rxbuf) != 2:
            return 0xFFFF
        else:
            return rxbuf[1] * 256 + rxbuf[0]

    def call_command(self, rom_addr):
        """Creates a command to call a specified ROM address.

        This method generates a command array to be sent to a device,
        instructing it to perform a jump to the specified ROM address.
        This function is static and can be accessed externally without
        an instance of the class.

        Parameters
        ----------
        rom_addr : int
            The target ROM address to call.

        Returns
        -------
        tuple
            return_data_size : int
                The size of the data expected to be returned by the command.
            txbuf : list of int
                The list representing the entire command to be sent,
                including the command to call the ROM address, the address
                itself, and the checksum.

        Notes
        -----
        The command created by this function does not check conditions before
        calling the specified address.

        Warnings
        --------
        The function calls the specified address unconditionally.
        """
        buf = [
            0x07,
            CommandTypes.Call.value,
            rom_addr & 0xFF,
            (rom_addr >> 8) & 0xFF,
            (rom_addr >> 16) & 0xFF,
            0x00,
        ]
        buf.append(rcb4_checksum(buf))
        return 4, buf

    def get_motion_play_number(self):
        """Retrieves the number of the motion currently being played.

        Returns
        -------
        int
            - The number of the motion currently being played.
            - 0 if no motion is being played.
            - -1 in case of communication failure.
            - -2 if the playback location is abnormal.

        Notes
        -----
        The current motion number is calculated from the current program
        counter and flags. If a jump to a different motion has occurred
        within a motion, the jumped motion number is returned.
        """
        rx_len, byte_list = self.move_ram_to_com_command(
            RamAddr.ProgramCounterRamAddress.value, 10
        )
        byte_list = self.serial_write(byte_list)[1:]

        if not rx_len or len(byte_list) != 10:
            return -1

        eflfg = sum(byte_list[3:8])
        pcunt = byte_list[0] + (byte_list[1] << 8) + (byte_list[2] << 16)
        mno = int(
            ((pcunt - RomAddr.MotionRomAddress.value) / self.MotionSingleDataCount) + 1
        )
        if eflfg == 0 or pcunt < RomAddr.MotionRomAddress.value:
            return 0
        if not (0 <= mno <= 120):
            return -2
        return mno

    def suspend_motion(self):
        """Suspends the currently playing motion.

        Returns
        -------
        bool
            True if the operation was successful, False otherwise.

        Notes
        -----
        Modifies the config data to suspend the motion.
        Direct manipulation of internal config data is involved,
        hence the initial configuration needs to be read.
        """
        self._config_data &= ~ConfigData.EnableRunEeprom.value
        self._config_data &= ~ConfigData.EnableServoResponse.value
        self._config_data &= ~ConfigData.EnableReferenceTable.value
        self._config_data &= ~ConfigData.EnableSio.value
        txbuf = [self._config_data & 0xFF, (self._config_data >> 8) & 0xFF]
        return self.write_move_com_to_ram_command(RamAddr.ConfigRamAddress.value, txbuf)

    def motion_number_to_address(self, motion_num):
        """Calculates the ROM address for the given motion number.

        Parameters
        ----------
        motion_num : int
            The motion number.

        Returns
        -------
        int
            The ROM address of the specified motion,
            or -1 if the motion address does not exist.

        Notes
        -----
        The address calculation is based on the motion number.
        """
        if 0 < motion_num <= self.MaxMotionCount:
            return (
                motion_num - 1
            ) * self.MotionSingleDataCount + RomAddr.MotionRomAddress.value
        else:
            return -1

    def set_motion_number(self, motion_num):
        """Jumps to the address of the specified motion.

        Parameters
        ----------
        motion_num : int
            The motion number.

        Returns
        -------
        bool
            True if the operation was successful, False otherwise.

        Notes
        -----
        Utilizes the Call command to jump to the ROM address of
        the specified motion.
        """
        if 0 < motion_num <= self.MaxMotionCount:
            _rx_size, buf = self.call_command(self.motion_number_to_address(motion_num))
            return self.serial_write(buf)
        else:
            return False

    def reset_program_counter(self):
        """Resets the program counter.

        Returns
        -------
        bool
            True if the operation was successful, False otherwise.

        Notes
        -----
        This method effectively changes the program counter to its
        initial settings after a reset, including resetting various flags.
        """
        buf = [
            RomAddr.MainLoopCmd.value & 0xFF,
            (RomAddr.MainLoopCmd.value >> 8) & 0xFF,
        ] + [0] * 8
        return self.write_move_com_to_ram_command(
            RamAddr.ProgramCounterRamAddress.value, buf
        )

    def resume_motion(self):
        """Resumes the motion that was suspended.

        Returns
        -------
        bool
            True if the operation was successful, False otherwise.

        Notes
        -----
        Modifies the config data to resume the motion.
        Direct manipulation of internal config data
        (`self._config_data`) is involved.
        """
        self._config_data |= ConfigData.EnableRunEeprom.value
        self._config_data &= ~ConfigData.EnableServoResponse.value
        self._config_data |= ConfigData.EnableReferenceTable.value
        self._config_data |= ConfigData.EnableSio.value
        buf = [self._config_data & 0xFF, (self._config_data >> 8) & 0xFF]
        return self.write_move_com_to_ram_command(RamAddr.ConfigRamAddress.value, buf)

    def play_motion(self, motion_num):
        """Plays the specified motion.

        Parameters
        ----------
        motion_num : int
            The motion number.

        Returns
        -------
        bool
            True if the motion is successfully played, False otherwise.

        Notes
        -----
        The steps to play a motion include suspending the current motion,
        resetting the program counter, jumping to the motion address,
        and then resuming motion.
        """
        if not (0 < motion_num < self.MaxMotionCount):
            return False
        if not self.suspend_motion():
            return False
        if not self.reset_program_counter():
            return False
        if not self.set_motion_number(motion_num):
            return False
        return self.resume_motion()


if __name__ == "__main__":
    interface = RCB4Interface()
    print(interface.auto_open())

    from datetime import datetime

    while True:
        start = datetime.now()
        print(interface.angle_vector())
        end = datetime.now()
        print(end - start)
