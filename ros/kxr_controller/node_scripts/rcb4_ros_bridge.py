#!/usr/bin/env python3

from collections import deque
import os
import os.path as osp
import shlex
import subprocess
import sys
import tempfile
import threading
import xml.etree.ElementTree as ET

import actionlib
from actionlib_msgs.msg import GoalID
from dynamic_reconfigure.server import Server
import geometry_msgs.msg
from kxr_controller.cfg import KXRParameteresConfig as Config
from kxr_controller.msg import AdjustAngleVectorAction
from kxr_controller.msg import AdjustAngleVectorResult
from kxr_controller.msg import PressureControl
from kxr_controller.msg import PressureControlAction
from kxr_controller.msg import PressureControlResult
from kxr_controller.msg import ServoOnOff
from kxr_controller.msg import ServoOnOffAction
from kxr_controller.msg import ServoOnOffResult
from kxr_controller.msg import Stretch
from kxr_controller.msg import StretchAction
from kxr_controller.msg import StretchResult
import numpy as np
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import JointState
import serial
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
import std_msgs.msg
import yaml

from rcb4.armh7interface import ARMH7Interface
from rcb4.rcb4interface import RCB4Interface

np.set_printoptions(precision=0, suppress=True)


def load_yaml(file_path, Loader=yaml.SafeLoader):
    """Load a YAML file into a Python dict.

    Parameters
    ----------
    file_path : str or pathlib.PosixPath
        The path to the YAML file.

    Returns
    -------
    data : dict
        A dict with the loaded yaml data.
    """
    if not osp.exists(str(file_path)):
        raise OSError(f"{file_path!s} not exists")
    with open(osp.expanduser(file_path)) as f:
        data = yaml.load(f, Loader=Loader)
    data = data["joint_name_to_servo_id"]
    joint_name_to_id = {}
    for name in data:
        if isinstance(data[name], int):
            joint_name_to_id[name] = data[name]
        else:
            joint_name_to_id[name] = data[name]["id"]
    return joint_name_to_id, data


def make_urdf_file(joint_name_to_id):
    robot = ET.Element(
        "robot", {"name": "dummy_robot", "xmlns:xi": "http://www.w3.org/2001/XInclude"}
    )

    ET.SubElement(robot, "link", name="base_link")
    previous_link_name = "base_link"

    for joint_name in joint_name_to_id.keys():
        link_name = f"{joint_name}_link"
        ET.SubElement(robot, "link", name=link_name)

        joint = ET.SubElement(robot, "joint", name=joint_name, type="continuous")
        ET.SubElement(joint, "parent", link=previous_link_name)
        ET.SubElement(joint, "child", link=link_name)

        previous_link_name = link_name

    tmp_urdf_file = tempfile.mktemp(suffix=".urdf")
    tree = ET.ElementTree(robot)
    tree.write(tmp_urdf_file, encoding="utf-8", xml_declaration=True)
    return tmp_urdf_file


def run_robot_state_publisher(namespace=None):
    command = f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun'
    command += " robot_state_publisher robot_state_publisher"
    if namespace is not None:
        command += f" _tf_prefix:={namespace}"
    command = shlex.split(command)
    process = subprocess.Popen(command)
    return process


def run_kxr_controller(namespace=None):
    command = f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun'
    command += " kxr_controller kxr_controller"
    command += " __name=:kxr_controller"
    command = shlex.split(command)
    process = subprocess.Popen(command)
    return process


def set_initial_position(positions, namespace=None):
    rospy.set_param(namespace + "/initial_position", positions)


def set_fullbody_controller(joint_names):
    controller_yaml_dict = {
        "type": "position_controllers/JointTrajectoryController",
        "joints": joint_names,
    }
    rospy.set_param("fullbody_controller", controller_yaml_dict)


def set_joint_state_controler():
    rospy.set_param(
        "joint_state_controller",
        {"type": "joint_state_controller/JointStateController", "publish_rate": 10},
    )


def set_robot_description(urdf_path, param_name="robot_description"):
    with open(urdf_path) as f:
        rospy.set_param(param_name, f.read())


class RCB4ROSBridge:
    def __init__(self):
        self._servo_lock = threading.Lock()
        self.proc_controller_spawner = None
        self.proc_robot_state_publisher = None
        self.proc_kxr_controller = None

        servo_config_path = rospy.get_param("~servo_config_path")
        self.joint_name_to_id, servo_infos = load_yaml(servo_config_path)

        r = RobotModel()
        urdf_path = rospy.get_param("~urdf_path", None)
        tmp_urdf = False
        if urdf_path is None:
            urdf_path = make_urdf_file(self.joint_name_to_id)
            tmp_urdf = True
            rospy.loginfo("Use temporary URDF")
        with open(urdf_path) as f:
            with no_mesh_load_mode():
                r.load_urdf_file(f)

        joint_list = [j for j in r.joint_list if j.__class__.__name__ != "FixedJoint"]
        self.joint_names = [j.name for j in joint_list]

        full_namespace = rospy.get_namespace()
        last_slash_pos = full_namespace.rfind("/")
        clean_namespace = full_namespace[:last_slash_pos] if last_slash_pos != 0 else ""
        self.clean_namespace = clean_namespace

        set_robot_description(
            urdf_path, param_name=clean_namespace + "/robot_description"
        )
        if tmp_urdf:
            os.remove(urdf_path)

        set_joint_state_controler()
        self.current_joint_states_pub = rospy.Publisher(
            clean_namespace + "/current_joint_states", JointState, queue_size=1
        )
        # Publish servo state like joint_trajectory_controller
        # https://wiki.ros.org/joint_trajectory_controller#Published_Topics
        self.servo_on_off_pub = rospy.Publisher(
            clean_namespace
            + "/fullbody_controller/servo_on_off_real_interface"
            + "/state",
            ServoOnOff,
            queue_size=1,
        )

        self.interface = None
        while not rospy.is_shutdown():
            try:
                if rospy.get_param("~device", None):
                    self.interface = ARMH7Interface.from_port(
                        rospy.get_param("~device", None)
                    )
                elif rospy.get_param("~use_rcb4"):
                    self.interface = RCB4Interface()
                    self.interface.auto_open()
                else:
                    self.interface = ARMH7Interface.from_port()
                break
            except serial.SerialException as e:
                rospy.logerr(f"Waiting for the port to become available. {e}")
            rospy.sleep(1.0)
        if self.interface is None:
            rospy.logerr("Could not open port!")
            sys.exit(1)
        self._prev_velocity_command = None

        self.srv = Server(Config, self.config_callback)

        # set servo ids to rosparam
        rospy.set_param(
            clean_namespace + "/servo_ids", self.get_ids(type='servo')
        )

        wheel_servo_sorted_ids = []
        trim_vector_servo_ids = []
        trim_vector_offset = []
        for _, info in servo_infos.items():
            if isinstance(info, int):
                continue
            servo_id = info["id"]
            direction = info.get("direction", 1)
            offset = info.get("offset", 0)
            if "type" in info and info["type"] == "continuous":
                wheel_servo_sorted_ids.append(servo_id)
            idx = self.interface.servo_id_to_index(servo_id)
            if idx is None:
                continue
            self.interface._joint_to_actuator_matrix[idx, idx] = (
                direction * self.interface._joint_to_actuator_matrix[idx, idx]
            )
            trim_vector_servo_ids.append(servo_id)
            trim_vector_offset.append(direction * offset)
        if self.interface.__class__.__name__ != "RCB4Interface":
            self.interface.trim_vector(trim_vector_offset, trim_vector_servo_ids)
        if self.interface.wheel_servo_sorted_ids is None:
            self.interface.wheel_servo_sorted_ids = wheel_servo_sorted_ids

        self.set_fullbody_controller(clean_namespace)
        self.set_initial_positions(clean_namespace)
        self.check_servo_states(retry_count=-1)

        rospy.loginfo("run kxr_controller")
        self.proc_kxr_controller = run_kxr_controller(namespace=clean_namespace)

        self.command_joint_state_sub = rospy.Subscriber(
            clean_namespace + "/command_joint_state",
            JointState,
            queue_size=1,
            callback=self.command_joint_state_callback,
        )

        self.velocity_command_joint_state_sub = rospy.Subscriber(
            clean_namespace + "/velocity_command_joint_state",
            JointState,
            queue_size=1,
            callback=self.velocity_command_joint_state_callback,
        )

        self.servo_on_off_server = actionlib.SimpleActionServer(
            clean_namespace + "/fullbody_controller/servo_on_off_real_interface",
            ServoOnOffAction,
            execute_cb=self.servo_on_off_callback,
            auto_start=False,
        )
        # Avoid 'rospy.exceptions.ROSException: publish() to a closed topic'
        rospy.sleep(0.1)
        self.servo_on_off_server.start()

        self.adjust_angle_vector_server = actionlib.SimpleActionServer(
            clean_namespace + "/fullbody_controller/adjust_angle_vector_interface",
            AdjustAngleVectorAction,
            execute_cb=self.adjust_angle_vector_callback,
            auto_start=False,
        )
        # Avoid 'rospy.exceptions.ROSException: publish() to a closed topic'
        rospy.sleep(0.1)
        self.adjust_angle_vector_server.start()
        self.cancel_motion_pub = rospy.Publisher(
            clean_namespace
            + "/fullbody_controller/follow_joint_trajectory"
            + "/cancel",
            GoalID,
            queue_size=1,
        )

        # TODO(someone) support rcb-4 miniboard
        if not rospy.get_param("~use_rcb4"):
            # Stretch
            self.stretch_server = actionlib.SimpleActionServer(
                clean_namespace + "/fullbody_controller/stretch_interface",
                StretchAction,
                execute_cb=self.stretch_callback,
                auto_start=False,
            )
            # Avoid 'rospy.exceptions.ROSException:
            # publish() to a closed topic'
            rospy.sleep(0.1)
            self.stretch_server.start()
            self.stretch_publisher = rospy.Publisher(
                clean_namespace + "/fullbody_controller/stretch",
                Stretch,
                queue_size=1,
                latch=True,
            )
            # Avoid 'rospy.exceptions.ROSException:
            # publish() to a closed topic'
            rospy.sleep(0.1)
            self.publish_stretch()
            # Pressure control
            self.control_pressure = rospy.get_param("~control_pressure", False)
            if self.control_pressure is True:
                self.pressure_control_thread = None
                self.pressure_control_server = actionlib.SimpleActionServer(
                    clean_namespace + "/fullbody_controller/pressure_control_interface",
                    PressureControlAction,
                    execute_cb=self.pressure_control_callback,
                    auto_start=False,
                )
                # Avoid 'rospy.exceptions.ROSException:
                # publish() to a closed topic'
                rospy.sleep(0.1)
                self.pressure_control_server.start()
                # Publish state topic like joint_trajectory_controller
                # https://wiki.ros.org/joint_trajectory_controller#Published_Topics
                self.pressure_control_pub = rospy.Publisher(
                    clean_namespace
                    + "/fullbody_controller/pressure_control_interface"
                    + "/state",
                    PressureControl,
                    queue_size=1,
                )
                self.air_board_ids = self.get_ids(type='air_board')
                rospy.set_param(
                    clean_namespace + "/air_board_ids",
                    self.air_board_ids
                )
                self.pressure_control_state = {}
                for idx in self.air_board_ids:
                    self.pressure_control_state[f"{idx}"] = {}
                    self.pressure_control_state[f"{idx}"]["start_pressure"] = 0
                    self.pressure_control_state[f"{idx}"]["stop_pressure"] = 0
                    self.pressure_control_state[f"{idx}"]["release"] = True
                self._pressure_publisher_dict = {}
                self._avg_pressure_publisher_dict = {}
                # Record 1 seconds pressure data.
                hz = rospy.get_param(self.clean_namespace + "/control_loop_rate", 20)
                self.recent_pressures = deque([], maxlen=1 * int(hz))

        self.proc_controller_spawner = subprocess.Popen(
            [
                f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun',
                "controller_manager",
                "spawner",
            ]
            + ["joint_state_controller", "fullbody_controller"]
        )
        self.proc_robot_state_publisher = run_robot_state_publisher(clean_namespace)

        self.publish_imu = rospy.get_param("~publish_imu", True)
        self.publish_sensor = rospy.get_param("~publish_sensor", False)
        self.publish_battery_voltage = rospy.get_param("~publish_battery_voltage", True)
        if self.interface.__class__.__name__ == "RCB4Interface":
            self.publish_imu = False
            self.publish_sensor = False
        if self.publish_imu:
            self.imu_frame_id = rospy.get_param(
                "~imu_frame_id", clean_namespace + "/" + r.root_link.name
            )
            self.imu_publisher = rospy.Publisher(
                clean_namespace + "/imu", sensor_msgs.msg.Imu, queue_size=1
            )
        if self.publish_sensor:
            self._sensor_publisher_dict = {}
        if self.publish_battery_voltage:
            self.battery_voltage_publisher = rospy.Publisher(
                clean_namespace + "/battery_voltage", std_msgs.msg.Float32, queue_size=1
            )

    def __del__(self):
        if self.proc_controller_spawner:
            self.proc_controller_spawner.kill()
        if self.proc_robot_state_publisher:
            self.proc_robot_state_publisher.kill()
        if self.proc_kxr_controller:
            self.proc_kxr_controller.kill()

    def unsubscribe(self):
        self.command_joint_state_sub.unregister()
        self.velocity_command_joint_state_sub.unregister()

    def config_callback(self, config, level):
        self.frame_count = config.frame_count
        self.wheel_frame_count = config.wheel_frame_count
        return config

    def get_ids(self, type='servo', retry_count=-1):
        attempts = 0
        ids = []
        while retry_count == -1 or attempts < retry_count:
            try:
                if type == 'servo' :
                    ids = self.interface.search_servo_ids()
                elif type == 'air_board':
                    ids = self.interface.search_air_board_ids()
                ids = ids.tolist()
                break
            except serial.serialutil.SerialException:
                rospy.logerr(f"[get_ids] Failed to retrieve {type} ids.")
                rospy.sleep(0.1)
                attempts += 1
                continue
            except Exception as e:
                rospy.logerr(f"[get_ids] Unexpected error: {e}")
                rospy.sleep(0.1)
                attempts += 1
                continue
        return ids

    def check_servo_states(self, retry_count=1):
        self.joint_servo_on = {jn: False for jn in self.joint_names}
        attempts = 0
        while retry_count == -1 or attempts < retry_count:
            try:
                servo_on_states = self.interface.servo_states()
                break
            except serial.serialutil.SerialException as e:
                rospy.logerr(f"[check_servo_states] Failed to retrieve servo states. {e}")
                rospy.sleep(0.1)
                attempts += 1
                continue
            except Exception as e:
                rospy.logerr(f"[check_servo_states] Unexpected error: {e}")
                rospy.sleep(0.1)
                attempts += 1
                continue
        if retry_count != -1 and attempts >= retry_count:
            rospy.logerr(
                "[check_servo_states] Failed to retrieve servo states after maximum retries."
            )
            return
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            idx = self.joint_name_to_id[jn]
            if idx in servo_on_states:
                self.joint_servo_on[jn] = True
            else:
                self.joint_servo_on[jn] = False

    def set_fullbody_controller(self, clean_namespace):
        self.fullbody_jointnames = []
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            servo_id = self.joint_name_to_id[jn]
            if servo_id in self.interface.wheel_servo_sorted_ids:
                continue
            self.fullbody_jointnames.append(jn)
        set_fullbody_controller(self.fullbody_jointnames)

    def set_initial_positions(self, clean_namespace):
        initial_positions = {}
        while True:
            try:
                init_av = self.interface.angle_vector()
                break
            except serial.serialutil.SerialException:
                rospy.logerr("[set_initial_positions] Failed to retrieve initial_positions.")
                rospy.sleep(0.1)
                continue
            except Exception as e:
                rospy.logerr(f"[set_initial_positions] Unexpected error: {e}")
                rospy.sleep(0.1)
                continue
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            servo_id = self.joint_name_to_id[jn]
            if servo_id in self.interface.wheel_servo_sorted_ids:
                continue
            idx = self.interface.servo_id_to_index(servo_id)
            if idx is None:
                continue
            initial_positions[jn] = float(np.deg2rad(init_av[idx]))
        set_initial_position(initial_positions, namespace=clean_namespace)

    def _msg_to_angle_vector_and_servo_ids(self, msg, velocity_control=False):
        used_servo_id = {}
        servo_ids = []
        angle_vector = []
        for name, angle in zip(msg.name, msg.position):
            if name not in self.joint_name_to_id or (
                name in self.joint_servo_on and not self.joint_servo_on[name]
            ):
                continue
            idx = self.joint_name_to_id[name]
            if velocity_control:
                if idx not in self.interface.wheel_servo_sorted_ids:
                    continue
            else:
                if idx in self.interface.wheel_servo_sorted_ids:
                    continue
            # should ignore duplicated index.
            if idx in used_servo_id:
                continue
            used_servo_id[idx] = True
            angle_vector.append(np.rad2deg(angle))
            servo_ids.append(idx)
        angle_vector = np.array(angle_vector)
        servo_ids = np.array(servo_ids, dtype=np.int32)
        valid_indices = self.interface.valid_servo_ids(servo_ids)
        return angle_vector[valid_indices], servo_ids[valid_indices]

    def velocity_command_joint_state_callback(self, msg):
        av, servo_ids = self._msg_to_angle_vector_and_servo_ids(
            msg, velocity_control=True
        )
        if len(av) == 0:
            return
        if self._prev_velocity_command is not None and np.allclose(
            self._prev_velocity_command, av
        ):
            return
        try:
            with self._servo_lock:
                self.interface.angle_vector(
                    av, servo_ids, velocity=self.wheel_frame_count)
            self._prev_velocity_command = av
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[velocity_command_joint] {e!s}")

    def command_joint_state_callback(self, msg):
        av, servo_ids = self._msg_to_angle_vector_and_servo_ids(
            msg, velocity_control=False
        )
        if len(av) == 0:
            return
        try:
            with self._servo_lock:
                self.interface.angle_vector(av, servo_ids, velocity=self.frame_count)
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[command_joint] {e!s}")

    def servo_on_off_callback(self, goal):
        servo_vector = []
        servo_ids = []
        for joint_name, servo_on in zip(goal.joint_names, goal.servo_on_states):
            if joint_name not in self.joint_name_to_id:
                continue
            servo_ids.append(self.joint_name_to_id[joint_name])
            if servo_on:
                servo_vector.append(32767)
            else:
                servo_vector.append(32768)
        max_retry_count = 10
        for retry_count in range(max_retry_count):
            try:
                with self._servo_lock:
                    self.interface.servo_angle_vector(servo_ids, servo_vector, velocity=1)
                    rospy.sleep(0.1)
                break
            except RuntimeError as e:
                self.unsubscribe()
                rospy.signal_shutdown(f"Disconnected {e}.")
            except serial.serialutil.SerialException as e:
                rospy.logerr(f"[servo_on_off] {e!s} Try {retry_count}/{max_retry_count}")
        return self.servo_on_off_server.set_succeeded(ServoOnOffResult())

    def adjust_angle_vector_callback(self, goal):
        servo_ids = []
        error_thresholds = []
        for joint_name, error_threshold in zip(goal.joint_names, goal.error_threshold):
            if joint_name not in self.joint_name_to_id:
                continue
            servo_ids.append(self.joint_name_to_id[joint_name])
            error_thresholds.append(error_threshold)
        try:
            adjust = self.interface.adjust_angle_vector(
                servo_ids=servo_ids,
                error_threshold=np.array(error_thresholds, dtype=np.float32),
            )
            # If adjustment occurs, cancel motion via follow joint trajectory
            if adjust is True:
                self.cancel_motion_pub.publish(GoalID())
                rospy.logwarn("Stop motion by sending follow joint trajectory cancel.")
        except RuntimeError as e:
            self.unsubscribe()
            rospy.signal_shutdown(f"Disconnected {e}.")
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[adjust_angle_vector] {e!s}")
        return self.adjust_angle_vector_server.set_succeeded(AdjustAngleVectorResult())

    def publish_stretch(self):
        # Get current stretch of all servo motors and publish them
        joint_names = []
        servo_ids = []
        for joint_name in self.joint_names:
            if joint_name not in self.joint_name_to_id:
                continue
            joint_names.append(joint_name)
            servo_ids.append(self.joint_name_to_id[joint_name])
        try:
            stretch = self.interface.read_stretch(servo_ids=servo_ids)
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[read_stretch] {e!s}")
            return
        stretch_msg = Stretch(joint_names=joint_names, stretch=stretch)
        self.stretch_publisher.publish(stretch_msg)

    def stretch_callback(self, goal):
        if len(goal.joint_names) == 0:
            goal.joint_names = self.joint_names
        joint_names = []
        servo_ids = []
        for joint_name in goal.joint_names:
            if joint_name not in self.joint_name_to_id:
                continue
            joint_names.append(joint_name)
            servo_ids.append(self.joint_name_to_id[joint_name])
        # Send new stretch
        stretch = goal.stretch
        try:
            self.interface.send_stretch(value=stretch, servo_ids=servo_ids)
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[send_stretch] {e!s}")
        # Return result
        self.publish_stretch()
        rospy.loginfo(f"Update {joint_names} stretch to {stretch}")
        return self.stretch_server.set_succeeded(StretchResult())

    def publish_pressure(self):
        for idx in self.air_board_ids:
            try:
                key = f"{idx}"
                if key not in self._pressure_publisher_dict:
                    self._pressure_publisher_dict[key] = rospy.Publisher(
                        self.clean_namespace + "/fullbody_controller/pressure/" + key,
                        std_msgs.msg.Float32,
                        queue_size=1,
                    )
                    self._avg_pressure_publisher_dict[key] = rospy.Publisher(
                        self.clean_namespace
                        + "/fullbody_controller/average_pressure/"
                        + key,
                        std_msgs.msg.Float32,
                        queue_size=1,
                    )
                    # Avoid 'rospy.exceptions.ROSException:
                    # publish() to a closed topic'
                    rospy.sleep(0.1)
                pressure = self.read_pressure_sensor(idx)
                self._pressure_publisher_dict[key].publish(
                    std_msgs.msg.Float32(data=pressure)
                )
                # Publish average pressure (noise removed pressure)
                self._avg_pressure_publisher_dict[key].publish(
                    std_msgs.msg.Float32(data=self.average_pressure)
                )
            except serial.serialutil.SerialException as e:
                rospy.logerr(f"[publish_pressure] {e!s}")

    def publish_pressure_control(self):
        for idx in list(self.pressure_control_state.keys()):
            idx = int(idx)
            msg = PressureControl()
            msg.board_idx = idx
            msg.start_pressure = self.pressure_control_state[f"{idx}"]["start_pressure"]
            msg.stop_pressure = self.pressure_control_state[f"{idx}"]["stop_pressure"]
            msg.release = self.pressure_control_state[f"{idx}"]["release"]
            self.pressure_control_pub.publish(msg)

    def pressure_control_loop(self, idx, start_pressure, stop_pressure, release):
        self.pressure_control_state[f"{idx}"]["start_pressure"] = start_pressure
        self.pressure_control_state[f"{idx}"]["stop_pressure"] = stop_pressure
        self.pressure_control_state[f"{idx}"]["release"] = release
        if self.pressure_control_running is False:
            return
        if release is True:
            self.release_vacuum(idx)
            self.pressure_control_running = False
            return
        vacuum_on = False
        while self.pressure_control_running:
            if vacuum_on is False and self.average_pressure > start_pressure:
                self.start_vacuum(idx)
                vacuum_on = True
            if vacuum_on and self.average_pressure <= stop_pressure:
                self.stop_vacuum(idx)
                vacuum_on = False
            rospy.sleep(0.1)

    def read_pressure_sensor(self, idx, force=False):
        while True:
            try:
                pressure = self.interface.read_pressure_sensor(idx)
                self.recent_pressures.append(pressure)
                return pressure
            except serial.serialutil.SerialException as e:
                rospy.logerr(f"[read_pressure_sensor] {e!s}")
                if force is True:
                    continue

    @property
    def average_pressure(self):
        return sum(self.recent_pressures) / len(self.recent_pressures)

    def release_vacuum(self, idx):
        """Connect work to air.

        After 1s, all valves are closed and pump is stopped.
        """
        try:
            self.interface.stop_pump()
            self.interface.open_work_valve(idx)
            self.interface.open_air_connect_valve()
            rospy.sleep(1)  # Wait until air is completely released
            self.interface.close_air_connect_valve()
            self.interface.close_work_valve(idx)
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[release_vacuum] {e!s}")

    def start_vacuum(self, idx):
        """Vacuum air in work"""
        try:
            self.interface.start_pump()
            self.interface.open_work_valve(idx)
            self.interface.close_air_connect_valve()
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[start_vacuum] {e!s}")

    def stop_vacuum(self, idx):
        """Seal air in work"""
        try:
            self.interface.close_work_valve(idx)
            self.interface.close_air_connect_valve()
            rospy.sleep(0.3)  # Wait for valve to close completely
            self.interface.stop_pump()
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[stop_vacuum] {e!s}")

    def pressure_control_callback(self, goal):
        if self.pressure_control_thread is not None:
            # Finish existing thread
            self.pressure_control_running = False
            # Wait for the finishing process complete
            while self.pressure_control_thread.is_alive() is True:
                rospy.sleep(0.1)
        # Set new thread
        idx = goal.board_idx
        start_pressure = goal.start_pressure
        stop_pressure = goal.stop_pressure
        release = goal.release
        self.pressure_control_running = True
        self.pressure_control_thread = threading.Thread(
            target=self.pressure_control_loop,
            args=(
                idx,
                start_pressure,
                stop_pressure,
                release,
            ),
            daemon=True,
        )
        self.pressure_control_thread.start()
        return self.pressure_control_server.set_succeeded(PressureControlResult())

    def publish_imu_message(self):
        msg = sensor_msgs.msg.Imu()
        msg.header.frame_id = self.imu_frame_id
        msg.header.stamp = rospy.Time.now()
        try:
            q_wxyz, acc, gyro = self.interface.read_imu_data()
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[publish_imu] {e!s}")
            return
        (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z) = (
            q_wxyz
        )
        (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) = gyro
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = acc
        self.imu_publisher.publish(msg)

    def publish_sensor_values(self):
        stamp = rospy.Time.now()
        msg = geometry_msgs.msg.WrenchStamped()
        msg.header.stamp = stamp
        try:
            sensors = self.interface.all_jointbase_sensors()
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[publish_sensor_values] {e!s}")
            return
        for sensor in sensors:
            for i in range(4):
                for typ in ["proximity", "force"]:
                    key = f"kjs_{sensor.id}_{typ}_{i}"
                    if typ == "proximity":
                        msg.wrench.force.x = sensor.ps[i]
                    elif typ == "force":
                        msg.wrench.force.x = sensor.adc[i]
                    if key not in self._sensor_publisher_dict:
                        self._sensor_publisher_dict[key] = rospy.Publisher(
                            self.clean_namespace + f"/kjs/{sensor.id}/{typ}/{i}",
                            geometry_msgs.msg.WrenchStamped,
                            queue_size=1,
                        )
                        # Avoid 'rospy.exceptions.ROSException:
                        # publish() to a closed topic'
                        rospy.sleep(0.1)
                    msg.header.frame_id = f"kjs_{sensor.id}_{i}_frame"
                    self._sensor_publisher_dict[key].publish(msg)

    def publish_battery_voltage_value(self):
        try:
            volt = self.interface.battery_voltage()
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[publish_battery_voltage] {e!s}")
            return
        self.battery_voltage_publisher.publish(std_msgs.msg.Float32(data=volt))

    def publish_joint_states(self):
        try:
            av = self.interface.angle_vector()
            torque_vector = self.interface.servo_error()
        except IndexError as e:
            rospy.logerr(f"[publish_joint_states] {e!s}")
            return
        except ValueError as e:
            rospy.logerr(f"[publish_joint_states] {e!s}")
            return
        except serial.serialutil.SerialException as e:
            rospy.logerr(f"[publish_joint_states] {e!s}")
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for name in self.joint_names:
            if name in self.joint_name_to_id:
                servo_id = self.joint_name_to_id[name]
                idx = self.interface.servo_id_to_index(servo_id)
                if idx is None:
                    continue
                msg.position.append(np.deg2rad(av[idx]))
                msg.effort.append(torque_vector[idx])
                msg.name.append(name)
        self.current_joint_states_pub.publish(msg)

    def publish_servo_on_off(self):
        self.check_servo_states()
        servo_on_off_msg = ServoOnOff()
        servo_on_off_msg.joint_names = list(self.joint_servo_on.keys())
        servo_on_off_msg.servo_on_states = list(
            self.joint_servo_on.values())
        self.servo_on_off_pub.publish(servo_on_off_msg)

    def run(self):
        rate = rospy.Rate(
            rospy.get_param(self.clean_namespace + "/control_loop_rate", 20)
        )

        while not rospy.is_shutdown():
            if self.interface.is_opened() is False:
                self.unsubscribe()
                rospy.signal_shutdown("Disconnected.")
                break
            self.publish_joint_states()
            self.publish_servo_on_off()

            if self.publish_imu and self.imu_publisher.get_num_connections():
                self.publish_imu_message()
            if self.publish_sensor:
                self.publish_sensor_values()
            if self.publish_battery_voltage:
                self.publish_battery_voltage_value()
            if rospy.get_param("~use_rcb4") is False and self.control_pressure:
                self.publish_pressure()
                self.publish_pressure_control()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rcb4_ros_bridge")
    ros_bridge = RCB4ROSBridge()
    ros_bridge.run()
