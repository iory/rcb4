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

from actionlib_msgs.msg import GoalID
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import geometry_msgs.msg
from kxr_controller.module_loader import KXRConfigLoader
from kxr_controller.module_loader import ModuleLoader
from kxr_controller.msg import AdjustAngleVectorAction
from kxr_controller.msg import AdjustAngleVectorResult
from kxr_controller.msg import PressureControl
from kxr_controller.msg import PressureControlAction
from kxr_controller.msg import PressureControlResult
from kxr_controller.msg import ServoOnOff
from kxr_controller.msg import ServoOnOffAction
from kxr_controller.msg import ServoOnOffResult
from kxr_controller.msg import ServoState
from kxr_controller.msg import ServoStateArray
from kxr_controller.msg import Stretch
from kxr_controller.msg import StretchAction
from kxr_controller.msg import StretchResult
from kxr_controller.multi_lock import MultiLock
from kxr_controller.serial import serial_call_with_retry
import numpy as np
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import JointState
import serial
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

from rcb4.armh7interface import ARMH7Interface
from rcb4.rcb4interface import RCB4Interface
from rcb4.rcb4interface import ServoOnOffValues

# async load heavy modules
server_loader = ModuleLoader('dynamic_reconfigure.server', 'Server')
actionlib_loader = ModuleLoader('actionlib')
kxr_config_loader = KXRConfigLoader()

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
        ET.SubElement(joint, "limit", velocity="7.47998", effort="0.656248")
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


def set_joint_state_controller(hz=10):
    rospy.set_param(
        "joint_state_controller",
        {"type": "joint_state_controller/JointStateController", "publish_rate": hz},
    )


def set_robot_description(urdf_path, param_name="robot_description"):
    with open(urdf_path) as f:
        rospy.set_param(param_name, f.read())


class RCB4ROSBridge:
    def __init__(self):
        self._update_temperature_limit = False
        self._update_current_limit = False

        self._during_servo_off = False
        # Set up configuration paths and parameters
        self.setup_paths_and_params()

        # Set up URDF and Robot Model
        self.robot_model, self.joint_names = self.setup_urdf_and_model()

        while not rospy.is_shutdown():
            ret = self.setup_interface_and_servo_parameters()
            if ret is True:
                break

        # Set up ROS parameters and controllers
        self.setup_ros_parameters()
        self.run_ros_robot_controllers()

        self.setup_publishers_and_servers()

        self.subscribe()
        rospy.loginfo("RCB4 ROS Bridge initialization completed.")

    def setup_paths_and_params(self):
        self.proc_controller_spawner = None
        self.proc_robot_state_publisher = None
        self.proc_kxr_controller = None
        self.servo_config_path = rospy.get_param("~servo_config_path")
        self.joint_name_to_id, self.servo_infos = load_yaml(self.servo_config_path)
        self.urdf_path = rospy.get_param("~urdf_path", None)
        self.use_rcb4 = rospy.get_param("~use_rcb4", False)
        self.control_pressure = rospy.get_param("~control_pressure", False)
        self.read_temperature = rospy.get_param("~read_temperature", False) and not self.use_rcb4
        self.read_current = rospy.get_param("~read_current", False) and not self.use_rcb4
        self.base_namespace = self.get_base_namespace()
        self.use_fullbody_controller = rospy.get_param("~use_fullbody_controller", True)
        if self.use_fullbody_controller is False:
            self.default_controller = rospy.get_param(self.base_namespace + "/default_controller", [])
        else:
            self.default_controller = ['fullbody_controller']

    def setup_urdf_and_model(self):
        robot_model = RobotModel()
        rospy.loginfo(f"[setup_urdf_and_model] Loading URDF File. {self.urdf_path}")
        if self.urdf_path is None:
            self.urdf_path = make_urdf_file(self.joint_name_to_id)
            rospy.loginfo("Use temporary URDF")
        with open(self.urdf_path) as f:
            with no_mesh_load_mode():
                robot_model.load_urdf_file(f)
        joint_list = [
            j for j in robot_model.joint_list if j.__class__.__name__ != "FixedJoint"
        ]
        joint_names = [j.name for j in joint_list]
        set_robot_description(
            self.urdf_path, param_name=self.base_namespace + "/robot_description"
        )
        if self.urdf_path is None:
            os.remove(self.urdf_path)
        return robot_model, joint_names

    def setup_ros_parameters(self):
        """Configure joint state and full body controllers."""
        self.hz = rospy.get_param(self.base_namespace + "/control_loop_rate", 30)
        set_joint_state_controller(self.hz)
        self.set_fullbody_controller()

    def setup_publishers_and_servers(self):
        """Set up ROS publishers and action servers."""
        self.current_joint_states_pub = rospy.Publisher(
            self.base_namespace + "/current_joint_states", JointState, queue_size=1
        )

        self.servo_states_pub = rospy.Publisher(
            self.base_namespace + "/servo_states", ServoStateArray, queue_size=1
        )

        # Publish servo state like joint_trajectory_controller
        # https://wiki.ros.org/joint_trajectory_controller#Published_Topics
        self.servo_on_off_pub = rospy.Publisher(
            self.base_namespace
            + "/fullbody_controller/servo_on_off_real_interface/state",
            ServoOnOff,
            queue_size=1,
        )

        self.cancel_motion_pub = rospy.Publisher(
            self.base_namespace
            + "/fullbody_controller/follow_joint_trajectory"
            + "/cancel",
            GoalID,
            queue_size=1,
        )

        self.publish_imu = rospy.get_param("~publish_imu", True) and not self.use_rcb4
        self.publish_sensor = (
            rospy.get_param("~publish_sensor", False) and not self.use_rcb4
        )
        self.publish_battery_voltage = (
            rospy.get_param("~publish_battery_voltage", True) and not self.use_rcb4
        )

        if self.publish_imu:
            self.imu_frame_id = rospy.get_param(
                "~imu_frame_id",
                self.base_namespace + "/" + self.robot_model.root_link.name,
            )
            self.imu_publisher = rospy.Publisher(
                self.base_namespace + "/imu", sensor_msgs.msg.Imu, queue_size=1
            )
        if self.publish_sensor:
            self._sensor_publisher_dict = {}
        if self.publish_battery_voltage:
            self.battery_voltage_publisher = rospy.Publisher(
                self.base_namespace + "/battery_voltage",
                std_msgs.msg.Float32,
                queue_size=1,
            )

        # Action servers for servo control
        self.setup_action_servers_and_clients()
        Server = server_loader.get_module()
        Config = kxr_config_loader.get_module()
        self.srv = Server(Config, self.config_callback)

    def setup_action_servers_and_clients(self):
        """Set up action servers for controlling servos and pressure."""

        # Servo on/off action server
        actionlib = actionlib_loader.get_module()
        self.servo_on_off_server = actionlib.SimpleActionServer(
            self.base_namespace + "/fullbody_controller/servo_on_off_real_interface",
            ServoOnOffAction,
            execute_cb=self.servo_on_off_callback,
            auto_start=False,
        )
        # Avoid 'rospy.exceptions.ROSException: publish() to a closed topic'
        rospy.sleep(0.1)
        self.servo_on_off_server.start()

        self.traj_action_clients = []
        self.traj_action_joint_names = []
        for controller in self.default_controller:
            controller_name = self.base_namespace + f"/{controller}/follow_joint_trajectory"
            traj_action_client = actionlib.SimpleActionClient(
                controller_name,
                FollowJointTrajectoryAction,
            )
            self.traj_action_joint_names.append(
                rospy.get_param(self.base_namespace + f"/{controller}/joints", []))
            rospy.loginfo(f'Waiting {controller_name}')
            traj_action_client.wait_for_server()
            self.traj_action_clients.append(traj_action_client)

        # Adjust angle vector action server
        self.adjust_angle_vector_server = actionlib.SimpleActionServer(
            self.base_namespace + "/fullbody_controller/adjust_angle_vector_interface",
            AdjustAngleVectorAction,
            execute_cb=self.adjust_angle_vector_callback,
            auto_start=False,
        )
        # Avoid 'rospy.exceptions.ROSException: publish() to a closed topic'
        rospy.sleep(0.1)
        self.adjust_angle_vector_server.start()

        # Additional action servers based on configuration
        if not self.use_rcb4:
            self.setup_stretch_and_pressure_control_servers()

    def setup_stretch_and_pressure_control_servers(self):
        """Configure stretch and pressure control action servers if enabled."""
        default_stretch = 30
        rospy.logwarn(f"Set default stretch values {default_stretch} for all servos")
        serial_call_with_retry(self.interface.send_stretch, value=default_stretch)
        actionlib = actionlib_loader.get_module()
        self.stretch_server = actionlib.SimpleActionServer(
            self.base_namespace + "/fullbody_controller/stretch_interface",
            StretchAction,
            execute_cb=self.stretch_callback,
            auto_start=False,
        )
        rospy.sleep(0.1)
        self.stretch_server.start()

        self.stretch_publisher = rospy.Publisher(
            self.base_namespace + "/fullbody_controller/stretch",
            Stretch,
            queue_size=1,
            latch=True,
        )
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            rospy.loginfo("Try to publish stretch values.")
            ret = self.publish_stretch()
            if ret is True:
                break
            rospy.sleep(0.1)

        if self.control_pressure:
            self.pressure_control_server = actionlib.SimpleActionServer(
                self.base_namespace + "/fullbody_controller/pressure_control_interface",
                PressureControlAction,
                execute_cb=self.pressure_control_callback,
                auto_start=False,
            )
            rospy.sleep(0.1)
            self.pressure_control_thread = {}
            self.pressure_control_running = {}
            self.pump_on_lock = MultiLock(lock_name="Pump ON")
            self.pump_off_lock = MultiLock(lock_name="Pump OFF")
            self.air_connect_lock = MultiLock(lock_name="Air Connect")
            self.air_disconnect_lock = MultiLock(lock_name="Air Disconnect")
            self.pressure_control_server.start()

            self.pressure_control_pub = rospy.Publisher(
                self.base_namespace
                + "/fullbody_controller/pressure_control_interface"
                + "/state",
                PressureControl,
                queue_size=1,
            )

            rospy.set_param(self.base_namespace + "/air_board_ids", self.air_board_ids)
            self.pressure_control_state = {}
            for idx in self.air_board_ids:
                self.pressure_control_state[f"{idx}"] = {}
                self.pressure_control_state[f"{idx}"]["trigger_pressure"] = 0
                self.pressure_control_state[f"{idx}"]["target_pressure"] = 0
                # At first, air is assumed to be released
                self.pressure_control_state[f"{idx}"]["release_duration"] = 1
            self._pressure_publisher_dict = {}
            self._avg_pressure_publisher_dict = {}
            # Record 1 seconds pressure data.
            self.recent_pressures = {}
            self.history_pressures = {}

    def setup_interface_and_servo_parameters(self):
        self.interface = self.setup_interface()

        def log_error_and_close_interface(function_name):
            msg = f"Failed to {function_name}. "
            msg += "Control board is switch off or cable is disconnected?"
            rospy.logerr(msg)
            self.interface.close()
            return False

        if self.read_current:
            serial_call_with_retry(self.interface.switch_reading_servo_current, enable=True, max_retries=3)
        if self.read_temperature:
            serial_call_with_retry(self.interface.switch_reading_servo_temperature, enable=True, max_retries=3)

        wheel_servo_sorted_ids = []
        trim_vector_servo_ids = []
        trim_vector_offset = []
        for _, info in self.servo_infos.items():
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
        self.interface._actuator_to_joint_matrix = np.linalg.inv(self.interface.joint_to_actuator_matrix)
        if self.interface.__class__.__name__ != "RCB4Interface":
            if len(trim_vector_offset) > 0:
                ret = serial_call_with_retry(
                    self.interface.trim_vector,
                    trim_vector_offset,
                    trim_vector_servo_ids,
                    max_retries=10,
                )
                if ret is None:
                    return log_error_and_close_interface("set trim_vector")
        if self.interface.wheel_servo_sorted_ids is None:
            self.interface.wheel_servo_sorted_ids = []
        self.interface.wheel_servo_sorted_ids = list(
            set(self.interface.wheel_servo_sorted_ids + wheel_servo_sorted_ids))

        # set servo ids to rosparam
        servo_ids = self.get_ids(type="servo")
        if servo_ids is None:
            return log_error_and_close_interface("get initial servo ids")
        rospy.set_param(self.base_namespace + "/servo_ids", servo_ids)
        ret = self.set_initial_positions()
        if ret is False:
            return log_error_and_close_interface("get initial angle vector")
        if self.control_pressure:
            self.air_board_ids = self.get_ids(type="air_board")
            if self.air_board_ids is None:
                return log_error_and_close_interface("get air board ids")
        return True

    def run_ros_robot_controllers(self):
        controllers = ["joint_state_controller"]
        if self.use_fullbody_controller:
            controllers.append('fullbody_controller')
        else:
            controllers.extend(self.default_controller)
        self.proc_controller_spawner = subprocess.Popen(
            [
                f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun',
                "controller_manager",
                "spawner",
            ]
            + controllers,
        )
        self.proc_robot_state_publisher = run_robot_state_publisher(self.base_namespace)
        self.proc_kxr_controller = run_kxr_controller(namespace=self.base_namespace)

    def setup_interface(self):
        rospy.loginfo("Try to connect servo control boards.")
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for the port to become available")
            try:
                if rospy.get_param("~device", None):
                    return ARMH7Interface.from_port(rospy.get_param("~device"))
                if self.use_rcb4:
                    interface = RCB4Interface()
                    ret = interface.auto_open()
                    if ret is True:
                        return interface
                interface = ARMH7Interface()
                ret = interface.auto_open()
                if ret is True:
                    return interface
            except serial.SerialException as e:
                rospy.logerr(f"Waiting for the port to become available: {e}")
            rospy.sleep(1.0)
        rospy.logerr("Could not open port!")
        sys.exit(1)

    def get_base_namespace(self):
        """Return the clean namespace for the node."""
        full_namespace = rospy.get_namespace()
        last_slash_pos = full_namespace.rfind("/")
        return full_namespace[:last_slash_pos] if last_slash_pos != 0 else ""

    def __del__(self):
        self.unsubscribe()
        if self.proc_controller_spawner:
            self.proc_controller_spawner.kill()
        if self.proc_robot_state_publisher:
            self.proc_robot_state_publisher.kill()
        if self.proc_kxr_controller:
            self.proc_kxr_controller.kill()

    def subscribe(self):
        self.command_joint_state_sub = rospy.Subscriber(
            self.base_namespace + "/command_joint_state",
            JointState,
            queue_size=1,
            callback=self.command_joint_state_callback,
        )
        self.velocity_command_joint_state_sub = rospy.Subscriber(
            self.base_namespace + "/velocity_command_joint_state",
            JointState,
            queue_size=1,
            callback=self.velocity_command_joint_state_callback,
        )

    def unsubscribe(self):
        self.command_joint_state_sub.unregister()
        self.velocity_command_joint_state_sub.unregister()

    def config_callback(self, config, level):
        self.frame_count = config.frame_count
        self.wheel_frame_count = config.wheel_frame_count
        self.current_limit = config.current_limit
        self.temperature_limit = config.temperature_limit
        self._update_temperature_limit = True
        self._update_current_limit = True
        return config

    def get_ids(self, type="servo", max_retries=10):
        if type == "servo":
            ids = serial_call_with_retry(
                self.interface.search_servo_ids, max_retries=max_retries
            )
        elif type == "air_board":
            ids = serial_call_with_retry(
                self.interface.search_air_board_ids, max_retries=max_retries
            )
        if ids is None:
            return
        ids = ids.tolist()
        return ids

    def set_fullbody_controller(self):
        self.fullbody_jointnames = []
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            servo_id = self.joint_name_to_id[jn]
            if servo_id in self.interface.wheel_servo_sorted_ids:
                continue
            self.fullbody_jointnames.append(jn)
        if len(self.fullbody_jointnames) == 0:
            rospy.logwarn("No joint names provided for fullbody controller.")
            rospy.logwarn(f"You should check servo config file {self.servo_config_path} and URDF's joint names {self.joint_names}")
        set_fullbody_controller(self.fullbody_jointnames)

    def set_initial_positions(self):
        initial_positions = {}
        init_av = serial_call_with_retry(self.interface.angle_vector, max_retries=10)
        if init_av is None:
            return False
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
        set_initial_position(initial_positions, namespace=self.base_namespace)
        return True

    def _valid_joint(self, joint_name):
        return joint_name in self.joint_name_to_id and (
            self.joint_name_to_id[joint_name] in self.interface.servo_on_states_dict
            and self.interface.servo_on_states_dict[self.joint_name_to_id[joint_name]]
        )

    def _msg_to_angle_vector_and_servo_ids(self, msg, velocity_control=False):
        used_servo_id = {}
        servo_ids = []
        angle_vector = []
        for name, angle in zip(msg.name, msg.position):
            if not self._valid_joint(name):
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
        if not self.interface.is_opened() or self._during_servo_off:
            return
        av, servo_ids = self._msg_to_angle_vector_and_servo_ids(
            msg, velocity_control=True
        )
        if len(av) == 0:
            return
        if not hasattr(self, "_prev_velocity_command"):
            self._prev_velocity_command = None
        if self._prev_velocity_command is not None and np.allclose(
            self._prev_velocity_command, av
        ):
            return
        ret = serial_call_with_retry(
            self.interface.angle_vector, av, servo_ids,
            velocity=self.wheel_frame_count)
        if ret is None:
            return
        self._prev_velocity_command = av

    def command_joint_state_callback(self, msg):
        if not self.interface.is_opened() or self._during_servo_off:
            return
        av, servo_ids = self._msg_to_angle_vector_and_servo_ids(
            msg, velocity_control=False
        )
        if len(av) == 0:
            return
        serial_call_with_retry(
            self.interface.angle_vector, av, servo_ids, velocity=self.frame_count
        )

    def servo_on_off_callback(self, goal):
        if not self.interface.is_opened():
            return self.servo_on_off_server.set_aborted(
                text="Failed to call servo on off. "
                + "Control board is switch off or cable is disconnected?"
            )

        # Publish current joint position to follow_joint_trajectory
        # to prevent sudden movements.
        self.cancel_motion_pub.publish(GoalID())  # Ensure no active motion
        rospy.sleep(0.1)  # Slight delay for smooth transition

        servo_vector = []
        servo_ids = []
        for joint_name, servo_on in zip(goal.joint_names, goal.servo_on_states):
            if joint_name not in self.joint_name_to_id:
                continue
            servo_ids.append(self.joint_name_to_id[joint_name])
            if servo_on:
                servo_vector.append(ServoOnOffValues.ON.value)
            else:
                servo_vector.append(ServoOnOffValues.OFF.value)

        self._during_servo_off = True
        ret = serial_call_with_retry(
            self.interface.servo_angle_vector,
            servo_ids,
            servo_vector,
            velocity=1,
            max_retries=10,
        )
        if ret is None:
            self._during_servo_off = False
            return self.servo_on_off_server.set_aborted(
                text="Failed to call servo on off. "
                + "Control board is switch off or cable is disconnected?"
            )

        av = serial_call_with_retry(self.interface.angle_vector,
                                    max_retries=10)
        if av is None:
            return self.servo_on_off_server.set_aborted(
                text="Failed to call servo on off. "
                + "Control board is switch off or cable is disconnected?"
            )
        for client, joint_name_list in zip(self.traj_action_clients, self.traj_action_joint_names):
            joint_names = []
            positions = []
            for joint_name in joint_name_list:
                angle = 0
                if joint_name in self.joint_name_to_id:
                    servo_id = self.joint_name_to_id[joint_name]
                    idx = self.interface.servo_id_to_index(servo_id)
                    if idx is not None:
                        angle = np.deg2rad(av[idx])
                joint_names.append(joint_name)
                positions.append(angle)
            # Create JointTrajectoryGoal to set current position on follow_joint_trajectory
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.trajectory.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = rospy.Duration(
                0.5
            )  # Short duration for immediate application
            trajectory_goal.trajectory.points = [point]
            # Initialize action client and wait for server
            client.send_goal(trajectory_goal)
        for client in self.traj_action_clients:
            client.wait_for_result()  # Wait for trajectory to complete
        self._during_servo_off = False
        return self.servo_on_off_server.set_succeeded(ServoOnOffResult())

    def adjust_angle_vector_callback(self, goal):
        if not self.interface.is_opened():
            return self.adjust_angle_vector_server.set_aborted(
                text="Failed to adjust angle vector"
            )
        servo_ids = []
        error_thresholds = []
        for joint_name, error_threshold in zip(goal.joint_names, goal.error_threshold):
            if not self._valid_joint(joint_name):
                continue
            servo_ids.append(self.joint_name_to_id[joint_name])
            error_thresholds.append(error_threshold)
        adjust = serial_call_with_retry(
            self.interface.adjust_angle_vector,
            servo_ids=servo_ids,
            error_threshold=np.array(error_thresholds, dtype=np.float32),
        )
        if adjust is None:
            return self.adjust_angle_vector_server.set_aborted(
                text="Failed to adjust angle vector"
            )
        # If adjustment occurs, cancel motion via follow joint trajectory
        if adjust is True:
            self.cancel_motion_pub.publish(GoalID())
            rospy.logwarn("Stop motion by sending follow joint trajectory cancel.")
        return self.adjust_angle_vector_server.set_succeeded(AdjustAngleVectorResult())

    def publish_stretch(self):
        if not self.interface.is_opened():
            return False
        # Get current stretch of all servo motors and publish them
        joint_names = []
        servo_ids = []
        for joint_name in self.joint_names:
            if joint_name not in self.joint_name_to_id:
                continue
            joint_names.append(joint_name)
            servo_ids.append(self.joint_name_to_id[joint_name])
        stretch = serial_call_with_retry(self.interface.read_stretch)
        if stretch is None:
            return False
        stretch_msg = Stretch(joint_names=joint_names, stretch=stretch)
        self.stretch_publisher.publish(stretch_msg)
        return True

    def stretch_callback(self, goal):
        if not self.interface.is_opened():
            return self.stretch_server.set_aborted(text="Failed to update stretch")
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
        success = serial_call_with_retry(
            self.interface.send_stretch, value=stretch, servo_ids=servo_ids,
            max_retries=3)
        if success is None:
            text = "Failed to update stretch"
            rospy.logwarn(text)
            return self.stretch_server.set_aborted(text=text)
        # need to wait for stretch value update.
        rospy.sleep(1.0)
        self.publish_stretch()
        rospy.loginfo(f"Update {joint_names} stretch to {stretch}")
        return self.stretch_server.set_succeeded(StretchResult())

    def publish_pressure(self):
        if not self.interface.is_opened():
            return False
        success = True
        for idx in self.air_board_ids:
            key = f"{idx}"
            if key not in self._pressure_publisher_dict:
                self._pressure_publisher_dict[key] = rospy.Publisher(
                    self.base_namespace + "/fullbody_controller/pressure/" + key,
                    std_msgs.msg.Float32,
                    queue_size=1,
                )
                self._avg_pressure_publisher_dict[key] = rospy.Publisher(
                    self.base_namespace + "/fullbody_controller/average_pressure/" + key,
                    std_msgs.msg.Float32,
                    queue_size=1,
                )
                # Avoid 'rospy.exceptions.ROSException:
                # publish() to a closed topic'
                rospy.sleep(0.1)
            pressure = serial_call_with_retry(self.interface.read_pressure_sensor, idx)
            if pressure is None:
                success = False
                continue
            self._pressure_publisher_dict[key].publish(
                std_msgs.msg.Float32(data=pressure)
            )
            if idx not in self.recent_pressures:
                self.recent_pressures[idx] = deque(maxlen=1 * int(self.hz))
            if self.is_outlier_pressure(idx, pressure):
                return
            self.recent_pressures[idx].append(pressure)
            # Publish average pressure (noise removed pressure)
            self._avg_pressure_publisher_dict[key].publish(
                std_msgs.msg.Float32(data=self.average_pressure(idx))
            )
        return success

    def publish_pressure_control(self):
        for idx in list(self.pressure_control_state.keys()):
            idx = int(idx)
            msg = PressureControl()
            msg.board_idx = idx
            msg.trigger_pressure = self.pressure_control_state[f"{idx}"]["trigger_pressure"]
            msg.target_pressure = self.pressure_control_state[f"{idx}"]["target_pressure"]
            msg.release_duration = self.pressure_control_state[f"{idx}"]["release_duration"]
            self.pressure_control_pub.publish(msg)

    def pressure_control_loop(self, idx, trigger_pressure, target_pressure, release_duration):
        self.pressure_control_state[f"{idx}"]["trigger_pressure"] = trigger_pressure
        self.pressure_control_state[f"{idx}"]["target_pressure"] = target_pressure
        self.pressure_control_state[f"{idx}"]["release_duration"] = release_duration
        if self.pressure_control_running[idx] is False:
            return
        if release_duration > 0:
            self.air_disconnect_lock.release(idx)
            self.pump_on_lock.release(idx)
            rospy.loginfo(f"[Release air work] id: {idx}")
            self.release_air_work(idx, release_duration)
            self.pressure_control_running[idx] = False
            return
        air_work_on = self.air_work_status(idx)
        while self.pressure_control_running[idx]:
            pressure = self.average_pressure(idx)
            if pressure is None:
                rospy.sleep(0.1)
                continue
            is_depressurizing = trigger_pressure > target_pressure
            is_pressurizing = trigger_pressure < target_pressure
            depressure_check_start = is_depressurizing and (pressure > trigger_pressure)
            depressure_check_stop = is_depressurizing and (pressure <= target_pressure)
            pressure_check_start = is_pressurizing and (pressure < trigger_pressure)
            pressure_check_stop = is_pressurizing and (pressure >= target_pressure)
            if air_work_on is False and (depressure_check_start or pressure_check_start):
                self.air_disconnect_lock.acquire(idx)
                self.pump_on_lock.acquire(idx)
                rospy.loginfo(f"[Start air work] id: {idx}")
                air_work_on = self.start_air_work(idx)
            if air_work_on and (depressure_check_stop or pressure_check_stop):
                self.air_disconnect_lock.release(idx)
                self.pump_on_lock.release(idx)
                rospy.loginfo(f"[Stop air work] id: {idx}")
                air_work_on = not self.stop_air_work(idx)
            rospy.sleep(0.1)

    def average_pressure(self, idx):
        n = len(self.recent_pressures[idx])
        if n == 0:
            return None
        return sum(self.recent_pressures[idx]) / n

    def is_outlier_pressure(self, idx, pressure):
        """Detects outliers by comparing input pressure to the history_pressures.
        Example
        ------
        >>> history_pressures[idx] = [7.61, 7.91, 6.57, 7.61, 7.24, 7.03, 6.86,
                                      6.86, 7.28, 5.81, 6.52, 7.53, 6.94, 7.19,
                                      6.94, 7.28, 7.53, 6.69, 6.94, 3.68]
        >>> pressure = -48.2
        """
        if idx not in self.history_pressures:
            self.history_pressures[idx] = deque(maxlen=20)
        if len(self.history_pressures[idx]) < 20:
            self.history_pressures[idx].append(pressure)
            return False
        median = np.median(self.history_pressures[idx])
        std_dev = np.std(self.history_pressures[idx])
        # Detect outlier
        if abs(pressure - median) <= 16 * std_dev:
            self.history_pressures[idx].append(pressure)
            return False
        # Consider consecutive outliers as valid data"
        if np.all(np.abs(np.array(self.history_pressures[idx])[-3:] - pressure) < std_dev * 4):
            self.history_pressures[idx].append(pressure)
            return False
        else:
            rospy.loginfo(f"[Outlier pressure] {idx}: {pressure}kPa")
            self.history_pressures[idx].append(pressure)
            return True

    def stop_pump(self):
        """Stop pump when all threads do not need pump-on

        """
        self.pump_on_lock.wait_for_all_released()
        return self.interface.stop_pump()

    def start_pump(self):
        """Start pump when all threads do not need pump-off

        """
        self.pump_off_lock.wait_for_all_released()
        return self.interface.start_pump()

    def open_air_connect_valve(self):
        """Open air connect valve when all threads do not need valve-close

        """
        self.air_disconnect_lock.wait_for_all_released()
        return self.interface.open_air_connect_valve()

    def close_air_connect_valve(self):
        """Close air connect valve when all threads do not need valve-open

        """
        self.air_connect_lock.wait_for_all_released()
        return self.interface.close_air_connect_valve()

    def release_air_work(self, idx, release_duration):
        """Connect work to air.

        After release_duration[s], all valves are closed and pump is stopped.
        """
        if not self.interface.is_opened():
            return False
        ret = serial_call_with_retry(self.stop_pump, max_retries=3)
        if ret is None:
            return False
        ret = serial_call_with_retry(self.interface.open_work_valve, idx, max_retries=3)
        if ret is None:
            return False
        self.air_connect_lock.acquire(idx)
        ret = serial_call_with_retry(self.open_air_connect_valve,
                                     max_retries=3)
        if ret is None:
            return False
        rospy.sleep(release_duration)  # Wait until air is completely released
        self.air_connect_lock.release(idx)
        ret = serial_call_with_retry(self.close_air_connect_valve,
                                     max_retries=3)
        if ret is None:
            return False
        ret = serial_call_with_retry(self.interface.close_work_valve,
                                     idx, max_retries=3)
        if ret is None:
            return False
        return True

    def start_air_work(self, idx):
        """Start air work"""
        if not self.interface.is_opened():
            return False

        ret = serial_call_with_retry(self.start_pump, max_retries=3)
        if ret is None:
            return False
        ret = serial_call_with_retry(self.interface.open_work_valve, idx, max_retries=3)
        if ret is None:
            return False
        ret = serial_call_with_retry(self.close_air_connect_valve, max_retries=3)
        if ret is None:
            return False
        return True

    def stop_air_work(self, idx):
        """Stop air work"""
        if not self.interface.is_opened():
            return False

        ret = serial_call_with_retry(self.interface.close_work_valve, idx, max_retries=3)
        if ret is None:
            return False
        ret = serial_call_with_retry(self.close_air_connect_valve, max_retries=3)
        if ret is None:
            return False
        rospy.sleep(0.3)  # Wait for valve to close completely
        ret = serial_call_with_retry(self.stop_pump, max_retries=3)
        if ret is None:
            return False
        return True

    def air_work_status(self, idx):
        """Returns whether or not the pump is working on air work.

        air_disconnect_lock and pump_on_lock are applied when start_air_work()
        and are removed when stop_air_work().
        """
        return idx in self.air_disconnect_lock.active_lock_idx()\
            and idx in self.pump_on_lock.active_lock_idx()

    def pressure_control_callback(self, goal):
        if not self.interface.is_opened():
            return
        idx = goal.board_idx
        if idx in self.pressure_control_thread:
            # Finish existing thread
            self.pressure_control_running[idx] = False
            # Wait for the finishing process complete
            while self.pressure_control_thread[idx].is_alive() is True:
                rospy.sleep(0.1)
        # Set new thread
        trigger_pressure = goal.trigger_pressure
        target_pressure = goal.target_pressure
        release_duration = goal.release_duration
        self.pressure_control_running[idx] = True
        self.pressure_control_thread[idx] = threading.Thread(
            target=self.pressure_control_loop,
            args=(
                idx,
                trigger_pressure,
                target_pressure,
                release_duration,
            ),
            daemon=True,
        )
        self.pressure_control_thread[idx].start()
        return self.pressure_control_server.set_succeeded(PressureControlResult())

    def publish_imu_message(self):
        if self.publish_imu is False or self.imu_publisher.get_num_connections() == 0:
            return
        if not self.interface.is_opened():
            return
        msg = sensor_msgs.msg.Imu()
        msg.header.frame_id = self.imu_frame_id
        msg.header.stamp = rospy.Time.now()
        q_wxyz_acc_gyro = serial_call_with_retry(self.interface.read_imu_data)
        if q_wxyz_acc_gyro is None:
            return
        q_wxyz, acc, gyro = q_wxyz_acc_gyro
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
        if self.publish_sensor is False:
            return
        if not self.interface.is_opened():
            return
        stamp = rospy.Time.now()
        msg = geometry_msgs.msg.WrenchStamped()
        msg.header.stamp = stamp
        sensors = serial_call_with_retry(self.interface.all_jointbase_sensors)
        if sensors is None:
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
                            self.base_namespace + f"/kjs/{sensor.id}/{typ}/{i}",
                            geometry_msgs.msg.WrenchStamped,
                            queue_size=1,
                        )
                        # Avoid 'rospy.exceptions.ROSException:
                        # publish() to a closed topic'
                        rospy.sleep(0.1)
                    msg.header.frame_id = f"kjs_{sensor.id}_{i}_frame"
                    self._sensor_publisher_dict[key].publish(msg)

    def publish_battery_voltage_value(self):
        if (
            self.publish_battery_voltage is False
            or self.battery_voltage_publisher.get_num_connections() == 0
        ):
            return
        battery_voltage = serial_call_with_retry(self.interface.battery_voltage)
        if battery_voltage is None:
            return
        self.battery_voltage_publisher.publish(std_msgs.msg.Float32(data=battery_voltage))

    def publish_joint_states(self):
        av = serial_call_with_retry(self.interface.angle_vector)
        torque_vector = serial_call_with_retry(self.interface.servo_error)
        if self.read_current:
            currents = serial_call_with_retry(self.interface.read_servo_current)
        else:
            currents = None
        if self.read_temperature:
            temperatures = serial_call_with_retry(self.interface.read_servo_temperature)
        else:
            temperatures = None
        if av is None or torque_vector is None:
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        servos_msg = ServoStateArray()
        servos_msg.header.stamp = msg.header.stamp
        for name in self.joint_names:
            if name in self.joint_name_to_id:
                servo_id = self.joint_name_to_id[name]
                idx = self.interface.servo_id_to_index(servo_id)
                if idx is None:
                    continue
                position = np.deg2rad(av[idx])
                effort = np.deg2rad(torque_vector[idx])
                msg.position.append(position)
                msg.effort.append(effort)
                msg.name.append(name)
                servo_state_msg = ServoState(
                    header=msg.header,
                    name=name,
                    position=position,
                    error=effort)
                if temperatures is not None and len(temperatures) > idx:
                    servo_state_msg.temperature = temperatures[idx]
                if currents is not None and len(currents) > idx:
                    servo_state_msg.current = currents[idx]
                servos_msg.servos.append(servo_state_msg)
        self.current_joint_states_pub.publish(msg)
        self.servo_states_pub.publish(servos_msg)
        return True

    def publish_servo_on_off(self):
        if self.servo_on_off_pub.get_num_connections() == 0:
            return
        if not self.interface.is_opened():
            return

        servo_on_off_msg = ServoOnOff()
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            idx = self.joint_name_to_id[jn]
            if idx not in self.interface.servo_on_states_dict:
                continue
            servo_on_off_msg.joint_names.append(jn)
            servo_state = self.interface.servo_on_states_dict[idx]
            servo_on_off_msg.servo_on_states.append(servo_state)
        self.servo_on_off_pub.publish(servo_on_off_msg)

    def reinitialize_interface(self):
        rospy.loginfo("Reinitialize interface.")
        self.unsubscribe()
        self.interface.close()
        self.setup_interface_and_servo_parameters()
        self.subscribe()
        rospy.loginfo("Successfully reinitialized interface.")

    def check_success_rate(self):
        # Calculate success rate
        for topic_type in ["joint_states", "pressure"]:
            if self.publish_attempts[topic_type] > 0:
                success_rate = (
                    self.publish_successes[topic_type] / self.publish_attempts[topic_type]
                )

                # Check if the success rate is below the threshold
                if success_rate < self.success_rate_threshold:
                    rospy.logwarn(
                        f"[{topic_type}] communication success rate ({success_rate:.2%}) below threshold; "
                        + "reinitializing interface."
                    )
                    self.reinitialize_interface()

            # Reset counters and timer
            self.publish_successes[topic_type] = 0
            self.publish_attempts[topic_type] = 0
        self.last_check_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(self.hz)

        self.publish_attempts = {}
        self.publish_successes = {}
        for topic_type in ["joint_states", "pressure"]:
            self.publish_attempts[topic_type] = 0
            self.publish_successes[topic_type] = 0
        self.last_check_time = rospy.Time.now()
        check_board_communication_interval = rospy.get_param(
            "~check_board_communication_interval", 2
        )
        self.success_rate_threshold = 0.8  # Minimum success rate required
        use_rcb4 = rospy.get_param("~use_rcb4")

        while not rospy.is_shutdown():
            if self._update_current_limit:
                ret = serial_call_with_retry(self.interface.send_current_limit,
                                             self.current_limit, max_retries=3)
                if ret is not None:
                    rospy.loginfo(f"Current limit set {self.current_limit}")
                    self._update_current_limit = False
                else:
                    rospy.logwarn("Could not set current limit")
            if self._update_temperature_limit:
                ret = serial_call_with_retry(self.interface.send_temperature_limit,
                                       self.temperature_limit, max_retries=3)
                if ret is not None:
                    rospy.loginfo(f"Temperature limit set {self.temperature_limit}")
                    self._update_temperature_limit = False
                else:
                    rospy.logwarn("Could not set temperature limit")

            success = self.publish_joint_states()
            self.publish_attempts["joint_states"] += 1
            if success:
                self.publish_successes["joint_states"] += 1

            self.publish_servo_on_off()
            self.publish_imu_message()
            self.publish_sensor_values()
            self.publish_battery_voltage_value()
            if use_rcb4 is False and self.control_pressure:
                self.publish_pressure_control()
                success = self.publish_pressure()
                self.publish_attempts["pressure"] += 1
                if success:
                    self.publish_successes["pressure"] += 1

            # Check success rate periodically
            current_time = rospy.Time.now()
            if (
                current_time - self.last_check_time
            ).to_sec() >= check_board_communication_interval:
                self.check_success_rate()

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rcb4_ros_bridge")
    ros_bridge = RCB4ROSBridge()
    ros_bridge.run()
