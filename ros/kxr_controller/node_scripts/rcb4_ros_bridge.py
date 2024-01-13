#!/usr/bin/env python3

import os
import os.path as osp
import shlex
import subprocess
import sys

import actionlib
from kxr_controller.msg import ServoOnOffAction
from kxr_controller.msg import ServoOnOffResult
import numpy as np
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import JointState
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
import yaml

from rcb4.armh7interface import ARMH7Interface
from rcb4.armh7interface import WormmoduleStruct


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
        raise OSError('{} not exists'.format(str(file_path)))
    with open(osp.expanduser(file_path), 'r') as f:
        data = yaml.load(f, Loader=Loader)
    data = data['joint_name_to_servo_id']
    joint_name_to_id = {}
    for name in data:
        if isinstance(data[name], int):
            joint_name_to_id[name] = data[name]
        else:
            joint_name_to_id[name] = data[name]['id']
    return joint_name_to_id, data


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
    command += ' __name=:kxr_controller'
    command = shlex.split(command)
    process = subprocess.Popen(command)
    return process


def set_initial_position(positions, namespace=None):
    rospy.set_param(namespace + '/initial_position',
                    positions)


def set_fullbody_controller(joint_names):
    controller_yaml_dict = {
        'type': 'position_controllers/JointTrajectoryController',
        'joints': joint_names,
    }
    rospy.set_param('kxr_fullbody_controller', controller_yaml_dict)


def set_joint_state_controler():
    rospy.set_param('joint_state_controller', {
        'type': 'joint_state_controller/JointStateController',
        'publish_rate': 10
    })


def set_robot_description(urdf_path,
                          param_name='robot_description'):
    with open(urdf_path) as f:
        rospy.set_param(param_name, f.read())


class RCB4ROSBridge(object):

    def __init__(self):
        r = RobotModel()
        urdf_path = rospy.get_param('~urdf_path')
        with open(urdf_path) as f:
            with no_mesh_load_mode():
                r.load_urdf_file(f)

        servo_config_path = rospy.get_param('~servo_config_path')
        self.joint_name_to_id, servo_infos = load_yaml(servo_config_path)

        joint_list = [j for j in r.joint_list
                      if j.__class__.__name__ != 'FixedJoint']
        self.joint_names = [j.name for j in joint_list]

        full_namespace = rospy.get_namespace()
        last_slash_pos = full_namespace.rfind('/')
        clean_namespace = full_namespace[:last_slash_pos] \
            if last_slash_pos != 0 else ''

        set_robot_description(
            urdf_path,
            param_name=clean_namespace + '/robot_description')
        set_joint_state_controler()
        self.current_joint_states_pub = rospy.Publisher(
            clean_namespace + '/current_joint_states',
            JointState,
            queue_size=1)

        self.arm = ARMH7Interface()
        arm = self.arm
        ret = arm.auto_open()
        if ret is not True:
            rospy.logerr('Could not open port!')
            sys.exit(1)
        arm.all_jointbase_sensors()
        # arm.send_stretch(40)
        self.id_to_index = self.arm.servo_id_to_index()
        self._prev_velocity_command = None

        print(arm.joint_to_actuator_matrix)
        print(arm._actuator_to_joint_matrix)
        for _, info in servo_infos.items():
            if isinstance(info, int):
                continue
            servo_id = info['id']
            direction = info['direction']
            idx = self.id_to_index[servo_id]
            arm._joint_to_actuator_matrix[idx, idx] = \
                direction * arm._joint_to_actuator_matrix[idx, idx]
        print(arm.joint_to_actuator_matrix)

        self.fullbody_jointnames = []
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            servo_id = self.joint_name_to_id[jn]
            if servo_id in arm.wheel_servo_sorted_ids:
                continue
            self.fullbody_jointnames.append(jn)
        set_fullbody_controller(self.fullbody_jointnames)
        # set_fullbody_controller(self.joint_names)
        print('Fullbody jointnames')
        print(self.fullbody_jointnames)

        self.servo_id_to_worm_id = self.arm.servo_id_to_worm_id

        self.joint_servo_on = {jn: False for jn in self.joint_names}
        servo_on_states = arm.servo_states()
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            idx = self.joint_name_to_id[jn]
            if idx in servo_on_states:
                self.joint_servo_on[jn] = True
            else:
                self.joint_servo_on[jn] = False
        print(self.joint_servo_on)
        print(self.id_to_index)

        initial_positions = {}
        init_av = arm.angle_vector()
        arm.servo_id_to_index()
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            servo_id = self.joint_name_to_id[jn]
            if servo_id in arm.wheel_servo_sorted_ids:
                continue
            initial_positions[jn] = float(
                np.deg2rad(init_av[self.id_to_index[servo_id]]))
        rospy.loginfo('run kxr_controller')
        set_initial_position(initial_positions, namespace=clean_namespace)
        self.proc_kxr_controller = run_kxr_controller(
            namespace=clean_namespace)

        self.worm_servo_ids = [
            arm.memory_cstruct(WormmoduleStruct, idx).servo_id
            for idx in arm.worm_sorted_ids]

        self.command_joint_state_sub = rospy.Subscriber(
            clean_namespace + '/command_joint_state',
            JointState, queue_size=1,
            callback=self.command_joint_state_callback)

        self.velocity_command_joint_state_sub = rospy.Subscriber(
            clean_namespace + '/velocity_command_joint_state',
            JointState, queue_size=1,
            callback=self.velocity_command_joint_state_callback)

        self.servo_on_off_server = actionlib.SimpleActionServer(
            clean_namespace
            + '/kxr_fullbody_controller/servo_on_off_real_interface',
            ServoOnOffAction,
            execute_cb=self.servo_on_off_callback,
            auto_start=False)
        self.servo_on_off_server.start()

        self.proc_controller_spawner = subprocess.Popen(
            [f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun',
             'controller_manager', 'spawner']
            + ['joint_state_controller', 'kxr_fullbody_controller'])
        self.proc_robot_state_publisher = run_robot_state_publisher(
            clean_namespace)

        self.publish_imu = rospy.get_param('~publish_imu', True)
        if self.publish_imu:
            self.imu_frame_id = rospy.get_param(
                '~imu_frame_id', clean_namespace + '/' + r.root_link.name)
            self.imu_publisher = rospy.Publisher(
                clean_namespace + '/imu',
                sensor_msgs.msg.Imu,
                queue_size=1)

    def __del__(self):
        self.proc_controller_spawner.kill()
        self.proc_robot_state_publisher.kill()
        self.proc_kxr_controller.kill()

    def unsubscribe(self):
        self.command_joint_state_sub.unregister()
        self.velocity_command_joint_state_sub.unregister()

    def velocity_command_joint_state_callback(self, msg):
        servo_ids = []
        av_length = len(self.arm.servo_sorted_ids)
        svs = np.zeros(av_length)
        valid_indices = []
        for name, angle in zip(msg.name, msg.position):
            if name not in self.joint_name_to_id:
                continue
            if name in self.joint_servo_on \
               and self.joint_servo_on[name] is False:
                continue
            idx = self.joint_name_to_id[name]

            # should ignore duplicated index.
            if idx not in self.id_to_index:
                continue
            # skip wheel joint
            if idx not in self.arm.wheel_servo_sorted_ids:
                continue
            if self.id_to_index[idx] in valid_indices:
                continue

            angle = np.rad2deg(angle)
            svs[self.id_to_index[idx]] = angle
            valid_indices.append(self.id_to_index[idx])
            servo_ids.append(idx)
        tmp_av = np.ones(av_length + 1)
        if len(valid_indices) == 0:
            return
        try:
            tmp_av[:len(svs)] = np.array(svs)
        except Exception:
            return
        svs = np.matmul(self.arm.joint_to_actuator_matrix, tmp_av)[
            np.array(valid_indices)]
        if self._prev_velocity_command is not None and np.allclose(
                self._prev_velocity_command, svs):
            return
        self._prev_velocity_command = svs
        try:
            self.arm.servo_angle_vector(servo_ids, svs, velocity=0)
        except RuntimeError as e:
            self.unsubscribe()
            rospy.signal_shutdown('Disconnected {}.'.format(e))

    def command_joint_state_callback(self, msg):
        servo_ids = []
        av_length = len(self.arm.servo_sorted_ids)
        svs = np.zeros(av_length)
        valid_indices = []
        worm_av = []
        worm_indices = []
        for name, angle in zip(msg.name, msg.position):
            if name not in self.joint_name_to_id:
                continue
            if name in self.joint_servo_on \
               and self.joint_servo_on[name] is False:
                continue
            idx = self.joint_name_to_id[name]

            # should ignore duplicated index.
            if idx not in self.id_to_index:
                continue
            # skip wheel joint
            if idx in self.arm.wheel_servo_sorted_ids:
                continue
            if self.id_to_index[idx] in valid_indices:
                continue

            angle = np.rad2deg(angle)
            if idx in self.worm_servo_ids:
                worm_idx = self.servo_id_to_worm_id[idx]
                worm_av.append(angle)
                worm_indices.append(worm_idx)
                angle = 135
            svs[self.id_to_index[idx]] = angle
            valid_indices.append(self.id_to_index[idx])
            servo_ids.append(idx)
        tmp_av = np.ones(av_length + 1)
        if len(valid_indices) == 0:
            return
        try:
            tmp_av[:len(svs)] = np.array(svs)
        except Exception:
            return
        svs = np.matmul(self.arm.joint_to_actuator_matrix, tmp_av)[
            np.array(valid_indices)]
        try:
            if len(worm_indices) > 0:
                worm_av_tmp = np.array(self.arm.read_cstruct_slot_vector(
                    WormmoduleStruct, 'ref_angle'), dtype=np.float32)
                worm_av_tmp[np.array(worm_indices)] = np.array(worm_av)
                self.arm.write_cstruct_slot_v(
                    WormmoduleStruct, 'ref_angle', worm_av_tmp)
            self.arm.servo_angle_vector(servo_ids, svs, velocity=0)
        except RuntimeError as e:
            self.unsubscribe()
            rospy.signal_shutdown('Disconnected {}.'.format(e))

    def servo_on_off_callback(self, goal):
        servo_vector = []
        servo_ids = []
        for joint_name, servo_on in zip(
                goal.joint_names, goal.servo_on_states):
            if joint_name not in self.joint_name_to_id:
                continue
            servo_ids.append(self.joint_name_to_id[joint_name])
            if servo_on:
                servo_vector.append(32767)
                self.joint_servo_on[joint_name] = True
            else:
                servo_vector.append(32768)
                self.joint_servo_on[joint_name] = False
        try:
            self.arm.servo_angle_vector(servo_ids, servo_vector, velocity=10)
        except RuntimeError as e:
            self.unsubscribe()
            rospy.signal_shutdown('Disconnected {}.'.format(e))
        return self.servo_on_off_server.set_succeeded(ServoOnOffResult())

    def create_imu_message(self):
        msg = sensor_msgs.msg.Imu()
        msg.header.frame_id = self.imu_frame_id
        msg.header.stamp = rospy.Time.now()
        q_wxyz = self.arm.read_quaternion()
        (msg.orientation.w, msg.orientation.x,
         msg.orientation.y, msg.orientation.z) = q_wxyz
        _, gyro = self.arm.gyro_norm_vector()
        (msg.angular_velocity.x, msg.angular_velocity.y,
         msg.angular_velocity.z) = gyro
        return msg

    def run(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.arm.is_opened() is False:
                self.unsubscribe()
                rospy.signal_shutdown('Disconnected.')
                break
            try:
                av = self.arm.angle_vector()
            except RuntimeError as e:
                self.unsubscribe()
                rospy.signal_shutdown('Disconnected {}.'.format(e))
                break
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            for name in self.joint_names:
                if name in self.joint_name_to_id:
                    servo_id = self.joint_name_to_id[name]
                    if servo_id in self.id_to_index:
                        msg.position.append(
                            np.deg2rad(
                                av[self.id_to_index[servo_id]]))
                        msg.name.append(name)
            self.current_joint_states_pub.publish(msg)

            if self.publish_imu and self.imu_publisher.get_num_connections():
                imu_msg = self.create_imu_message()
                self.imu_publisher.publish(imu_msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('rcb4_ros_bridge')
    ros_bridge = RCB4ROSBridge()  # NOQA
    ros_bridge.run()
