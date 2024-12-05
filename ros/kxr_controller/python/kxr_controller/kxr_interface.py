import numbers

import actionlib
import actionlib_msgs.msg
import control_msgs.msg
import numpy as np
import rospy
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
import std_msgs.msg

from kxr_controller.msg import AdjustAngleVectorAction
from kxr_controller.msg import AdjustAngleVectorGoal
from kxr_controller.msg import PressureControlAction
from kxr_controller.msg import PressureControlGoal
from kxr_controller.msg import ServoOnOffAction
from kxr_controller.msg import ServoOnOffGoal
from kxr_controller.msg import Stretch
from kxr_controller.msg import StretchAction
from kxr_controller.msg import StretchGoal


class KXRROSRobotInterface(ROSRobotInterfaceBase):
    def __init__(self, *args, **kwargs):
        namespace = kwargs.get("namespace", "")
        namespace = f"/{namespace}" if namespace else ""
        joint_param = namespace + "/fullbody_controller/joints"
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.joint_names = rospy.get_param(joint_param, None)
            if self.joint_names is not None:
                break
            rospy.logwarn(f"Waiting {joint_param} set")
            rate.sleep()
        super(KXRROSRobotInterface, self).__init__(*args, **kwargs)
        self.use_sim_time = rospy.get_param("/use_sim_time", False)
        # Servo on off client
        if self.use_sim_time is False:
            self.servo_on_off_client = actionlib.SimpleActionClient(
                namespace + "/fullbody_controller/servo_on_off_real_interface",
                ServoOnOffAction
            )
            self.servo_on_off_client.wait_for_server()
            # Stretch client
            self.stretch_client = actionlib.SimpleActionClient(
                namespace + "/fullbody_controller/stretch_interface", StretchAction
            )
            timeout = rospy.Duration(10.0)
            self.enabled_stretch = True
            if not self.stretch_client.wait_for_server(timeout):
                rospy.logerr("Stretch action server not available.")
                self.enabled_stretch = False
            self.stretch_topic_name = namespace + "/fullbody_controller/stretch"
            # Pressure control client
            pressure_param = namespace + "/rcb4_ros_bridge/control_pressure"
            self.control_pressure = rospy.get_param(pressure_param, False)
            if self.control_pressure is True:
                self.pressure_control_client = actionlib.SimpleActionClient(
                    namespace + "/fullbody_controller/pressure_control_interface",
                    PressureControlAction,
                )
                self.enabled_pressure_control = True
                if not self.pressure_control_client.wait_for_server(timeout):
                    rospy.logerr("PressureControl action server not available.")
                    self.enabled_pressure_control = False
                self.pressure_topic_name_base = (
                    namespace + "/fullbody_controller/pressure/"
                )
            # Adjust angle vector client
            self.adjust_angle_vector_client = actionlib.SimpleActionClient(
                namespace + "/fullbody_controller/adjust_angle_vector_interface",
                AdjustAngleVectorAction,
            )
            self.adjust_angle_vector_client.wait_for_server()

    def servo_on(self, joint_names=None):
        if self.use_sim_time:
            rospy.logwarn("In simulation mode, servo on function is disabled.")
            return
        if joint_names is None:
            joint_names = self.joint_names
        goal = ServoOnOffGoal()
        client = self.servo_on_off_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        goal.joint_names = joint_names
        goal.servo_on_states = [True] * len(joint_names)
        client.send_goal(goal)
        client.wait_for_result(timeout=rospy.Duration(10))
        return client

    def servo_off(self, joint_names=None):
        if self.use_sim_time:
            rospy.logwarn("In simulation mode, servo off function is disabled.")
            return
        if joint_names is None:
            joint_names = self.joint_names
        goal = ServoOnOffGoal()
        client = self.servo_on_off_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        goal.joint_names = joint_names
        goal.servo_on_states = [False] * len(joint_names)
        client.send_goal(goal)
        client.wait_for_result(timeout=rospy.Duration(10))
        return client

    def adjust_angle_vector(self, joint_names=None, error_threshold=None):
        if self.use_sim_time:
            rospy.logwarn("In simulation mode, adjust angle vector function is disabled.")
            return
        if error_threshold is None:
            error_threshold = np.deg2rad(5)
        if joint_names is None:
            joint_names = self.joint_names
        goal = AdjustAngleVectorGoal()
        client = self.adjust_angle_vector_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        goal.joint_names = joint_names
        if isinstance(error_threshold, numbers.Number):
            error_threshold = [error_threshold for _ in range(len(joint_names))]
        goal.error_threshold = error_threshold
        client.send_goal(goal)

    def wait_interpolation(
        self,
        *args,
        timeout=0,
        adjust_angle_vector=False,
        joint_names=None,
        error_threshold=None,
        **kwargs,
    ):
        if adjust_angle_vector is False:
            return super().wait_interpolation(*args, timeout=timeout, **kwargs)
        if error_threshold is None:
            error_threshold = np.deg2rad(5)
        if timeout == 0:
            timeout = float("inf")
        start_time = rospy.Time.now()
        # When all controllers finish interpolation or timeout,
        # return from this function
        while not rospy.is_shutdown():
            self.adjust_angle_vector(
                joint_names=joint_names, error_threshold=error_threshold
            )
            is_interpolating_list = super().wait_interpolation(
                *args, timeout=0.1, **kwargs
            )
            is_interpolating = any(is_interpolating_list)
            elapsed_time = rospy.Time.now() - start_time
            elapsed_time = elapsed_time.to_sec()
            if is_interpolating is False or elapsed_time > timeout:
                return is_interpolating_list

    def send_stretch(self, value=127, joint_names=None):
        if not self.enabled_stretch:
            rospy.logerr("Stretch action server not available.")
            return
        if joint_names is None:
            joint_names = self.joint_names
        goal = StretchGoal()
        client = self.stretch_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        goal.joint_names = joint_names
        goal.stretch = value
        client.send_goal(goal)

    def read_stretch(self):
        if not self.enabled_stretch:
            rospy.logerr("Stretch action server not available.")
            return
        return rospy.wait_for_message(self.stretch_topic_name, Stretch)

    def send_pressure_control(self, board_idx, start_pressure, stop_pressure, release):
        goal = PressureControlGoal(
            board_idx=board_idx,
            start_pressure=start_pressure,
            stop_pressure=stop_pressure,
            release=release,
        )
        client = self.pressure_control_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        client.send_goal(goal)

    def read_pressure(self, board_idx):
        if not self.enabled_pressure_control:
            rospy.logerr("Pressure control action server not available.")
            return
        return rospy.wait_for_message(
            self.pressure_topic_name_base + f"{board_idx}", std_msgs.msg.Float32
        )

    @property
    def fullbody_controller(self):
        cont_name = "fullbody_controller"
        return {
            "controller_type": cont_name,
            "controller_action": cont_name + "/follow_joint_trajectory",
            "controller_state": cont_name + "/state",
            "action_type": control_msgs.msg.FollowJointTrajectoryAction,
            "joint_names": self.joint_names,
        }

    def default_controller(self):
        controller_names = rospy.get_param(
            self.namespace + 'default_controller', None)
        if controller_names is None:
            return [self.fullbody_controller]
        controllers = []
        for cont_name in controller_names:
            controllers.append({
                "controller_type": cont_name,
                "controller_action": cont_name + "/follow_joint_trajectory",
                "controller_state": cont_name + "/state",
                "action_type": control_msgs.msg.FollowJointTrajectoryAction,
                "joint_names": rospy.get_param(
                    self.namespace + cont_name + '/joints'),
            })
        return controllers
