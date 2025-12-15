#!/usr/bin/env python

import os
import rospy
import actionlib
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QLabel, QPushButton
from rqt_gui_py.plugin import Plugin

from sensor_msgs.msg import JointState
from kxr_controller.msg import ServoOnOffAction, ServoOnOffGoal


class ServoOnOff(Plugin):

    def __init__(self, context):
        super(ServoOnOff, self).__init__(context)
        self.setObjectName('ServoOnOff')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file
        ui_file = os.path.join(
            rospkg.RosPack().get_path('kxr_rqt_plugin'),
            'resource',
            'ServoOnOff.ui'
        )

        # Load UI
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('ServoOnOffUi')

        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number())
            )

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize variables
        self._namespace = '/'
        self._joint_names = []
        self._joint_checkboxes = {}
        self._action_client = None

        # Connect signals
        self._widget.refresh_button.clicked.connect(self._on_refresh_clicked)
        self._widget.all_on_button.clicked.connect(self._on_all_on_clicked)
        self._widget.all_off_button.clicked.connect(self._on_all_off_clicked)
        self._widget.namespace_line_edit.returnPressed.connect(self._on_refresh_clicked)

        # Initialize
        self._on_refresh_clicked()

    def _on_refresh_clicked(self):
        """Refresh joint list and reconnect to action server."""
        self._namespace = self._widget.namespace_line_edit.text()
        if not self._namespace.endswith('/'):
            self._namespace += '/'

        # Try to connect to action server
        action_name = self._namespace + 'fullbody_controller/servo_on_off_real_interface'

        try:
            self._widget.label_status.setText('Status: Connecting...')
            self._action_client = actionlib.SimpleActionClient(
                action_name,
                ServoOnOffAction
            )

            # Wait for server with timeout
            if self._action_client.wait_for_server(rospy.Duration(3.0)):
                self._widget.label_status.setText('Status: Connected to ' + action_name)

                # Get joint names from rosparam
                self._get_joint_names_from_param()
            else:
                self._widget.label_status.setText('Status: Failed to connect to ' + action_name)
                self._action_client = None

        except Exception as e:
            self._widget.label_status.setText('Status: Error - ' + str(e))
            self._action_client = None

    def _get_joint_names_from_param(self):
        """Get joint names from rosparam."""
        joints_param = self._namespace + 'fullbody_controller/joints'

        try:
            if rospy.has_param(joints_param):
                joint_names = rospy.get_param(joints_param)
                rospy.loginfo(f"Got {len(joint_names)} joints from {joints_param}")

                if joint_names and joint_names != self._joint_names:
                    self._joint_names = list(joint_names)
                    self._create_joint_checkboxes()
                    self._widget.label_status.setText(
                        f'Status: Connected ({len(joint_names)} joints)'
                    )
            else:
                rospy.logwarn(f"Parameter {joints_param} not found. Trying joint_states...")
                # Fall back to joint_states if param not found
                self._subscribe_joint_states()
        except Exception as e:
            rospy.logerr(f"Failed to get joints from param: {e}")
            self._subscribe_joint_states()

    def _subscribe_joint_states(self):
        """Subscribe to joint_states topic to get joint names."""
        # Try namespace/joint_states first, then fall back to /joint_states
        if self._namespace == '/':
            joint_states_topic = '/joint_states'
        else:
            joint_states_topic = self._namespace + 'joint_states'

        rospy.loginfo(f"Subscribing to joint_states topic: {joint_states_topic}")

        # Subscribe to joint_states (one-shot)
        rospy.Subscriber(
            joint_states_topic,
            JointState,
            self._joint_states_callback,
            queue_size=1
        )

    def _joint_states_callback(self, msg):
        """Callback for joint_states topic."""
        if msg.name and msg.name != self._joint_names:
            self._joint_names = list(msg.name)
            self._create_joint_checkboxes()

    def _create_joint_checkboxes(self):
        """Create checkboxes for each joint."""
        # Clear existing checkboxes
        layout = self._widget.joint_layout
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        self._joint_checkboxes = {}

        # Create checkbox for each joint
        for joint_name in self._joint_names:
            # Create horizontal layout for each joint
            h_layout = QHBoxLayout()

            # Create checkbox
            checkbox = QCheckBox(joint_name)
            checkbox.setChecked(False)
            self._joint_checkboxes[joint_name] = checkbox
            h_layout.addWidget(checkbox)

            # Create individual ON button
            on_button = QPushButton('ON')
            on_button.clicked.connect(
                lambda checked, jn=joint_name: self._on_individual_on_clicked(jn)
            )
            h_layout.addWidget(on_button)

            # Create individual OFF button
            off_button = QPushButton('OFF')
            off_button.clicked.connect(
                lambda checked, jn=joint_name: self._on_individual_off_clicked(jn)
            )
            h_layout.addWidget(off_button)

            # Add to main layout
            layout.addLayout(h_layout)

    def _on_all_on_clicked(self):
        """Turn on all servos."""
        if self._action_client is None:
            rospy.logwarn('Action client not connected')
            return

        goal = ServoOnOffGoal()
        goal.joint_names = self._joint_names
        goal.servo_on_states = [True] * len(self._joint_names)

        self._send_goal(goal)

    def _on_all_off_clicked(self):
        """Turn off all servos."""
        if self._action_client is None:
            rospy.logwarn('Action client not connected')
            return

        goal = ServoOnOffGoal()
        goal.joint_names = self._joint_names
        goal.servo_on_states = [False] * len(self._joint_names)

        self._send_goal(goal)

    def _on_individual_on_clicked(self, joint_name):
        """Turn on individual servo."""
        if self._action_client is None:
            rospy.logwarn('Action client not connected')
            return

        goal = ServoOnOffGoal()
        goal.joint_names = [joint_name]
        goal.servo_on_states = [True]

        self._send_goal(goal)

    def _on_individual_off_clicked(self, joint_name):
        """Turn off individual servo."""
        if self._action_client is None:
            rospy.logwarn('Action client not connected')
            return

        goal = ServoOnOffGoal()
        goal.joint_names = [joint_name]
        goal.servo_on_states = [False]

        self._send_goal(goal)

    def _send_goal(self, goal):
        """Send goal to action server."""
        try:
            self._action_client.send_goal(goal)
            self._action_client.wait_for_result(rospy.Duration(5.0))
            result = self._action_client.get_result()

            if result:
                rospy.loginfo('Servo on/off command completed successfully')
            else:
                rospy.logwarn('Servo on/off command failed or timed out')

        except Exception as e:
            rospy.logerr('Error sending servo on/off goal: ' + str(e))

    def shutdown_plugin(self):
        """Shutdown the plugin."""
        pass

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings."""
        instance_settings.set_value('namespace', self._namespace)

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore settings."""
        namespace = instance_settings.value('namespace', '/')
        self._widget.namespace_line_edit.setText(namespace)
        self._on_refresh_clicked()
