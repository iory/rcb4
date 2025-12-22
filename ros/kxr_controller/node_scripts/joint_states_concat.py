#!/usr/bin/env python3

"""
Usage in launch file:

<node name="joint_states_concat" pkg="kxr_controller" type="joint_states_concat.py">
  <rosparam param="input_topics">
    - topic: /kxr/joint_states
      prefix: "kxr_"
    - topic: /panda/joint_states
      prefix: "panda_"
  </rosparam>
  <param name="output_topic" value="/combined_joint_states"/>
  <param name="publish_rate" value="100.0"/>
</node>

You can also use the simple format without prefixes:

<node name="joint_states_concat" pkg="kxr_controller" type="joint_states_concat.py">
  <rosparam param="input_topics">
    - /kxr/joint_states
    - /panda/joint_states
  </rosparam>
</node>
"""

import rospy
from sensor_msgs.msg import JointState


class JointStatesConcat:
    """Concatenate multiple JointState messages into a single JointState message."""

    def __init__(self):
        input_topics = rospy.get_param("~input_topics", [])
        if not input_topics:
            rospy.logerr("No input_topics specified. Please set ~input_topics parameter.")
            rospy.signal_shutdown("No input topics")
            return

        self.output_topic = rospy.get_param("~output_topic", "joint_states")
        self.publish_rate = rospy.get_param("~publish_rate", 100.0)

        self.latest_msgs = {}
        self.topic_prefixes = {}
        self.subscribers = []

        for item in input_topics:
            if isinstance(item, dict):
                topic = item.get("topic", "")
                prefix = item.get("prefix", "")
            else:
                topic = item
                prefix = ""

            if not topic:
                rospy.logwarn("Empty topic name, skipping")
                continue

            self.latest_msgs[topic] = None
            self.topic_prefixes[topic] = prefix
            sub = rospy.Subscriber(
                topic,
                JointState,
                self.callback,
                callback_args=topic,
                queue_size=1,
            )
            self.subscribers.append(sub)
            if prefix:
                rospy.loginfo(f"Subscribed to {topic} with prefix '{prefix}'")
            else:
                rospy.loginfo(f"Subscribed to {topic}")

        self.publisher = rospy.Publisher(
            self.output_topic, JointState, queue_size=1
        )
        rospy.loginfo(f"Publishing concatenated joint states to {self.output_topic}")

    def callback(self, msg, topic):
        self.latest_msgs[topic] = msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_concatenated()
            rate.sleep()

    def publish_concatenated(self):
        if not any(self.latest_msgs.values()):
            return

        concat_msg = JointState()
        concat_msg.header.stamp = rospy.Time.now()

        seen_names = set()
        for topic, msg in self.latest_msgs.items():
            if msg is None:
                continue
            prefix = self.topic_prefixes.get(topic, "")
            for i, name in enumerate(msg.name):
                prefixed_name = prefix + name
                if prefixed_name in seen_names:
                    continue
                seen_names.add(prefixed_name)
                concat_msg.name.append(prefixed_name)
                if i < len(msg.position):
                    concat_msg.position.append(msg.position[i])
                if i < len(msg.velocity):
                    concat_msg.velocity.append(msg.velocity[i])
                if i < len(msg.effort):
                    concat_msg.effort.append(msg.effort[i])

        if concat_msg.name:
            self.publisher.publish(concat_msg)


if __name__ == "__main__":
    rospy.init_node("joint_states_concat")
    node = JointStatesConcat()
    node.run()
