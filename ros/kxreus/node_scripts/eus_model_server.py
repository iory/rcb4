#!/usr/bin/env python

import os
import tempfile

from filelock import FileLock
from kxr_models.md5sum_utils import checksum_md5
import rospkg
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from urdfeus.urdf2eus import urdf2eus


class EusModelServer(object):

    def __init__(self):
        full_namespace = rospy.get_namespace()
        last_slash_pos = full_namespace.rfind('/')
        self.clean_namespace = full_namespace[:last_slash_pos] \
            if last_slash_pos != 0 else ''

    def load_robot_model(self, urdf_path):
        r = RobotModel()
        with open(urdf_path, 'r') as f:
            with no_mesh_load_mode():
                r.load_urdf_file(f)
        return r

    def run(self):
        rate = rospy.Rate(1)

        previous_md5sum = None
        robot_model = None

        rospack = rospkg.RosPack()
        kxr_models_path = rospack.get_path('kxr_models')

        while not rospy.is_shutdown():
            rate.sleep()
            urdf = rospy.get_param(
                self.clean_namespace + '/robot_description',
                None)
            if urdf is not None:
                tmp_file = tempfile.mktemp()
                with open(tmp_file, "w") as f:
                    f.write(urdf)
                md5sum = checksum_md5(tmp_file)
                if previous_md5sum is None or previous_md5sum != md5sum:
                    robot_model = self.load_robot_model(tmp_file)
                previous_md5sum = md5sum

                eus_path = os.path.join(
                    kxr_models_path, 'models',
                    'euslisp', '{}.l'.format(md5sum))
                robot_name = robot_model.urdf_robot_model.name
                if os.path.exists(eus_path):
                    rospy.set_param(self.clean_namespace + '/eus_robot_name',
                                    robot_name)
                    rospy.set_param(self.clean_namespace + '/eusmodel_hash',
                                    md5sum)
                    continue

                lock_path = eus_path + ".lock"
                lock = FileLock(lock_path, timeout=10)
                try:
                    with lock:
                        with open(eus_path, 'w') as f:
                            urdf2eus(tmp_file, fp=f)
                        rospy.loginfo(
                            'Eusmodel is saved to {}'.format(eus_path))
                    os.remove(lock_path)
                finally:
                    if os.path.exists(tmp_file):
                        os.remove(tmp_file)


if __name__ == '__main__':
    rospy.init_node('model_server')
    server = EusModelServer()
    server.run()
