#!/usr/bin/env python


import argparse

import IPython
from kxr_controller.check_ros_master import is_ros_master_local
from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np  # NOQA
import rospy
from skrobot.model import RobotModel


def main():
    parser = argparse.ArgumentParser(description="Run KXRROSRobotInterface")
    parser.add_argument(
        "--viewer", type=str, help="Specify the viewer: trimesh", default=None
    )
    parser.add_argument(
        "--namespace", type=str, help="Specify the ROS namespace", default=""
    )
    args = parser.parse_args()

    rospy.init_node("kxr_interface", anonymous=True)

    if is_ros_master_local() is True:
        robot_description = args.namespace + "/robot_description"
    else:
        from kxr_models.download_urdf import download_urdf_mesh_files
        download_urdf_mesh_files(args.namespace)
        robot_description = args.namespace + "/robot_description_viz"

    robot_model = RobotModel()
    if args.viewer is None:
        from skrobot.utils.urdf import no_mesh_load_mode
        with no_mesh_load_mode():
            robot_model.load_urdf_from_robot_description(robot_description)
    else:
        robot_model.load_urdf_from_robot_description(robot_description)
    ri = KXRROSRobotInterface(
        robot_model, namespace=args.namespace, controller_timeout=60.0
    )

    if args.viewer is not None:
        if args.viewer == "trimesh":
            from skrobot.viewers import TrimeshSceneViewer

            viewer = TrimeshSceneViewer(resolution=(640, 480))
            viewer.add(robot_model)
            viewer.show()
        elif args.viewer == "pyrender":
            from skrobot.viewers import PyrenderViewer

            viewer = PyrenderViewer(resolution=(640, 480))
            viewer.add(robot_model)
            viewer.show()
        else:
            raise NotImplementedError(f"Not supported viewer {args.viewer}")

    # Drop into an IPython shell with both local & global vars
    IPython.embed(global_ns=globals(), local_ns=locals())  # NOQA


if __name__ == "__main__":
    main()
