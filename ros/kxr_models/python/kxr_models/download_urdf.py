from importlib.metadata import version
import os.path as osp
import re

import gdown
import rospkg
import rospy

from kxr_models.ros import get_namespace

gdown_version = version("gdown")


def _version_tuple(version_string):
    """Return the leading numeric components of a version string.

    Parameters
    ----------
    version_string : str
        Version string such as ``"5.2.0"`` or ``"5.1.0rc1"``.

    Returns
    -------
    tuple of int
        Three numeric components padded with zeros, e.g. ``(5, 1, 0)``
        for ``"5.1"``.
    """
    numbers = [int(x) for x in re.findall(r"\d+", version_string)[:3]]
    return tuple(numbers + [0] * (3 - len(numbers)))


def download_urdf_mesh_files(namespace=None):
    if namespace is None:
        namespace = get_namespace()
    port = rospy.get_param(namespace + "/model_server_port", 8123)
    server_ip = rospy.get_param(namespace + "/model_server_ip", "localhost")

    urdf_hash = rospy.get_param(namespace + "/urdf_hash", None)
    compressed_urdf_hash = rospy.get_param(
        namespace + "/compressed_urdf_hash", None)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and urdf_hash is None and compressed_urdf_hash is None:
        rate.sleep()
        rospy.loginfo("Waiting rosparam {} and {} set".format(
            namespace + "/urdf_hash", namespace + "/compressed_urdf_hash"))
        urdf_hash = rospy.get_param(namespace + "/urdf_hash", None)
        compressed_urdf_hash = rospy.get_param(namespace + "/compressed_urdf_hash", None)

    server_url = f"http://{server_ip}:{port}/urdf/{urdf_hash}.tar.gz"

    rospack = rospkg.RosPack()
    kxr_models_path = rospack.get_path("kxr_models")

    compressed_urdf_path = osp.join(
        kxr_models_path, "models", "urdf", f"{urdf_hash}.tar.gz"
    )
    while not osp.exists(compressed_urdf_path):
        rospy.loginfo(f"Waiting {compressed_urdf_path} from server")
        rospy.sleep(1.0)
        if _version_tuple(gdown_version) < (5, 1, 0):
            gdown.cached_download(url=server_url,
                                  md5=f'{compressed_urdf_hash}',
                                  path=compressed_urdf_path)
        else:
            gdown.cached_download(url=server_url,
                                  hash=f'md5:{compressed_urdf_hash}',
                                  path=compressed_urdf_path)
    gdown.extractall(osp.join(kxr_models_path, "models", "urdf", f"{urdf_hash}.tar.gz"))
