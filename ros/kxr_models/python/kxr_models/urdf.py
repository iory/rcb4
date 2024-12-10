import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from kxr_models.compress import make_tarfile
from kxr_models.md5sum_utils import checksum_md5
from kxr_models.path import resolve_filepath


def aggregate_urdf_mesh_files(input_urdf_path, output_directory, compress=False):
    urdf_path = Path(input_urdf_path)
    if not urdf_path.exists():
        raise OSError(f"No such urdf {urdf_path}")
    robot_md5sum = checksum_md5(urdf_path)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(urdf_path, parser)
    root = tree.getroot()

    output_directory = Path(output_directory)

    output_dir = output_directory / robot_md5sum
    os.makedirs(output_dir, mode=0o777, exist_ok=True)

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename is None:
            continue
        abs_path = resolve_filepath(filename, urdf_path)
        if abs_path is not None:
            mesh_robot_md5sum = checksum_md5(abs_path)
            suffix = Path(abs_path).suffix
            shutil.copy(abs_path, output_dir / f"{mesh_robot_md5sum}{suffix}")
            if suffix == ".obj":
                mtl_path = Path(abs_path).with_suffix(".mtl")
                if mtl_path.exists():
                    shutil.copy(mtl_path, output_dir / mtl_path.name)
            mesh.set(
                "filename",
                f"package://kxr_models/models/urdf/{robot_md5sum}/{mesh_robot_md5sum}{suffix}",
            )
    for mesh in root.findall(".//texture"):
        filename = mesh.get("filename")
        if filename is None:
            continue
        abs_path = resolve_filepath(filename, urdf_path)
        if abs_path is not None:
            mesh_robot_md5sum = checksum_md5(abs_path)
            suffix = Path(abs_path).suffix
            shutil.copy(abs_path, output_dir / f"{mesh_robot_md5sum}{suffix}")
            mesh.set(
                "filename",
                f"package://kxr_models/models/urdf/{robot_md5sum}/{mesh_robot_md5sum}{suffix}",
            )
    output_urdf_path = output_dir / "{}.urdf".format(root.get("name"))
    tree.write(output_urdf_path)
    if compress:
        return make_tarfile(output_dir)
    return output_urdf_path


def replace_urdf_path(input_urdf_path, output_urdf_path):
    urdf_path = Path(input_urdf_path)
    if not urdf_path.exists():
        raise OSError(f"No such urdf {urdf_path}")
    robot_md5sum = checksum_md5(urdf_path)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(urdf_path, parser)
    root = tree.getroot()
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename is None:
            continue
        abs_path = resolve_filepath(filename, urdf_path)
        if abs_path is not None:
            mesh_robot_md5sum = checksum_md5(abs_path)
            suffix = Path(abs_path).suffix
            mesh.set(
                "filename",
                f"package://kxr_models/models/urdf/{robot_md5sum}/{mesh_robot_md5sum}{suffix}",
            )
    for mesh in root.findall(".//texture"):
        filename = mesh.get("filename")
        if filename is None:
            continue
        abs_path = resolve_filepath(filename, urdf_path)
        if abs_path is not None:
            mesh_robot_md5sum = checksum_md5(abs_path)
            suffix = Path(abs_path).suffix
            mesh.set(
                "filename",
                f"package://kxr_models/models/urdf/{robot_md5sum}/{mesh_robot_md5sum}{suffix}",
            )
    tree.write(output_urdf_path)
    return output_urdf_path
