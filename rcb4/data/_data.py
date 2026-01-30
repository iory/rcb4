from collections import namedtuple
from distutils.version import StrictVersion
import os
import os.path as osp
import subprocess

from colorama import Fore
from colorama import Style

from rcb4._lazy_imports import _lazy_gdown
from rcb4._lazy_imports import _lazy_gdown_version

data_dir = osp.abspath(osp.dirname(__file__))
_default_cache_dir = osp.expanduser("~/.rcb4")


ELFINFO = namedtuple("ELFINFO", ["url", "md5sum"])

elf_infos = {
    "v0.6.2": ELFINFO(
        "https://github.com/iory/rcb4/releases/download/v0.6.2.elf/v0.6.2.elf",
        "9205b34ce4d81b87c4f11e6139f96b17",
    ),
    "v0.6.3": ELFINFO(
        "https://github.com/iory/rcb4/releases/download/v0.6.3.elf/v0.6.3.elf",
        "93be022a09d117e5c4338e24a8131901",
    ),
    "v0.6.4": ELFINFO(
        "https://github.com/iory/rcb4/releases/download/v0.6.4.elf/v0.6.4.elf",
        "e1c386f350f780536a51f3bb24cc0cd1",
    ),
    "v0.6.5": ELFINFO(
        "https://github.com/iory/rcb4/releases/download/v0.6.5.elf/v0.6.5.elf",
        "2ec2d0cde4c3d5c5a9da2f705d3c6c50",
    ),
}


def get_latest_version(elf_infos):
    latest_version = sorted(elf_infos.keys())[-1]
    return latest_version, elf_infos[latest_version]


def get_cache_dir():
    return os.environ.get("RCB4_CACHE_DIR", _default_cache_dir)


def kondoh7_elf(version="latest"):
    if version == 'latest':
        version, _ = get_latest_version(elf_infos)
    if version not in elf_infos:
        raise RuntimeError(
            f"Invalid armh7 version. Valid versions are {list(elf_infos.keys())}"
        )
    elf_info = elf_infos[version]
    target_path = osp.join(get_cache_dir(), "elf", version + ".elf")
    if osp.exists(target_path):
        return target_path
    gdown_version = _lazy_gdown_version()
    gdown = _lazy_gdown()
    try:
        if StrictVersion(gdown_version) < StrictVersion("5.1.0"):
            gdown.cached_download(
                url=elf_info.url, path=target_path, md5=elf_info.md5sum, quiet=True
            )
        else:
            gdown.cached_download(
                url=elf_info.url,
                path=target_path,
                hash=f"md5:{elf_info.md5sum}",
                quiet=True,
            )
    except Exception as e:
        print(Fore.RED + str(e))
        print(
            "Download elf file failed. \nPlease download "
            + Fore.BLUE
            + elf_info.url
            + Fore.RED
            + " and put it to "
            + target_path
        )
        print(Style.RESET_ALL)
    return target_path


def stlink():
    st_flash_path = osp.join(
        get_cache_dir(), "stlink", "stlink-1.7.0", "build", "Release", "bin", "st-flash"
    )
    if osp.exists(st_flash_path):
        return st_flash_path
    url = "https://github.com/stlink-org/stlink/archive/refs/tags/v1.7.0.tar.gz"
    md5sum = "583a506c8e5e65577d623b5ace992fe5"
    target_path = osp.join(get_cache_dir(), "stlink", "v1.7.0.tar.gz")
    target_dir = osp.join(get_cache_dir(), "stlink", "stlink-1.7.0")
    gdown_version = _lazy_gdown_version()
    gdown = _lazy_gdown()
    if StrictVersion(gdown_version) < StrictVersion("5.1.0"):
        gdown.cached_download(
            url=url,
            path=target_path,
            md5=md5sum,
            quiet=True,
            postprocess=gdown.extractall,
        )
    else:
        gdown.cached_download(
            url=url,
            path=target_path,
            hash=f"md5:{md5sum}",
            quiet=True,
            postprocess=gdown.extractall,
        )
    ret = subprocess.run(
        f"cd {target_dir} && make",
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if ret.returncode == 0:
        return st_flash_path
    else:
        raise RuntimeError("Building stlink failed. Notify to maintainer.")
