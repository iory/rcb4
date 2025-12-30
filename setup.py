import shlex
import subprocess
import sys

from setuptools import find_packages
from setuptools import setup

version = '0.1.3'


if sys.argv[-1] == "release":
    # Release via github-actions.
    commands = [
        f"git tag v{version:s}",
        "git push origin main --tag",
    ]
    for cmd in commands:
        print(f"+ {cmd}")
        subprocess.check_call(shlex.split(cmd))
    sys.exit(0)


setup_requires = []

with open("requirements.txt") as f:
    install_requires = []
    for line in f:
        req = line.split("#")[0].strip()
        install_requires.append(req)

setup(
    name="rcb4",
    version=version,
    description="A python library for RCB4",
    author="Iori Yanokura",
    author_email="yanokura@jsk.imi.i.u-tokyo.ac.jp",
    url="https://github.com/iory/rcb4",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    license="MIT",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "Natural Language :: English",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: Implementation :: CPython",
    ],
    packages=find_packages(),
    zip_safe=False,
    setup_requires=setup_requires,
    install_requires=install_requires,
    entry_points={
        "console_scripts": [
            "rcb4-write-firmware=rcb4.apps.write_firmware:main",
            "armh7-tools=rcb4.apps.armh7_tool:main",
            "ics-manager=rcb4.apps.ics_manager:main",
        ],
    },
)
