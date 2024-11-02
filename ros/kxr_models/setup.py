#!/usr/bin/env python

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(
    packages=["kxr_models"],
    package_dir={"": "python"},
)

setup(**d)
