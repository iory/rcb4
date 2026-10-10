# flake8: noqa

import sys


if sys.version_info[0] < 3:
    print("\033[91mThis package is not supported in Python 2.\033[0m")
    sys.exit(1)


_version = None
__all__ = []


def __getattr__(name):
    global _version
    if name == "__version__":
        if _version is None:
            from importlib.metadata import version
            _version = version('rcb4')
        return _version
    raise AttributeError(
        "module {} has no attribute {}".format(__name__, name))


def __dir__():
    return __all__ + ['__version__']
