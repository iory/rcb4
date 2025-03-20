# flake8: noqa

import sys


if sys.version_info[0] < 3:
    print("\033[91mThis package is not supported in Python 2.\033[0m")
    sys.exit(1)


_pkg_resources = None
_version = None
__all__ = []


def _lazy_pkg_resources():
    global _pkg_resources
    if _pkg_resources is None:
        import pkg_resources
        _pkg_resources = pkg_resources
    return _pkg_resources


def __getattr__(name):
    global _version
    if name == "__version__":
        if _version is None:
            pkg_resources = _lazy_pkg_resources()
            _version = pkg_resources.get_distribution(
                'rcb4').version
        return _version
    raise AttributeError(
        "module {} has no attribute {}".format(__name__, name))


def __dir__():
    return __all__ + ['__version__']
