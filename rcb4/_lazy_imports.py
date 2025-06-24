_pkg_resources = None
_gdown = None
_gdown_version = None


def _lazy_pkg_resources():
    global _pkg_resources
    if _pkg_resources is None:
        import pkg_resources
        _pkg_resources = pkg_resources
    return _pkg_resources


def _lazy_gdown():
    global _gdown
    if _gdown is None:
        import gdown
        _gdown = gdown
    return _gdown


def _lazy_gdown_version():
    global _gdown_version
    if _gdown_version is None:
        _gdown_version = _lazy_pkg_resources().get_distribution('gdown').version
    return _gdown_version
