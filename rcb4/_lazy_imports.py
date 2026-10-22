_gdown = None
_gdown_version = None


def _lazy_gdown():
    global _gdown
    if _gdown is None:
        import gdown
        _gdown = gdown
    return _gdown


def _lazy_gdown_version():
    global _gdown_version
    if _gdown_version is None:
        from importlib.metadata import version
        _gdown_version = version('gdown')
    return _gdown_version
