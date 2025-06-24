import importlib
import threading


class ModuleLoader:
    def __init__(self, module_path, attribute=None):
        self._module = None
        self._load_complete = threading.Event()
        self._module_path = module_path
        self._attribute = attribute
        self._thread = threading.Thread(target=self._load_module)
        self._thread.daemon = True
        self._thread.start()

    def _load_module(self):
        try:
            module = importlib.import_module(self._module_path)
            if self._attribute:
                self._module = getattr(module, self._attribute)
            else:
                self._module = module
            self._load_complete.set()
        except ImportError as e:
            print(f"Failed to import {self._module_path}: {e}")
            self._module = None
            self._load_complete.set()

    def get_module(self, timeout=None):
        if not self._load_complete.wait(timeout=timeout):
            raise TimeoutError(f"Loading {self._module_path} timed out")
        if self._module is None:
            raise ValueError(f"Failed to load {self._module_path}")
        return self._module


class KXRConfigLoader:
    def __init__(self):
        self._load_complete = threading.Event()
        self._module = None
        self._thread = threading.Thread(target=self._load_module)
        self._thread.daemon = True
        self._thread.start()

    def _load_module(self):
        try:
            from kxr_controller.cfg import KXRParametersConfig as Config
        except Exception:
            print('\x1b[31m'
                + "The imported configuration is outdated. Please run 'catkin build kxr_controller'."
                + '\x1b[39m')
            from kxr_controller.cfg import (
                KXRParameteresConfig as Config,  # spellchecker:disable-line
            )
        self._module = Config
        self._load_complete.set()

    def get_module(self, timeout=None):
        if not self._load_complete.wait(timeout=timeout):
            raise TimeoutError("Loading KXRParametersConfig timed out")
        if self._module is None:
            raise ValueError("Failed to load KXRParametersConfig")
        return self._module


if __name__ == '__main__':
    from datetime import datetime
    start = datetime.now()
    server_loader = ModuleLoader('dynamic_reconfigure.server', 'Server')
    actionlib_loader = ModuleLoader('actionlib')
    server_loader.get_module()
    actionlib_loader.get_module()
    end = datetime.now()
    print(f'load dynamic_reconfigure server and actionlib time: {end - start}')
