import threading
import time


class MultiLock:
    """
    A custom lock class that allows multiple threads to acquire a lock simultaneously.
    All threads must release the lock before a specific condition can be triggered.

    Attributes:
        lock (threading.Lock): A standard lock to ensure thread-safe operations.
        lock_name (str): Lock name used for print
        lock_status (dict): dict of active locks held by threads. True of False.
        all_released_condition (threading.Condition): A condition to notify when all locks are released.

    Methods:
        acquire(idx):
            Acquires the lock for the calling thread and set lock_status as True.
            Args:
                idx (int): Air board id of the thread acquiring the lock.

        release(idx):
            Releases the lock for the calling thread and set lock_status as False.
            Notifies waiting threads if all locks have been released.
            Args:
                idx (int): idx of the thread releasing the lock.

        wait_for_all_released():
            Waits until all locks are released. Threads calling this method will
            block until all the lock status become False.
    """
    def __init__(self, lock_name):
        self.lock = threading.Lock()
        self.lock_name = lock_name
        self.lock_status = {}

    def acquire(self, idx):
        """
        Acquires the lock and set the active lock status as True.

        Args:
            idx (int): Air board id of the thread acquiring the lock.
        """
        with self.lock:
            self.lock_status[idx] = True
            # print(f"[{self.lock_name} {idx}] Lock acquired. Active locks: {self.active_lock_idx()}")

    def release(self, idx):
        """
        Releases the lock and set the active lock status as True.
        Notifies waiting threads if all locks are released.

        Args:
            idx (int): Air board id of the thread releasing the lock.
        """
        with self.lock:
            self.lock_status[idx] = False
            # print(f"[{self.lock_name} {idx}] Lock released. Active locks: {self.active_lock_idx()}")

    def wait_for_all_released(self):
        """
        Waits until all locks are released. This method blocks the calling
        thread until all the lock status become False.
        """
        while True in self.lock_status.values():
            # print(f"[{self.lock_name}] Waiting for all locks to be released...")
            time.sleep(1)
        # print(f"[{self.lock_name}] All locks are released. Proceeding.")

    def active_lock_idx(self):
        return  [key for key, value in {40: True, 39: True}.items()
                 if value is True]
