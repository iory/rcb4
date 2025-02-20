import traceback

import rospy
import serial


def serial_call_with_retry(func, *args, max_retries=1, retry_interval=0.1, **kwargs):
    """Wraps a serial communication function with error handling and retry logic.

    Parameters
    ----------
    func : callable
        The serial communication function to call.
    *args
        Positional arguments to pass to the function.
    max_retries : int, optional
        Maximum number of retry attempts (default is 3).
    retry_interval : float, optional
        Time to wait between retries, in seconds (default is 0.1 seconds).
    **kwargs
        Keyword arguments to pass to the function.

    Returns
    -------
    Any
        The return value of the function, if successful.
        Returns None if all attempts fail.

    Raises
    ------
    serial.SerialException, OSError
        Logs the errors and retries if any exception occurs. Logs an error if all retries fail.
    """
    attempts = 0
    while attempts < max_retries:
        try:
            return func(*args, **kwargs)
        except (KeyboardInterrupt, SystemExit):
            # Re-raise these exceptions to allow program termination
            raise
        except (
            serial.SerialException,
            OSError,
            ValueError,
            IndexError,
            RuntimeError,
        ) as e:
            rospy.logerr(f"[{func.__name__}] Error: {e}")
            attempts += 1
            rospy.sleep(retry_interval)
        except Exception as e:
            # Catch all other exceptions and log them
            rospy.logerr(f"[{func.__name__}] Exception occurred: {e}")
            rospy.logerr(f"Stack trace:\n{traceback.format_exc()}")
            attempts += 1
            rospy.sleep(retry_interval)
    rospy.logerr(f"[{func.__name__}] Failed after {max_retries} attempts.")
    return None
