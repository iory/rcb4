import numpy as np


def convert_data(raw_data, range_g):
    """Convert raw 16-bit signed integer data

    Convert raw 16-bit signed integer accelerometer data
    to actual values.

    Parameters
    ----------
    raw_data : int or np.ndarray
        Raw data, expected as 16-bit signed integers.
    range_g : float
        The measuring range of the accelerometer in +-g, e.g., 8, 16, 32, 64.

    Returns
    -------
    float or np.ndarray
        The actual values in g. Returns a float if the input is
        a scalar, or np.ndarray if the input is an array.

    Examples
    --------
    >>> convert_data(16000, 8)
    3.91

    >>> convert_data(np.array([16000, -16000]), 8)
    np.array([3.91, -3.91])
    """
    max_value = 32767
    conversion_factor = (range_g * 2) / (max_value * 2)
    return np.array(raw_data) * conversion_factor
