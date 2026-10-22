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


def convert_adc_to_newton(adc_value, resistor_k=15.0):
    """
    Convert ADC value to Force [N] using HSFPAR003A parameters.

    Parameters
    ----------
    adc_value : float
        The raw ADC value (or sum of ADC values).
    resistor_k : float, optional
        The resistance value of the gain resistor Rg in kOhm.
        Default is 15.0 kOhm.

    Returns
    -------
    force_n : float
        Calculated force in Newtons.

    Notes
    -----
    Calculation Logic:
        1. Gain (G) = 1 + 100k / Rg
        2. Sensitivity (S) = 3.7 mV/V/N = 0.0037 V/V/N
        3. Vdd = 3.3 V
        4. Resolution = 12 bit (4096)

        Force per LSB (K) is derived from:
        K = (Vdd / Resolution) / (Gain * S * Vdd)

        Force [N] = adc_value * K
    """
    # Constants from datasheet and circuit
    vdd = 3.3
    sensitivity = 0.0037  # V/V/N
    adc_resolution = 4096.0
    gain = 1.0 + (100.0 / resistor_k)
    force_factor = (vdd / adc_resolution) / (gain * sensitivity * vdd)
    force_n = adc_value * force_factor
    return force_n
