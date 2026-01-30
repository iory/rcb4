import numpy as np
from scipy.optimize import least_squares


def fit_sphere_to_imu_data(data, target_gravity=9.80665):
    X = np.array(data)
    if len(X) < 10:
        raise ValueError("Not enough data points.")

    def error_func(p, x):
        b = p[0:3]
        s = p[3:6]
        corrected = (x - b) * s
        return np.linalg.norm(corrected, axis=1) - target_gravity

    initial_bias = np.mean(X, axis=0)
    initial_guess = np.concatenate([initial_bias, [1.0, 1.0, 1.0]])

    result = least_squares(error_func, initial_guess, args=(X,), loss='soft_l1', f_scale=0.1)
    if not result.success:
        raise RuntimeError(f"Optimization failed: {result.message}")

    return result.x[0:3], result.x[3:6]


def check_data_sufficiency(data, min_range=4.0):
    X = np.array(data)
    ranges = np.ptp(X, axis=0)
    return np.all(ranges > min_range), {"ranges": ranges, "count": len(X)}
