from typing import List

import numpy as np


def rcb4_checksum(byte_list: List[int]) -> int:
    """Calculates the checksum for a list of byte values.

    The checksum is calculated as the sum of all byte values,
    each masked with 0xff, and then the result is masked with 0xff.

    Parameters
    ----------
    byte_list : list of int
        The list of byte values for which the checksum is to be calculated.

    Returns
    -------
    int
        The calculated checksum.
    """
    # Use Python's int type to avoid numpy overflow warnings
    checksum = 0
    for b in byte_list:
        checksum += int(b) & 0xFF
    return checksum & 0xFF


def rcb4_velocity(v) -> int:
    """Adjust the velocity value to be within the range of 1 to 255.

    This function takes an input velocity `v`, rounds it to the nearest
    integer, and then clamps the value within the range 1 to 255 inclusive.

    Parameters
    ----------
    v : float
        The input velocity value that needs to be adjusted.

    Returns
    -------
    int
        The adjusted velocity value, which is an integer
        in the range from 1 to 255 inclusive.

    Examples
    --------
    >>> rcb4_velocity(0.5)
    1
    >>> rcb4_velocity(127.8)
    128
    >>> rcb4_velocity(300)
    255
    """
    return max(1, min(255, int(round(v))))


def encode_servo_ids_to_nbytes_bin(ids: List[int], num_bytes: int) -> List[int]:
    """Encode a list of servo motor IDs into a specified number of bytes.

    This function takes a list of servo motor IDs (each between 0 and
    num_bytes * 8 - 1) and encodes it into a specified number of bytes.
    Each bit in the byte sequence represents whether a corresponding
    servo motor is active (1) or not (0). The function then splits
    this bit sequence into the specified number of bytes.

    Parameters
    ----------
    ids : List[int]
        A list of integers representing the servo motor IDs.
        Each ID should be less than num_bytes * 8.
    num_bytes : int
        The number of bytes to encode the IDs into.

    Returns
    -------
    List[int]
        A list of integers, where each integer is
        a byte representation (0-255) of the servo motor states.
        The list represents the bit sequence split into the specified
        number of bytes.
    """
    bit_representation = 0
    for idx in ids:
        bit_representation |= 1 << int(idx)
    return [int((bit_representation >> (8 * i)) & 0xFF) for i in range(num_bytes)]


def encode_servo_ids_to_5bytes_bin(ids: List[int]) -> List[int]:
    """Encode a list of servo motor IDs into a 5-byte representation.

    This is a specialized use of the general
    function 'encode_servo_ids_to_nbytes_bin' for encoding
    servo motor IDs into 5 bytes. It's suitable for servo motors
    with IDs ranging from 0 to 39.

    Parameters
    ----------
    ids : List[int]
        A list of integers representing the servo motor IDs.
        Each ID should be in the range 0 to 39.

    Returns
    -------
    List[int]
        A list of 5 integers, each representing a byte of the servo
        motor states.

    Examples
    --------
    >>> encode_servo_ids_to_5bytes_bin([2, 9, 16, 23, 30])
    [4, 2, 1, 128, 64]

    The corresponding binary representation of each byte is:
    '00000100' (for the servo with ID 2),
    '00000010' (for the servo with ID 9),
    '00000001' (for the servo with ID 16),
    '10000000' (for the servo with ID 23),
    '01000000' (for the servo with ID 30).

    This means the servo motors with IDs 2, 9, 16, 23, and 30 are active.
    """
    return encode_servo_ids_to_nbytes_bin(ids, 5)


def encode_servo_positions_to_bytes(fvector: List[float]) -> List[int]:
    """Creates a buffer with servo positions from a float vector.

    Each element in fvector should be in the range 0 to 0xFFFF (65535).
    This range corresponds to the typical range for a 16-bit integer.

    Parameters
    ----------
    fvector : List[float]
        A list of floating-point values representing servo positions.

    Returns
    -------
    bytes: List[int]
        A bytes object representing the low and high bytes of servo positions.
    """
    # Ensure all values are within the 0 to 0xFFFF range
    fvector = np.clip(fvector, 0, 0xFFFF)

    int_positions = np.round(fvector).astype(np.int16)
    return list(int_positions.tobytes())


def encode_servo_velocity_and_position_to_bytes(
    velocities: List[float], positions: List[float]
) -> List[int]:
    """Convert lists of servo velocities and positions to a byte sequence.

    This function takes lists of servo velocities and positions, clips them to
    appropriate ranges, and encodes them as a sequence of bytes. Each velocity
    is represented as a single byte (uint8), and each position is represented
    as two bytes (uint16). The resulting byte sequence alternates between
    a velocity byte and the two bytes of a position.

    Parameters
    ----------
    velocities : List[float]
        A list of servo velocities. Each velocity should be a float, and
        the list will be clipped to the range [1, 255].

    positions : List[float]
        A list of servo positions. Each position should be a float, and
        the list will be clipped to the range [0, 65535].

    Returns
    -------
    List[int]
        A list of bytes representing the interleaved velocities and positions.
        Each velocity is followed by the low byte and then the high byte of the
        corresponding position.

    Examples
    --------
    >>> encode_servo_velocity_and_position_to_bytes(
    [100.5, 200.3, 300.7], [400.9, 500.1, 600.5])
    [100, 145, 1, 200, 244, 1, 255, 88, 2]

    Note
    ----
    The function assumes that the lengths of `velocities` and `positions`
    are the same.
    """
    velocities = np.clip(velocities, 1, 0xFF)
    positions = np.clip(positions, 0, 0xFFFF)

    int_positions = np.round(positions).astype(np.uint16)
    int_velocities = np.round(velocities).astype(np.uint8)

    # Split the 16-bit position data into two 8-bit parts
    positions_low = (int_positions & 0xFF).astype(np.uint8)
    positions_high = (int_positions >> 8).astype(np.uint8)

    velocities_positions = np.vstack(
        (int_velocities, positions_low, positions_high)
    ).T.flatten()
    return list(velocities_positions.tobytes())


def four_bit_to_num(lst: List[int], values: List[int]):
    result = 0
    for index in lst:
        result = (result << 4) | (values[index - 1] & 0x0F)
    return result


def rcb4_servo_svector(ids: List[int], svector: List[float]) -> List[int]:
    return [int(round(v)) & 0xFF for _, v in zip(ids, svector)]
