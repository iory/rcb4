from typing import List
from typing import Union


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
    return sum(b & 0xff for b in byte_list) & 0xff


def rcb4_velocity(v):
    return min(255, int(round(v)))


def encode_servo_ids_to_nbytes_bin(
        ids: List[int], num_bytes: int) -> List[int]:
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
        bit_representation |= 1 << idx
    return [(bit_representation >> (8 * i)) & 0xFF for i in range(num_bytes)]


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


def rcb4_servo_positions(
        ids: Union[int, List[int]], fvector: List[float]) -> List[int]:
    """Creates a buffer with servo positions from given ids and float vector.

    Parameters
    ----------
    ids : list or similar
        A list of ids corresponding to servo positions.
    fvector : list
        A list of floating-point values representing servo positions.

    Returns
    -------
    list
        A list of bytes representing the low and high bytes of servo positions.
    """
    if not isinstance(ids, list):
        ids = list(ids)

    fv = [int(round(v)) for v in fvector]
    buf = []
    for d in fv:
        buf.append(d & 0xff)
        buf.append((d >> 8) & 0xff)
    return buf


def four_bit_to_num(lst: List[int], values: List[int]):
    result = 0
    for index in lst:
        result = (result << 4) | (values[index - 1] & 0x0f)
    return result


def rcb4_servo_svector(ids: List[int], svector: List[float]) -> List[int]:
    return [int(round(v)) & 0xff
            for _, v in zip(ids, svector)]
