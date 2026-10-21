#!/usr/bin/env python
"""Open an RCB-4 / RCB-4mini board and drop into Python with it as ``ri``.

    rcb4-shell
    rcb4-shell --port /dev/ttyUSB0

The board is opened as RCB4Interface.open() does by default: run flat, the
servos that answer assigned and left free. IPython is used if it is
installed, the plain Python prompt otherwise.
"""

import argparse
import code
import sys

import serial

from rcb4.rcb4interface import com_baudrates
from rcb4.rcb4interface import RCB4Interface

BANNER = """
ri is the board. Servos are named by ICS number (2 x servo ID + port,
odd = SIO5-8); angles are in degrees.

  ri.print_servos()                  every servo: angle, target, torque
  ri.angle_vector()                  current angles
  ri.hold()                          torque on, where each servo is now
  ri.angle_vector([30], [5], time=1.0)   ICS 5 to 30 deg in 1 s
  ri.wait_interpolation()            wait until the move has arrived
  ri.neutral(time=2.0)               every servo to 0 deg
  ri.free()                          torque off

Servos that are free do not move: hold() them first.
"""


def open_board(port):
    interface = RCB4Interface()
    if port is None:
        if interface.auto_open():
            return interface
    else:
        for baudrate in com_baudrates:
            try:
                if interface.open(port=port, baudrate=baudrate):
                    return interface
            except serial.SerialException:
                pass
            interface.close()
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", "-p", default=None,
                        help="serial port or ftdi:// URL; default: the first "
                             + "Dual USB Adapter whose board answers")
    args = parser.parse_args()

    ri = open_board(args.port)
    if ri is None:
        print("No RCB-4 board answered.", file=sys.stderr)
        sys.exit(1)
    ri.print_servos()
    namespace = {"ri": ri}
    try:
        from IPython import start_ipython
    except ImportError:
        code.interact(banner=BANNER + "(IPython is not installed: plain Python prompt)",
                      local=namespace)
        return
    print(BANNER)
    start_ipython(argv=[], user_ns=namespace)


if __name__ == "__main__":
    main()
