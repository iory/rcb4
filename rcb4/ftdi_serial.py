try:
    from pyftdi.serialext import serial_for_url
    PYFTDI_AVAILABLE = True
except ImportError:
    PYFTDI_AVAILABLE = False


def open_ftdi_serial(url, baudrate, timeout, parity="E"):
    """Open a pyftdi serial port without leaking a broken object on failure.

    ``serial_for_url`` builds the pyftdi ``Serial`` and opens it in one go.
    When the adapter is missing (e.g. unplugged while ics-manager waits for
    it), ``open()`` raises before setting ``udev``, and the object's
    finalizer then calls ``close()``, which reads ``self.udev`` and prints
    ``AttributeError: 'Serial' object has no attribute 'udev'`` for every
    failed attempt. Setting ``udev`` before opening lets ``close()`` succeed.

    Parameters
    ----------
    url : str
        pyftdi URL such as ``ftdi://Kondo:DUAL USB ADAPTER/1``.
    baudrate : int
        Baud rate to open the port at.
    timeout : float
        Read timeout in seconds.
    parity : str, optional
        Parity, ``"E"`` (even) for ICS and RCB-4.

    Returns
    -------
    serial.SerialBase
        The opened port.
    """
    port_serial = serial_for_url(
        url, baudrate=baudrate, timeout=timeout, do_not_open=True)
    port_serial.udev = None
    port_serial.parity = parity
    port_serial.open()
    return port_serial
