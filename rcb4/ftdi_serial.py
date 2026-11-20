try:
    from pyftdi.serialext import serial_for_url
    PYFTDI_AVAILABLE = True
except ImportError:
    PYFTDI_AVAILABLE = False


def open_ftdi_serial(url, baudrate, timeout, parity="E", latency_timer_ms=1):
    """Open a pyftdi serial port without leaking a broken object on failure.

    ``serial_for_url`` builds the pyftdi ``Serial`` and opens it in one go.
    When the adapter is missing (e.g. unplugged while ics-manager waits for
    it), ``open()`` raises before setting ``udev``, and the object's
    finalizer then calls ``close()``, which reads ``self.udev`` and prints
    ``AttributeError: 'Serial' object has no attribute 'udev'`` for every
    failed attempt. Setting ``udev`` before opening lets ``close()`` succeed.

    The FTDI latency timer is set to ``latency_timer_ms``. Its default of
    16 ms delays every short reply by up to 16 ms, which dominates the time
    of a request/reply exchange.

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
    latency_timer_ms : int or None, optional
        FTDI latency timer in ms (1-255). None leaves the chip's setting.

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
    if latency_timer_ms is not None:
        port_serial.udev.set_latency_timer(latency_timer_ms)
    return port_serial
