import serial.tools.list_ports
import usb1


def get_vendor_id_and_product_id(port):
    """Retrieve the vendor ID and product ID for the specified serial port.

    Parameters
    ----------
    port : str
        The serial port for which to retrieve the vendor and product IDs.

    Returns
    -------
    tuple
        A tuple containing the vendor ID and product ID.

    Raises
    ------
    ValueError
        If the USB device information cannot be found for the specified port.
    """
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.device == port:
            if p.vid is not None and p.pid is not None:
                return p.vid, p.pid
            else:
                raise ValueError(f"No USB device information found for port {port}")
    raise ValueError(f"Could not find USB device information for port {port}")


def reset_usb_device(port):
    """Reset the USB device corresponding to the specified serial port.

    Parameters
    ----------
    port : str
        The serial port whose USB device needs to be reset.

    Raises
    ------
    ValueError
        If the USB device cannot be found.
    """
    # Get vendor ID and product ID from the port
    vendor_id, product_id = get_vendor_id_and_product_id(port)
    # Create a USB context and open the device by vendor ID and product ID
    context = usb1.USBContext()
    handle = context.openByVendorIDAndProductID(vendor_id, product_id, skip_on_error=True)
    if handle is None:
        raise ValueError("Device not found")
    # Reset the USB device
    handle.resetDevice()
    print(f"USB device {vendor_id:04x}:{product_id:04x} has been reset.")


def reset_serial_port(port):
    """Reset the serial port if it is a USB serial port.

    Parameters
    ----------
    port : str
        The serial port to be reset.

    Raises
    ------
    ValueError
        If the port is not a USB serial port.
    """
    try:
        # Try to get vendor ID and product ID to determine if it's a USB device
        reset_usb_device(port)
    except ValueError as e:
        # If no USB device information is found,
        # it's likely not a USB serial port
        raise ValueError(f"{port} cannot be reset via USB reset. {e}")
