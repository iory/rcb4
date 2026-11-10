"""Wire protocol of Kondo control boards, without any I/O.

* :mod:`rcb4.protocol.framing` -- length byte + checksum, ack, version and the
  batch servo move, shared by the RCB-4mini and the ARM-H7.
* :mod:`rcb4.protocol.rcb4mini` -- the RCB-4mini's commands and memory layout.
* :mod:`rcb4.protocol.ics` -- the ICS 3.5 / 3.6 commands a host sends directly
  to a servo, and the loopback and reply-header checks of their answers.

Nothing here opens a port, sleeps, or converts to angles, so it imports only
the standard library and can be used by any driver (rcb4's own
:class:`~rcb4.rcb4interface.RCB4Interface`, or another host's own) as the single
source of truth for the bytes on the wire.
"""
