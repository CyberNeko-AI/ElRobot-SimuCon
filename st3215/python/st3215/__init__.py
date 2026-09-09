"""ST3215 serial-bus servo driver and auto-calibration (PyO3 bindings).

Example::

    from st3215 import St3215

    d = St3215("/dev/tty.usbserial-XXXX")     # 1 Mbps by default
    motors = d.scan(8)                        # discover IDs 1..=8

    d.set_torque(1, True)                     # enable torque
    d.set_position(1, 2048)                   # move to mid position
    pos = d.read_position(1)
    vel = d.read_velocity(1)
    load = d.read_load(1)

    d.auto_calibrate_elrobot()                # full 8-motor calibration
"""

from ._st3215 import St3215

__all__ = ["St3215"]
__version__ = "0.1.0"
