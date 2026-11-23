"""The companion computer: DroneLab's perception running as its own process, next to the autopilot.

On a real drone this is a Jetson or Raspberry Pi wired to the flight controller's serial port. Here it is a
separate Python process talking MAVLink to ArduCopter SITL. See service.py.
"""
