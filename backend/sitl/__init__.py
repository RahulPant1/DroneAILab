"""Phase 5: real autopilot software (ArduPilot ArduCopter) running in SITL on this laptop.

SITL = "software in the loop": the same flight code that runs on a real flight controller,
compiled for the PC and connected to a simulated drone instead of real motors and sensors.
We talk to it with MAVLink (the radio protocol real ground stations use), over 127.0.0.1 only.
"""
