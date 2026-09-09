"""Physical constants used by the vehicle model.

Internal units are SI. Angles in the plant and NLP are radians.
"""

# Standard gravity used to convert specific impulse to exhaust velocity:
#   v_e = I_sp * G0
G0 = 9.81  # m/s^2

# Local gravitational acceleration for the 2D landing problem (flat Earth).
G_EARTH = 9.81  # m/s^2
