import math

from robotpy_toolkit_7407.motors.ctre_motors import (
    talon_sensor_accel_unit,
    talon_sensor_unit,
    talon_sensor_vel_unit,
)
from robotpy_toolkit_7407.unum.units import cm
from robotpy_toolkit_7407.utils.units import deg, ft, hour, inch, m, mile, rad, rev, s

period = 0.03

# --- DRIVETRAIN ---
drivetrain_turn_gear_ratio = (3353.33 * talon_sensor_unit / rad).asNumber()
drivetrain_move_gear_ratio = ((544318 * talon_sensor_unit) / (511 * inch)).asNumber(
    rad / m
)

track_width = (24.2 * inch).asNumber(m)

# TODO Maybe change these
drivetrain_max_vel = (20 * mile / hour).asNumber(m / s)
drivetrain_target_max_vel = (7 * mile / hour).asNumber(m / s)
drivetrain_max_angular_vel = (2 * rev / s).asNumber(rad / s)
drivetrain_max_climb_vel = (2 * mile / hour).asNumber(m / s)
