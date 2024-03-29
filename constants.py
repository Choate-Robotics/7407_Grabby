import math

from robotpy_toolkit_7407.motors.rev_motors import (
    rev_sensor_accel_unit,
    rev_sensor_unit,
    rev_sensor_vel_unit
)

from robotpy_toolkit_7407.utils.units import deg, ft, hour, inch, m, mile, rad, rev, s

period = 0.03

# --- DRIVETRAIN ---
# drivetrain_turn_gear_ratio = ((8.16 * 4096)/(2*math.pi) * rev_sensor_unit / rad).asNumber()
drivetrain_turn_gear_ratio = 12.732
drivetrain_move_gear_ratio = ((544318 * rev_sensor_unit) / (511 * inch)).asNumber(
    rad / m
)

drivetrain_move_gear_ratio_as_rotations_per_meter = drivetrain_move_gear_ratio / (2 * math.pi)


track_width = (24.2 * inch).asNumber(m)

# TODO Maybe change these
drivetrain_max_vel = (200 * mile / hour).asNumber(m / s)
drivetrain_target_max_vel = (20 * mile / hour).asNumber(m / s)
drivetrain_max_angular_vel = (20 * rev / s).asNumber(rad / s)
drivetrain_max_climb_vel = (10 * mile / hour).asNumber(m / s)
input_ramp_limit = .2