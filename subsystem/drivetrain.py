import math
from dataclasses import dataclass

import rev
from ctre import CANCoder, Pigeon2
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.sensors.gyro.ADIS16448 import GyroADIS16448
from robotpy_toolkit_7407.subsystem_templates.drivetrain import (
    SwerveDrivetrain,
    SwerveGyro,
    SwerveNode,
)
from robotpy_toolkit_7407.utils.units import (
    deg,
    meters,
    meters_per_second,
    rad,
    radians,
    radians_per_second,
    s,
)
from wpimath.geometry import Pose2d

import constants
from oi.keymap import Keymap

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


@dataclass
class SparkMaxSwerveNode(SwerveNode):
    m_move: SparkMax
    m_turn: SparkMax
    encoder: CANCoder
    encoder_zeroed_absolute_pos_radians: radians = 0

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        current_absolute_pos_radians = math.radians(self.encoder.getAbsolutePosition())
        new_sensor_pos_radians = (
            current_absolute_pos_radians - self.encoder_zeroed_absolute_pos_radians
        )
        self.m_turn.set_sensor_position(
            new_sensor_pos_radians * constants.drivetrain_turn_gear_ratio
        )

    # reposition the wheels
    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(pos * constants.drivetrain_turn_gear_ratio)

    def get_current_motor_angle(self) -> radians:
        return self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: meters_per_second):
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> radians_per_second:
        return self.m_move.get_sensor_velocity() / constants.drivetrain_move_gear_ratio


class PigeonIMUGyro(SwerveGyro):
    def __init__(self):
        self._gyro = Pigeon2(26)
        self._gyro.configMountPose(0, 0, 0)

    def init(self):
        self.reset_angle()

    def get_robot_heading(self) -> radians:
        return math.radians(self._gyro.getYaw())

    # reset the gyro
    def reset_angle(self):
        self._gyro.setYaw(0)


class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG), SparkMax(8, config=TURN_CONFIG), CANCoder(12)
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG), SparkMax(2, config=TURN_CONFIG), CANCoder(9)
    )
    n_10 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG), SparkMax(6, config=TURN_CONFIG), CANCoder(11)
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG), SparkMax(4, config=TURN_CONFIG), CANCoder(10)
    )
    gyro = PigeonIMUGyro()
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    axis_y2 = Keymap.Drivetrain.DRIVE_Y2_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.01
    deadzone_angular_velocity: radians_per_second = (5 * deg / s).asNumber(rad / s)
    start_pose: Pose2d = Pose2d(0, 0, 0)
