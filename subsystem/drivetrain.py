import math
from dataclasses import dataclass
import rev
import wpilib
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
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
    encoder: wpilib.AnalogEncoder
    encoder_zeroed_absolute_pos: float = 0
    drive_reversed: bool = False
    turn_reversed: bool = False

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        current_pos = self.encoder.getAbsolutePosition()
        zeroed_pos = self.encoder_zeroed_absolute_pos
        new_pos = current_pos - zeroed_pos
        self.m_turn.set_sensor_position(new_pos * constants.drivetrain_turn_gear_ratio)

    def raw_output(self, power):
        self.m_move.set_raw_output(power)
    
    def set_motor_angle(self, pos: radians):
        if self.turn_reversed:
            pos *= -1
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def direct_set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def get_current_motor_angle(self) -> radians:
        return (
            (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
            * 2
            * math.pi
        )

    def set_motor_velocity(self, vel: meters_per_second):
        if self.drive_reversed:
            vel *= -1
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> radians_per_second:
        return self.m_move.get_sensor_velocity() / constants.drivetrain_move_gear_ratio

    def get_drive_motor_traveled_distance(self) -> meters:
        sensor_position = self.m_move.get_sensor_position()
        if self.drive_reversed:
            sensor_position *= -1

        return (
            sensor_position
            / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_turn_motor_angle(self) -> radians:
        return (
            (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
            * 2
            * math.pi
        )

class Drivetrain(SwerveDrivetrain):
    n_back_left = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        wpilib.AnalogEncoder(0),
        encoder_zeroed_absolute_pos=0.578,
        turn_reversed=True,
    )
    n_back_right = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        wpilib.AnalogEncoder(3),
        encoder_zeroed_absolute_pos=0.414,
        turn_reversed=True,
        drive_reversed=True,
    )
    n_front_left = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        wpilib.AnalogEncoder(1),
        encoder_zeroed_absolute_pos=0.990,
        turn_reversed=True,
        drive_reversed=True,
    )
    n_front_right = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        wpilib.AnalogEncoder(2),
        encoder_zeroed_absolute_pos=0.58,
        turn_reversed=True,
    )
    
    gyro = PigeonIMUGyro_Wrapper(13)
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

    def get_axis_dx(self):
        return self.axis_dx.value

    def get_axis_dy(self):
        return self.axis_dy.value
    
    def get_axis_rotation(self):
        return self.axis_rotation.value
