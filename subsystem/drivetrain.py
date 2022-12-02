import math
import time
from dataclasses import dataclass

import ctre
import rev
import wpilib
from ctre import Pigeon2
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
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

from robotpy_toolkit_7407.sensors.gyro import ADIS16448

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
    encoder: ctre.CANCoder
    encoder_zeroed_absolute_pos: float = 0
    drive_reversed: bool = False
    turn_reversed: bool = False

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        current_absolute_pos = self.encoder.getAbsolutePosition()
        sensor_diff = self.encoder_zeroed_absolute_pos-current_absolute_pos

        print(f"T_CURRENT: {current_absolute_pos*2*math.pi} E_CURRENT: {self.get_current_motor_angle()} T_DISTANCE: {sensor_diff*2*math.pi} T_DESIRED: {self.encoder_zeroed_absolute_pos*2*math.pi} E_DESIRED: {sensor_diff*2*math.pi + self.get_current_motor_angle()}")

        pos = sensor_diff*2*math.pi + self.get_current_motor_angle()
        self.m_turn.set_target_position((pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio)
        # time.sleep(1)
        # self.m_turn.set_sensor_position(0)

    # reposition the wheels
    def set_motor_angle(self, pos: radians):
        if self.turn_reversed:
            pos *= -1
        self.m_turn.set_target_position((pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio)

    def get_current_motor_angle(self) -> radians:
        return self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: meters_per_second):
        if self.drive_reversed:
            vel *= -1
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


class ADIS16448_Wrapper(SwerveGyro):
    def __init__(self):
        self._gyro = ADIS16448.GyroADIS16448()

    def init(self):
        self.reset_angle()

    def get_robot_heading(self) -> radians:
        return self._gyro.angle

    # # reset the gyro
    # def reset_angle(self):
    #     self._gyro.setYaw(0)


class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        wpilib.AnalogEncoder(1),
        encoder_zeroed_absolute_pos=0.990,
        turn_reversed=True,
        drive_reversed=True
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        wpilib.AnalogEncoder(0),
        encoder_zeroed_absolute_pos=0.578,
        turn_reversed=True,
    )

    n_10 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        wpilib.AnalogEncoder(2),
        encoder_zeroed_absolute_pos=0.58,
        turn_reversed=True,
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        wpilib.AnalogEncoder(3),
        encoder_zeroed_absolute_pos=0.414,
        turn_reversed=True,
        drive_reversed=True
    )

    # gyro = PigeonIMUGyro()
    gyro = ADIS16448_Wrapper()
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
