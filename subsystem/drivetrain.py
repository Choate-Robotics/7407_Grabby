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

from robotpy_toolkit_7407.utils.math import *
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
    name: str = "node"

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        current_pos = self.encoder.getAbsolutePosition() - self.encoder_zeroed_absolute_pos
        self.m_turn.set_sensor_position(current_pos * constants.drivetrain_turn_gear_ratio)
        self.set_motor_angle(current_pos)

    def raw_output(self, power):
        self.m_move.set_raw_output(power)
    
    def _set_angle(self, target_angle: radians, initial_angle: radians):
        target_sensor_angle, flipped, flip_sensor_offset = self._resolve_angles(self, target_angle, initial_angle)

        if self.name == "front_right":
            print("TARGET SENSOR ANGLE: ", target_sensor_angle)
        # target_sensor_angle -= self.motor_sensor_offset
        if self.name == "front_right":
            print("OFFSET: ", print(self.motor_sensor_offset))
            print("TARGET SENSOR AFTER: ", target_sensor_angle)

        if flipped:
            if self.name == "front_right":
                print("FLIPPED--------------------------")
            self.motor_reversed = not self.motor_reversed
            self.motor_sensor_offset += flip_sensor_offset

        # print("FINAL ANGLE CALCULATED ", target_sensor_angle)
        if self.name == "front_right":
            print("TARGET FIN: ", target_sensor_angle)
        self.set_motor_angle(target_sensor_angle)

    @staticmethod
    def _resolve_angles(self, target_angle: radians, initial_angle: radians) -> tuple[float, bool, float]:
        """
        :param target_angle: Target node angle
        :param initial_angle: Initial node sensor angle
        :return: (target_sensor_angle, flipped, flip_sensor_offset)
        """

        # Actual angle difference in radians
        diff = bounded_angle_diff(initial_angle, target_angle)
        if self.name == "front_right":
            # print("TARGET ANGLE IS ", target_angle)
            # print("INITIAL ANGLE IS ", initial_angle)
            # print("DIFFERENCE IS ", diff)
            print("ANGLE CALCULATED IS ", diff + initial_angle)
        # Should we flip
        if abs(diff) > 0.65 * math.pi:
            flip_sensor_offset = math.pi if diff > 0 else -math.pi
            if self.name == "front_right":
                pass
                # print("OFFSET ", flip_sensor_offset)
            diff -= flip_sensor_offset
            return diff + initial_angle, True, flip_sensor_offset

        return diff + initial_angle, False, 0
    
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
        if self.name == "front_right":
            pass
        #     print("current angle ", (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
        # * 2
        # * math.pi)
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

SwerveNode._set_angle = SparkMaxSwerveNode._set_angle
SwerveNode._resolve_angles = SparkMaxSwerveNode._resolve_angles
class Drivetrain(SwerveDrivetrain):
    n_front_right = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        wpilib.AnalogEncoder(0),
        encoder_zeroed_absolute_pos=0.106,
        turn_reversed=True,
        drive_reversed=True,
    )
    n_back_right = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        wpilib.AnalogEncoder(3),
        encoder_zeroed_absolute_pos=0.414,
        turn_reversed=True,
        drive_reversed=True,
        name="front_right"
    )
    n_front_left = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        wpilib.AnalogEncoder(1),
        encoder_zeroed_absolute_pos=0.476,
        turn_reversed=True
    )
    n_back_left = SparkMaxSwerveNode(
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