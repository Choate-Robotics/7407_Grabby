import math

import commands2
import wpilib
from robotpy_toolkit_7407 import Subsystem

import command
import constants
from oi.OI import OI
from robot_systems import Robot


class Grabby(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()

        subsystems: list[Subsystem] = list(
            {
                k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem)
            }.values()
        )

        for sub in subsystems:
            sub.init()

        commands2.CommandScheduler.getInstance().setPeriod(constants.period)

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

        wpilib.SmartDashboard.putNumber(
            "Encoder 00", Robot.drivetrain.n_00.encoder.getAbsolutePosition()
        )
        wpilib.SmartDashboard.putNumber(
            "Encoder 01", Robot.drivetrain.n_01.encoder.getAbsolutePosition()
        )
        wpilib.SmartDashboard.putNumber(
            "Encoder 10", Robot.drivetrain.n_10.encoder.getAbsolutePosition()
        )
        wpilib.SmartDashboard.putNumber(
            "Encoder 11", Robot.drivetrain.n_11.encoder.getAbsolutePosition()
        )

    def teleopInit(self):
        def zero():
            Robot.drivetrain.n_00.zero()
            Robot.drivetrain.n_01.zero()
            Robot.drivetrain.n_10.zero()
            Robot.drivetrain.n_11.zero()

        # zero()

        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(Robot.drivetrain)
        )

    def teleopPeriodic(self):
        ...


    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Grabby)
