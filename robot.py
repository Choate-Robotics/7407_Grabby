import commands2
import ctre
import wpilib
from robotpy_toolkit_7407 import Subsystem

import command
import config
import constants
import sensors
import subsystem
import utils
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

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(Robot.drivetrain)
        )

    def teleopPeriodic(self):
        pass

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
