import commands2
import wpilib

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

        commands2.CommandScheduler.getInstance().setPeriod(constants.period)
        Robot.drivetrain.init()

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().schedule(
            command.DrivetrainZero(Robot.drivetrain).andThen(
                command.DriveSwerveCustom(Robot.drivetrain)
            )
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
