import commands2
import wpilib
import math
import command
import constants
from oi.OI import OI
from robot_systems import Robot
from oi.keymap import Keymap

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

        # Robot.drivetrain.n_front_left.set_motor_angle(math.pi/2)
        # Robot.drivetrain.n_front_right.set_motor_angle(math.pi/2)
        # Robot.drivetrain.n_back_left.set_motor_angle(math.pi/2)
        # Robot.drivetrain.n_back_right.set_motor_angle(math.pi/2)
        

    def teleopPeriodic(self):
        print("gyro ", Robot.drivetrain.gyro.get_robot_heading())
        # print("n_front_left ", Robot.drivetrain.n_front_left.encoder.getAbsolutePosition())
        # print("n_front_right ", Robot.drivetrain.n_front_right.encoder.getAbsolutePosition())
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
