import commands2
import wpilib

import command
import constants
from oi.OI import OI
from robot_systems import Robot
from oi.keymap import Keymap
from wpilib.shuffleboard import Shuffleboard, ShuffleboardContainer
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
            # command.DriveSwerveCustom(Robot.drivetrain)
        )

    def teleopPeriodic(self):
        # curr_angle = Robot.drivetrain.n_front_left.get_current_motor_angle()
        # dx = Robot.drivetrain.get_axis_dx()
        # dy = Robot.drivetrain.get_axis_dy()
        # theta = Robot.drivetrain.get_axis_rotation()
        # self.pos.append({'dx': dx, 'dy': dy, 'theta': theta, 'curr_angle': curr_angle}) # raw value
        pass

    def autonomousInit(self):
        # previous_angle = 0
        # increasing = False
        # count = 0
        # for pos in self.pos:  
        #     if count == 0:
        #         previous_angle = pos['curr_angle']
        #     elif count == 1:
        #         increasing = pos['curr_angle'] >= previous_angle
        #     else:  
        #         dir = pos['curr_angle'] >= previous_angle
        #         if dir != increasing:
        #             increasing = dir
        #             print("SNAPS BACK AT ", pos['curr_angle'])
        #             print("DX IS ", pos['dx'])
        #             print("DY IS ", pos['dy'])
        #             print("D_THETA IS ", pos['theta'])
        #             print("")
        #     previous_angle = pos['curr_angle']
        #     count += 1
        ...

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Grabby)
