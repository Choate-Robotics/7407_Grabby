from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Drivetrain
import constants
from wpimath.filter import SlewRateLimiter

def curve_abs(x):
    return x**2.4



def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    driver_centric = True
    driver_centric_reversed = False

    def initialize(self) -> None:
        self.dx_limiter = SlewRateLimiter(constants.input_ramp_limit)
        
        self.dy_limiter = SlewRateLimiter(constants.input_ramp_limit)


    def execute(self) -> None:
        

        dx, dy, d_theta = self.dx_limiter.calculate(-self.subsystem.axis_dx.value), self.dy_limiter.calculate(self.subsystem.axis_dy.value), -self.subsystem.axis_rotation.value,

        if abs(d_theta) < 0.15:
            d_theta = 0

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)
        
        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        if DriveSwerveCustom.driver_centric:
            self.subsystem.set_driver_centric(
                (-dy, dx), d_theta * self.subsystem.max_angular_vel
            )
            
        elif DriveSwerveCustom.driver_centric_reversed:
            self.subsystem.set_driver_centric(
                (dy, -dx), d_theta * self.subsystem.max_angular_vel
            )
        else:
            self.subsystem.set_robot_centric(
                (dx, dy), d_theta * self.subsystem.max_angular_vel
            )

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set(0, 0)
        self.subsystem.n_front_right.set(0, 0)
        self.subsystem.n_back_left.set(0, 0)
        self.subsystem.n_back_right.set(0, 0)
        self.dx_limiter.reset(0)
        self.dy_limiter.reset(0)
    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DrivetrainZero(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def zero(self):
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()

        self.subsystem.n_back_left.set_motor_angle(0)
        self.subsystem.n_back_right.set_motor_angle(0)
        self.subsystem.n_front_left.set_motor_angle(0)
        self.subsystem.n_front_right.set_motor_angle(0)
        
        self.dx_limiter.reset(0)
        self.dy_limiter.reset(0)

    def zero_success(self):
        threshold = 0.02

        success = True

        for i in [
            self.subsystem.n_back_left,
            self.subsystem.n_back_right,
            self.subsystem.n_front_left,
            self.subsystem.n_front_right,
        ]:
            if not (
                abs(i.encoder.getAbsolutePosition() - i.encoder_zeroed_absolute_pos)
                < threshold
            ):
                success = False

        return success

    def initialize(self) -> None:
        ...

    def execute(self) -> None:
        self.zero()

    def isFinished(self) -> bool:
        self.subsystem.gyro.reset_angle()
        return self.zero_success()

    def end(self, interrupted: bool) -> None:
        pass
