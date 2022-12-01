from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Drivetrain


def curve_abs(x):
    return x**2.4


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    driver_centric = False
    driver_centric_reversed = False

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value,
            self.subsystem.axis_dy.value,
            -self.subsystem.axis_rotation.value,
        )

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
        self.subsystem.n_00.set(0, 0)
        self.subsystem.n_01.set(0, 0)
        self.subsystem.n_10.set(0, 0)
        self.subsystem.n_11.set(0, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
