from robotpy_toolkit_7407.oi import (
    DefaultButton,
    JoystickAxis,
    LogitechController,
    XBoxController,
)

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController


class Controllers:
    DRIVER = 0
    OPERATOR = 1


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[1])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_ROTATION_AXIS = JoystickAxis(
            Controllers.DRIVER, controllerDRIVER.R_JOY[0]
        )
        DRIVE_Y2_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.R_JOY[1])
        RESET_GYRO = DefaultButton(Controllers.DRIVER, controllerDRIVER.A)
        REZERO_MOTORS = DefaultButton(Controllers.DRIVER, controllerDRIVER.B)
        AIM_SWERVE = DefaultButton(Controllers.DRIVER, controllerDRIVER.RT)
        DRIVER_CENTRIC = DefaultButton(Controllers.DRIVER, controllerDRIVER.LB)
        DRIVER_CENTRIC_REVERSED = DefaultButton(Controllers.DRIVER, controllerDRIVER.RB)
