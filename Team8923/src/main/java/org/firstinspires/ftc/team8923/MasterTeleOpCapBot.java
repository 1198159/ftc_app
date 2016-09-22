package org.firstinspires.ftc.team8923;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes for the CapBot
 */
abstract class MasterTeleOpCapBot extends Master
{
    void driveMecanumTeleOp()
    {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        double angle = Math.atan2(y, x);
        double power = calculateDistance(x, y);
        double turnPower = gamepad1.right_stick_x;

        driveMecanum(angle, power, turnPower);
    }
}
