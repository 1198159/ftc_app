package org.firstinspires.ftc.team8923;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes for the CapBot
 */
abstract class MasterTeleOp extends Master
{
    void driveMecanumTeleOp()
    {
        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;

        double angle = Math.atan2(y, x);
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        driveMecanum(angle, power, turnPower);
    }
}
