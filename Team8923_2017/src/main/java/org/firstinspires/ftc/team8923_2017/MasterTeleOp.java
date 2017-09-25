package org.firstinspires.ftc.team8923_2017;

/**
 * Holds all code necessary to run the robot in driver controlled mode
 */

public abstract class MasterTeleOp extends Master
{
    void DriveOmni45TeleOp()
    {
        if(gamepad1.dpad_down)
            SlowModeDivisor = 5.0;
        else if(gamepad1.dpad_up)
            SlowModeDivisor = 1.0;

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveOmni45(angle, power, turnPower);
    }
}
