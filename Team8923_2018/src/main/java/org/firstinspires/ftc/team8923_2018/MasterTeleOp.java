package org.firstinspires.ftc.team8923_2018;

abstract class MasterTeleOp extends Master
{
    void driveMecanumTeleOp()
    {
        // Reverse drive if desired
        /*if(gamepad1.start)
            reverseDrive(false);
        if(gamepad1.back)
            reverseDrive(true);*/

        if(gamepad1.dpad_down)
            slowModeDivisor = 3.0;
        else if(gamepad1.dpad_up)
            slowModeDivisor = 1.0;

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        // Hank has asked to just use cardinal directions
        if(Math.abs(x) > Math.abs(y))
            y = 0;
        else
            x = 0;
        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveMecanum(angle, power, turnPower);
    }

    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
}
