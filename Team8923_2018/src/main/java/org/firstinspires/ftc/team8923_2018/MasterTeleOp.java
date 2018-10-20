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
        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveMecanum(angle, power, turnPower);
        sendTelemetry();
    }

    void runLift()
    {
        if(gamepad1.left_trigger > 0.35)
        {
            motorLift.setPower(1.0);
        }
        else if(gamepad1.right_trigger > 0.35)
        {
            motorLift.setPower(-1.0);
        }
        else
        {
            motorLift.setPower(0.0);
        }
    }

    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    void sendTelemetry()
    {
        telemetry.addData("left stick x:", gamepad1.left_stick_x);
        telemetry.addData("left stick y:", gamepad1.left_stick_y);
        telemetry.addData("right stick x:", gamepad1.right_stick_x);
        telemetry.addData("right stick y:", gamepad1.right_stick_y);
        telemetry.update();
    }
}
