package org.firstinspires.ftc.team8923;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes for the CapBot
 */
abstract class MasterTeleOp extends Master
{
    void driveMecanumTeleOp()
    {
        if(gamepad1.dpad_down)
            slowModeDivisor = 4.0;
        else if(gamepad1.dpad_up)
            slowModeDivisor = 1.0;

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        driveMecanum(angle, power, turnPower);
    }

    void runLift()
    {
        if(gamepad2.dpad_up)
            motorLift.setPower(1.0);
        else if(gamepad2.dpad_down)
            motorLift.setPower(-1.0);
        else
            motorLift.setPower(0);
    }

    void grabCapBall()
    {
        if(gamepad2.right_bumper)
        {
            servoGrabberRight.setPosition(ServoPositions.GRABBER_GRAB.pos);
            servoGrabberLeft.setPosition(ServoPositions.GRABBER_GRAB.pos);
        }
        if(gamepad2.left_bumper)
        {
            servoGrabberRight.setPosition(ServoPositions.GRABBER_RELEASE.pos);
            servoGrabberLeft.setPosition(ServoPositions.GRABBER_RELEASE.pos);
        }
    }

    void runCollector()
    {
        motorCollector.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }

    void runLauncher()
    {
        if(gamepad2.b)
        {
            motorFlywheel.setPower(1.0);
        }
        if(gamepad2.y)
        {
            motorFlywheel.setPower(0.0);
        }
    }

    void activateFinger()
    {
        if(gamepad2.dpad_left)
        {
            servoFinger.setPosition(ServoPositions.FINGER_EXTEND.pos);
            servoFinger.setPosition(ServoPositions.FINGER_RETRACT.pos);
        }
    }

    void actuateLauncher()
    {

    }
}
