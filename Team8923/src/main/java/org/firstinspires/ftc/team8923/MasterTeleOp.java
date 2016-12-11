package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes for the CapBot
 */
abstract class MasterTeleOp extends Master
{
    // TODO: Add comments
    private ElapsedTime fingerTimer = new ElapsedTime();
    private double flywheelPower = 0.0;

    void driveMecanumTeleOp()
    {
        if(gamepad1.dpad_down)
            slowModeDivisor = 4.0;
        else if(gamepad1.dpad_up)
            slowModeDivisor = 1.0;

        if(gamepad1.start)
            reverseDrive(false);
        if(gamepad1.back)
            reverseDrive(true);

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        driveMecanum(angle, power, turnPower);
    }

    void controlBeaconPusher()
    {
        if(gamepad1.a)
            servoBeaconPusher.setPosition(ServoPositions.BEACON_EXTEND.pos);
        else if(gamepad1.x)
            servoBeaconPusher.setPosition(ServoPositions.BEACON_RETRACT.pos);
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
        motorCollector.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }

    // We want to be able to shoot from various distances, so this sets the motor power and servo
    // position based on the desired distance
    void controlFlywheel()
    {
        // Set motor power and servo position based on various distances
        if(gamepad2.x)
            setFlywheelPowerAndAngle(400);
        else if(gamepad2.y)
            setFlywheelPowerAndAngle(800);
        else if(gamepad2.b)
            setFlywheelPowerAndAngle(1200);
        // Stop flywheel
        else if(gamepad2.a)
            motorFlywheel.setPower(0.0);

        // Shoot ball
        if(gamepad2.start)
        {
            servoFinger.setPosition(ServoPositions.FINGER_EXTEND.pos);
            fingerTimer.reset();
        }
        // Retract finger after 250 milliseconds
        if(fingerTimer.milliseconds() > 250)
            servoFinger.setPosition(ServoPositions.FINGER_RETRACT.pos);
    }
}
