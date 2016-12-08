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
            motorFlywheel.setPower(flywheelPower);
        }
        if(gamepad2.y)
        {
            motorFlywheel.setPower(0.0);
        }
    }

    void shootBall()
    {
        if(gamepad2.dpad_left)
        {
            servoFinger.setPosition(ServoPositions.FINGER_EXTEND.pos);
            fingerTimer.reset();
        }
        if(fingerTimer.milliseconds() > 250)
            servoFinger.setPosition(ServoPositions.FINGER_RETRACT.pos);
    }

    void setFlywheelAngleAndSpeed()
    {
        double distanceToGoal;

        // If one of the desired buttons is pressed, set the distance. Otherwise return and do nothing
        if(gamepad1.x)
            distanceToGoal = 0.4;
        else if(gamepad1.y)
            distanceToGoal = 0.8;
        else if(gamepad1.b)
            distanceToGoal = 1.2;
        else
            return;

        // Acceleration due to gravity
        double g = 9.8;
        // Calculate vertical velocity using energy: vY = sqrt(2gh) where h is 2 meters
        double vY = Math.sqrt(2 * g * 2);
        // Calculate the time taken to reach the top of the arc using: a = v / t
        double flyTime = vY / g;
        // Calculate horizontal velocity needed to reach goal: vX = x / t
        double vX= distanceToGoal / flyTime;
        // vTotal = sqrt(vX ^ 2 + vY ^ 2)
        double velocity = Math.sqrt(Math.pow(vX, 2) + Math.pow(vY, 2));
        // Calculate angle required to shoot into goal. vX and vY are flipped to get angle from vertical
        double angle = Math.atan2(vX, vY);

        // Calculate motor power and servo position based on velocity and angle required
        double power = Range.scale(velocity, 0, 15, 0, 1); // 15 m/s is max launching speed of flywheel
        double position = Range.scale(angle, 0, 60, 0, 1); // Servo has a 3:1 gear ratio, so adjust angle

        // Set motor power and servo position
        flywheelPower = power;
        servoFlywheelAngle.setPosition(position);
    }
}
