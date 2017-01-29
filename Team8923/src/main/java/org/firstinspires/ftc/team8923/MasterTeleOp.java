package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes
 */
abstract class MasterTeleOp extends Master
{
    private ElapsedTime timer = new ElapsedTime();
    private boolean hopperServoMoving = false;

    void driveMecanumTeleOp()
    {
        // Reverse drive if desired
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

    // TODO: Test
    void driveAroundCapBall()
    {
        // Reverse drive if desired
        if(gamepad1.start)
            reverseDrive(false);
        if(gamepad1.back)
            reverseDrive(true);

        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        // Distance from center of robot to center of cap ball
        double radius = 0.5;
        // Maximum rotation rate of robot
        double maxOmega = 1;
        // Find desired rotation rate from scaling joystick
        double omega = Range.scale(turnPower, -1.0, 1.0, -maxOmega, maxOmega);
        // Calculate tangential velocity from equation: v=wr
        double velocity = omega * radius;
        // Either go left or right
        double driveAngle = Math.signum(turnPower) * 90;

        // Give values to  drive method
        driveMecanum(driveAngle, velocity, turnPower);
    }

    // Extends and retracts beacon pusher
    void controlBeaconPusher()
    {
        if(gamepad1.a)
            servoBeaconPusher.setPosition(ServoPositions.BEACON_EXTEND.pos);
        else if(gamepad1.x)
            servoBeaconPusher.setPosition(ServoPositions.BEACON_RETRACT.pos);
    }

    // Runs lift up and down
    void runLift()
    {
        if(gamepad2.dpad_up)
            motorLift.setPower(1.0);
        else if(gamepad2.dpad_down)
            motorLift.setPower(-1.0);
        // If no button is pressed, stop!
        else
            motorLift.setPower(0);
    }

    // Runs collector. Can be run forwards and backwards
    void runCollector()
    {
        // Full speed is too fast
        double speedFactor = 0.6;
        motorCollector.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * speedFactor);
    }

    void controlHopper()
    {
        if (gamepad2.left_bumper || hopperServoMoving)
        {
            hopperServoMoving = true;
            timer.startTime();
            //TODO: add functionality for the catapult to come to a "cocked" position here?
            if(!(timer.milliseconds() > 1000))
            {
                servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH.pos);
                if (timer.milliseconds() < 1000)
                    return;
            }
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
            if(timer.milliseconds() < 2000)
                return;
            timer.reset();
            hopperServoMoving = false;
        }
    }

    void controlCatapult()
    {
        // Only control the catapult if it's done moving
        if(Math.abs(motorCatapult.getCurrentPosition() - motorCatapult.getTargetPosition()) < 20)
        {
            // 1680 ticks per motor revolution geared 3:1
            int ticksPerCycle = 1680 * 3;

            // Go to next arming location
            if(gamepad2.left_bumper)
            {
                catapultCycle++;
                int targetPosition = catapultZero + ticksPerCycle * catapultCycle;
                motorCatapult.setTargetPosition(targetPosition);
            }
            // Run motor forward a bit to launch particle
            else if(gamepad2.right_bumper)
            {
                int targetPosition = motorCatapult.getCurrentPosition() + 2000;
                motorCatapult.setTargetPosition(targetPosition);
            }
        }

        // As of right now, there is not a way to know if the catapult has been zeroed correctly.
        // This code allows the driver to manually control the motor to set a zero location for
        // reference, which is intended to be just before the catapult launches
        if(Math.abs(gamepad2.right_stick_y) > 0.1)
        {
            motorCatapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorCatapult.setPower(-gamepad2.right_stick_y); // Y axis is flipped
        }
        // Return control to motor controller
        else if(motorCatapult.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
        {
            // Set new zero location
            catapultZero = motorCatapult.getCurrentPosition();
            motorCatapult.setTargetPosition(catapultZero);
            catapultCycle = 0;
            motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Motor will need power to move to next target when it's requested, but won't move yet
            // because it's already at the target (zero location)
            motorCatapult.setPower(1.0);
        }
    }
}
