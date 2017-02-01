package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes
 */
abstract class MasterTeleOp extends Master
{
    private ElapsedTime hopperTimer = new ElapsedTime();
    private boolean hopperServoMoving = false;

    void driveMecanumTeleOp()
    {
        // Reverse drive if desired
        if(gamepad1.start)
            reverseDrive(false);
        if(gamepad1.back)
            reverseDrive(true);

        if(gamepad1.dpad_down)
            slowModeDivisor = 3.0;
        else if(gamepad1.dpad_up)
            slowModeDivisor = 1.0;

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
        double radius = 1.5;
        // Maximum rotation rate of robot
        double maxOmega = 1.0;
        // Find desired rotation rate from scaling joystick
        double omega = Range.scale(turnPower, -1.0, 1.0, -maxOmega, maxOmega);
        // Calculate tangential velocity from equation: v=wr
        double velocity = omega * radius;
        // Either go left or right
        double driveAngle = 90;

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
        if(gamepad2.x)
            servoCapBallHolder.setPosition(ServoPositions.CAP_BALL_HOLD.pos);

        // Run lift up
        if(gamepad2.dpad_up)
        {
            motorLift.setPower(1.0);
            // Retract cap ball holder when raising
            servoCapBallHolder.setPosition(ServoPositions.CAP_BALL_RELEASE.pos);
        }
        // Run lift down
        else if(gamepad2.dpad_down)
            motorLift.setPower(-1.0);
        // If no button is pressed, stop!
        else
            motorLift.setPower(0);

        // TODO: Test
        // Deploy lift semi-autonomously
        if(gamepad1.b)
        {
            servoBeaconPusher.setPosition(ServoPositions.BEACON_EXTEND.pos);
            sleep(250);
            motorLift.setPower(1.0);
            sleep(500);
            motorLift.setPower(-1.0);
            sleep(750);
            motorLift.setPower(0.0);
            servoBeaconPusher.setPosition(ServoPositions.BEACON_RETRACT.pos);
        }
    }

    // Runs collector. Can be run forwards and backwards
    void runCollector()
    {
        // Full speed is too fast
        double speedFactor = 0.6;

        // Don't run collector while hopper sweeper is pushing particles
        if(hopperServoMoving)
        {
            motorCollector.setPower(0);
            return;
        }

        motorCollector.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * speedFactor);
    }

    void controlHopper()
    {
        // When the hopper has 2 particles, the servo doesn't need to move as far
        if(gamepad2.back)
        {
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_FIRST.pos);
            hopperTimer.reset();
            hopperServoMoving = true;
        }
        // When the hopper has 1 particle, the servo needs to move further
        else if(gamepad2.start)
        {
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
            hopperTimer.reset();
            hopperServoMoving = true;
        }
        // Move sweeper back
        else
        {
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
            if(hopperTimer.milliseconds() > 500)
                hopperServoMoving = false;
        }
    }

    void controlCatapult()
    {
        /*
         * The catapult needs to know where the arm is at any point in time. We do this by defining
         * a zero location, which is determined with a touch sensor. Then all movements are based
         * on the encoder that references the zero value. When the zero location has a value of 0,
         * that means that it hasn't yet been set. So we keep checking until the touch sensor stops
         * being depressed, and use that as the zero location.
         */
        if(catapultZero == 0)
        {
            if(gamepad2.left_bumper)
            {
                motorCatapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorCatapult.setPower(1.0);
            }
            if(!catapultButton.isPressed())
            {
                catapultZero = motorCatapult.getCurrentPosition();
                motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorCatapult.setTargetPosition(catapultZero);
            }
            // Don't run any other code until the armed location has been set
            return;
        }

        // Only control the catapult if it's done moving
        if(catapultIsAtTarget())
        {
            // Go to next arming location
            if(gamepad2.left_bumper)
            {
                catapultZero += CATAPULT_TICKS_PER_CYCLE;
                motorCatapult.setTargetPosition(catapultZero);
            }
            // Run motor forward a bit to launch particle
            else if(gamepad2.right_bumper)
            {
                int targetPosition = motorCatapult.getCurrentPosition() + 3000;
                motorCatapult.setTargetPosition(targetPosition);
            }
        }

        // Give manual control to driver if necessary
        if(Math.abs(gamepad2.right_stick_y) > 0.1)
        {
            motorCatapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorCatapult.setPower(-gamepad2.right_stick_y); // Y axis is flipped
        }
        // Return control to motor controller
        else if(motorCatapult.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
        {
            // Keep the motor here
            motorCatapult.setTargetPosition(motorCatapult.getCurrentPosition());
            motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Motor will need power to move to next target when it's requested, but won't move yet
            // because it's already at the target (zero location)
            motorCatapult.setPower(1.0);
        }
    }
}
