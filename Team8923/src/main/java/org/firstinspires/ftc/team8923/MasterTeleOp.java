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
            servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_EXTEND.pos);
        else if(gamepad1.x)
            servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_RETRACT.pos);
        if(gamepad1.dpad_left)
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_LEFT.pos);
        else if(gamepad1.dpad_right)
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_RIGHT.pos);
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
        else if(!liftDeploying)
            motorLift.setPower(0);

        // Code below is for auto lift deployment. It is written as a state machine to allow
        // drivers to continue operating robot

        // Start auto deployment when requested
        if(gamepad1.b)
        {
            liftState = 0;
            liftDeploying = true;
            liftTimer.reset();
        }

        int liftTolerance = 50;

        // Move beacon pusher servo down to ensure it's out of the way
        if(liftState == 0 && liftDeploying)
        {
            liftState++;
            servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_RETRACT.pos);
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_CENTER.pos);
        }
        // Raise the lift to make it deploy
        else if(liftState == 1 && liftTimer.milliseconds() > 500)
        {
            liftState++;
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftZero = motorLift.getCurrentPosition();
            motorLift.setTargetPosition(liftZero + 600);
            motorLift.setPower(1.0);
        }
        // Lower the lift once it's deployed
        else if(liftState == 2 && Math.abs(motorLift.getCurrentPosition() - motorLift.getTargetPosition()) < liftTolerance)
        {
            liftState++;
            motorLift.setTargetPosition(liftZero);
        }
        // Stop moving the lift and return control to driver
        else if(liftState == 3 && Math.abs(motorLift.getCurrentPosition() - motorLift.getTargetPosition()) < liftTolerance)
        {
            liftState++;
            motorLift.setPower(0.0);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftDeploying = false;
        }
    }

    int liftState = 0;
    int liftZero = 0;
    boolean liftDeploying = false;
    ElapsedTime liftTimer = new ElapsedTime();

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
            if(hopperTimer.milliseconds() > 250)
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
                catapultZero = motorCatapult.getCurrentPosition() + 1000;
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
                int targetPosition = catapultZero + 3000;
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
            // If the driver rotates for more than 2 cycles then pressed a button, the motor
            // will run backwards, which is bad. So we unset the zero, which the driver
            // will reset with the button again.
            catapultZero = 0;
        }
    }
}
