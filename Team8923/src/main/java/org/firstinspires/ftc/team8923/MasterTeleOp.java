package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes
 */
abstract class MasterTeleOp extends Master
{
    // Variables used for hopper control
    private ElapsedTime hopperTimer = new ElapsedTime();
    private boolean hopperServoMoving = false;

    // Variables used for semi-auto lift deployment
    private int liftState = 0;
    private int liftZero = 0;
    private boolean liftDeploying = false;
    private boolean liftDeployed = false;
    private boolean liftRecovering = false;
    private boolean liftRecovered = false;
    private ElapsedTime liftTimer = new ElapsedTime();

    // Variables used for semi-auto catapult shooting
    private int catapultState = 0;
    private int particlesToShoot = 0;
    private boolean guideButtonLast = false;
    private boolean catapultShooting = false;
    ElapsedTime catapultTimer = new ElapsedTime();

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
        // Don't give control to the beacon pusher after lift has been deployed
        if(liftDeployed)
            return;

        if(gamepad1.a)
            servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_EXTEND.pos);
        else if(gamepad1.x)
        {
            servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_RETRACT.pos);
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_CENTER.pos);
        }
        if(gamepad1.dpad_left)
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_LEFT.pos);
        else if(gamepad1.dpad_right)
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_RIGHT.pos);
    }

    // Contains controls for the lift and it's automatic routines, and cap ball holder arm
    void runLift()
    {
        // Don't allow driver to move the lift once the arms have been recovered
        if(!liftRecovered && !liftRecovering)
        {
            // Move cap ball holder arm to hold the cap ball
            if(gamepad2.x)
                servoCapBallHolder.setPosition(ServoPositions.CAP_BALL_HOLD.pos);
            // Move cap ball holder arm to release the cap ball. This is on the dpad to ensure it's
            // out of the way when the lift is raised
            else if(gamepad2.dpad_up)
                servoCapBallHolder.setPosition(ServoPositions.CAP_BALL_RELEASE.pos);
        }

        // Give lift control to driver only after lift deployment
        if(liftDeployed && !liftRecovering)
        {
            // Run lift up
            if(gamepad2.dpad_up)
                motorLift.setPower(1.0);
                // Run lift down
            else if(gamepad2.dpad_down)
                motorLift.setPower(-1.0);
                // If no button is pressed, stop!
            else
                motorLift.setPower(0);
        }

        // Code below is for auto lift deployment and arm recovery. They are written as state
        // machine loops to allow drivers to continue operating robot

        // Start auto deployment when requested
        if(gamepad1.b)
        {
            liftState = 0;
            liftDeploying = true;
            liftTimer.reset();
            telemetry.log().add("Starting Lift Deployment");

            // Move beacon pusher servos to ensure they're out of the way
            servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_RETRACT.pos);
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_CENTER.pos);
            servoLiftHolder.setPosition(ServoPositions.LIFT_RELEASE.pos);
            telemetry.log().add("Moving Beacon Pusher");
        }
        // Raise the lift to make it deploy
        else if(liftDeploying && liftState == 0 && liftTimer.milliseconds() > 500)
        {
            liftState++;
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftZero = motorLift.getCurrentPosition();
            motorLift.setTargetPosition(liftZero + 600);
            motorLift.setPower(1.0);
            telemetry.log().add("Raising Lift");
        }
        // Lower the lift once it's deployed
        else if(liftDeploying && liftState == 1 && motorIsAtTarget(motorLift))
        {
            liftState++;
            motorLift.setTargetPosition(liftZero);
            telemetry.log().add("Lowering Lift");
        }
        // Stop moving the lift and return control to driver
        else if(liftDeploying && liftState == 2 && motorIsAtTarget(motorLift))
        {
            liftState++;
            motorLift.setPower(0.0);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_RIGHT.pos);
            liftDeploying = false;
            liftDeployed = true;
            telemetry.log().add("Lift Done Deploying");
        }

        // Start auto arm recovery when requested only after lift has been deployed
        if(gamepad2.y && liftDeployed)
        {
            liftState = 0;
            liftRecovering = true;
            liftTimer.reset();
            telemetry.log().add("Starting Lift Recovery");

            if(!liftRecovered)
            {
                // Raise the lift to make it deploy
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setTargetPosition(liftZero + 1550);
                motorLift.setPower(1.0);
                telemetry.log().add("Positioning Lift");
            }
        }
        // Lower the lift once it's deployed
        else if(liftRecovering && liftState == 0 && motorIsAtTarget(motorLift) || (liftRecovered && liftState == 0))
        {
            liftState++;
            servoCapBallHolder.setPosition(ServoPositions.CAP_BALL_HOLD.pos);
            liftTimer.reset();
            telemetry.log().add("Moving Arm Servo");
        }
        // Stop moving the lift and return control to driver
        else if(liftRecovering && liftState == 1 && liftTimer.milliseconds() > 1000)
        {
            liftState++;
            servoCapBallHolder.setPosition(ServoPositions.CAP_BALL_UP.pos);
            motorLift.setPower(0.0);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRecovering = false;
            liftRecovered = true;
            telemetry.log().add("Recovering Lift Arms");
        }
    }

    // Runs collector. Can be run forwards and backwards
    void runCollector()
    {
        // Full speed is too fast
        double speedFactor = 0.6;

        // Don't control the collector when the catapult is firing
        if(catapultShooting)
            return;

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
        // When the hopper has 1 particle, the servo needs to move further
        if(gamepad2.start)
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
        if(catapultShooting)
        {
            if(particlesToShoot == 4)
            {
                // fire - wait
                // hopper push first
                // arm - wait
                // hopper back
                // start collecting
                // particles--

                if(catapultState == 0 && catapultButton.isPressed())
                    armCatapult();
                if(catapultState == 0 && motorIsAtTarget(motorCatapult))
                {
                    // Shoot particle currently in cup
                    fireCatapult();
                    catapultState++;
                }
                // Wait until catapult finishes firing
                else if(catapultState == 1 && motorIsAtTarget(motorCatapult))
                {
                    // Start pushing the first particle into the cup and arm the catapult
                    servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_FIRST.pos);
                    armCatapult();
                    catapultState++;
                }
                // Wait until catapult finishes arming
                else if(catapultState == 2 && motorIsAtTarget(motorCatapult))
                {
                    // Move the sweeper servo back and collect the next ball into the hopper
                    servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
                    motorCollector.setPower(1.0);
                    catapultState = 0;
                    particlesToShoot--;
                }
            }
            else if(particlesToShoot == 3)
            {
                // fire - wait
                // stop collecting
                // hopper push first
                // arm - wait
                // hopper back
                // particles--

                if(catapultState == 0 && catapultButton.isPressed())
                    armCatapult();
                if(catapultState == 0 && motorIsAtTarget(motorCatapult))
                {
                    // Shoot particle currently in cup
                    fireCatapult();
                    catapultState++;
                }
                // Wait until catapult finishes firing
                else if(catapultState == 1 && motorIsAtTarget(motorCatapult))
                {
                    // Start pushing first particle into cup, stop collector, and arm catapult
                    servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_FIRST.pos);
                    motorCollector.setPower(0.0);
                    armCatapult();
                    catapultState++;
                }
                // Wait until catapult finishes arming
                else if(catapultState == 2 && motorIsAtTarget(motorCatapult))
                {
                    // Move the sweeper servo back
                    servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
                    catapultState = 0;
                    particlesToShoot--;
                }
            }
            else if(particlesToShoot == 2)
            {
                // fire - wait
                // hopper push second
                // arm - wait
                // hopper back
                // particles--

                if(catapultState == 0 && catapultButton.isPressed())
                    armCatapult();
                if(catapultState == 0 && motorIsAtTarget(motorCatapult))
                {
                    // Shoot particle currently in cup
                    fireCatapult();
                    catapultState++;
                }
                // Wait until catapult finishes firing
                else if(catapultState == 1 && motorIsAtTarget(motorCatapult))
                {
                    // Start pushing the first particle into the cup and arm the catapult
                    servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
                    armCatapult();
                    catapultState++;
                }
                // Wait until catapult finishes arming
                else if(catapultState == 2 && motorIsAtTarget(motorCatapult))
                {
                    // Move the sweeper servo back
                    servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
                    catapultState = 0;
                    particlesToShoot--;
                }
            }
            else if(particlesToShoot == 1)
            {
                // fire - wait
                // arm
                // particles--

                if(catapultState == 0 && catapultButton.isPressed())
                    armCatapult();
                if(catapultState == 0 && motorIsAtTarget(motorCatapult))
                {
                    // Shoot particle currently in cup
                    fireCatapult();
                    catapultState++;
                }
                // Wait until catapult finishes firing
                else if(catapultState == 1 && motorIsAtTarget(motorCatapult))
                {
                    // Arm catapult for next time
                    armCatapult();
                    catapultState = 0;
                    particlesToShoot--;
                }
            }
            else if(particlesToShoot == 0)
                catapultShooting = false;
        }
        else
        {
            // Que up particles to shoot
            if(gamepad2.guide && !guideButtonLast && particlesToShoot < 4)
                particlesToShoot++;
            else if(gamepad2.back)
                particlesToShoot = 0;

            guideButtonLast = gamepad2.guide;

            // Activate semi-auto shooting
            if(gamepad2.left_bumper)
                catapultShooting = true;
        }

        telemetry.addData("Particles to shoot", particlesToShoot);

        // Only control the catapult if it's not shooting
        if(!catapultShooting)
        {
            if(gamepad2.right_bumper && motorIsAtTarget(motorCatapult))
            {
                motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorCatapult.setTargetPosition(motorCatapult.getCurrentPosition() + CATAPULT_TICKS_PER_CYCLE / 2);
                motorCatapult.setPower(1.0);
            }

            if(catapultArming && !catapultButton.isPressed() && catapultButtonLast)
            {
                catapultArming = false;
                motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorCatapult.setTargetPosition(motorCatapult.getCurrentPosition());
                motorCatapult.setPower(1.0);
            }

            catapultButtonLast = catapultButton.isPressed();

            // Give manual control to driver if necessary
            if(Math.abs(gamepad2.right_stick_y) > 0.1)
            {
                motorCatapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorCatapult.setPower(-gamepad2.right_stick_y); // Y axis is flipped
            }
            // Return control to motor controller
            else if(motorCatapult.getMode() == DcMotor.RunMode.RUN_USING_ENCODER && !catapultArming)
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
}
