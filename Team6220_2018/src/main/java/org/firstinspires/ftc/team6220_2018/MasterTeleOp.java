package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input.
*/
abstract public class MasterTeleOp extends MasterOpMode
{
    // For using turning and other autonomous functionalities in TeleOp.
    MasterAutonomous masterAutonomous;
    boolean slowMode = false;
    // Allows us to switch front of robot.
    boolean driveReversed = true;
    // Allows us to switch front of robot.
    boolean hangServoDeployed = false;
    // Determines whether arm is in RUN_TO_POSITION or RUN_USING_ENCODER.
    //boolean armRunModePosition = true;
    boolean armRunModeUsingEncoder = false;

    boolean armSlowMode = false;

    // Factor that adjusts magnitudes of vertical and horizontal movement.
    double tFactor = Constants.T_FACTOR;
    // Factor that adjusts magnitude of rotational movement.
    double rFactor = Constants.R_FACTOR;

    double armFactorIn = Constants.MOTOR_COLLECTOR_IN;
    double armFactorOut = Constants.MOTOR_COLLECTOR_OUT;

    // Takes driver 1 input to run hanger system.
    void driveHanger()
    {
        if (driver1.getRightTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
            motorHanger.setPower(stickCurve.getOuput(-driver1.getRightTriggerValue()));
        else if (driver1.getLeftTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
            motorHanger.setPower(stickCurve.getOuput(driver1.getLeftTriggerValue()));
        else
            motorHanger.setPower(0.0);

        // Toggle hanger servo
        if (driver1.isButtonJustPressed(Button.B) && !hangServoDeployed)
        {
            servoHanger.setPosition(Constants.SERVO_HANG_DEPLOYED);
            hangServoDeployed = true;
        }
        else if (driver1.isButtonJustPressed(Button.B) && hangServoDeployed)
        {
            servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
            hangServoDeployed = false;
        }

        //telemetry.addData("Trigger val Right: ", driver1.getRightTriggerValue());
        //telemetry.addData("Trigger val Left: ", driver1.getLeftTriggerValue());
        telemetry.addData("Hanger Enc: ", motorHanger.getCurrentPosition());
    }

    // Uses driver 2 input to drive arm and collector motors.
    void driveCollectorMechanism()
    {
        if (driver2.isButtonJustPressed(Button.RIGHT_BUMPER) && !armSlowMode)
        {
            armFactorIn = Constants.MOTOR_COLLECTOR_SLOW_IN;
            armFactorOut = Constants.MOTOR_COLLECTOR_SLOW_OUT;
            armSlowMode = true;
        }
        else if (driver2.isButtonJustPressed(Button.RIGHT_BUMPER) && armSlowMode)
        {
            armFactorIn = Constants.MOTOR_COLLECTOR_IN;
            armFactorOut = Constants.MOTOR_COLLECTOR_OUT;
            armSlowMode = false;
        }

        // Collect and eject minerals.  Buttons have to be held to power collector.
        if (driver2.isButtonPressed(Button.DPAD_DOWN))
        {
            motorCollector.setPower(armFactorIn);
        }
        else if (driver2.isButtonPressed(Button.DPAD_UP))
        {
            motorCollector.setPower(armFactorOut);
        }
        else
        {
            motorCollector.setPower(0);
        }


        if (Math.abs(driver2.getRightStickY()) >= Constants.MINIMUM_JOYSTICK_POWER)
        {
            armRunModeUsingEncoder = true;
            motorArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (motorArmLeft.getCurrentPosition() <= Constants.ARM_SWITCH__HEIGHT)
            {
                driveArm(-0.3 * stickCurve.getOuput(driver2.getRightStickY()));
            }
            else
            {
                driveArm(-0.1 * stickCurve.getOuput(driver2.getRightStickY()));
            }
            //motorArmLeft.setPower(-Constants.MAX_ARM_POWER * stickCurve.getOuput(driver2.getRightStickY()));
            //motorArmRight.setPower(-Constants.MAX_ARM_POWER * stickCurve.getOuput(driver2.getRightStickY()));
        }
        else if (Math.abs(driver2.getRightStickY()) < Constants.MINIMUM_JOYSTICK_POWER && armRunModeUsingEncoder)
        {
            armRunModeUsingEncoder = false;
            driveArm(0);
            //motorArmLeft.setPower(0);
            motorArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //motorArmRight.setPower(0);
            motorArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        /*// Toggle arm run mode boolean using start button.
        if(driver2.isButtonJustPressed(Button.RIGHT_STICK_PRESS) && armRunModePosition)
        {
            motorArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armRunModePosition = false;
        }
        else if(driver2.isButtonJustPressed(Button.RIGHT_STICK_PRESS) && !armRunModePosition)
        {
            motorArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRunModePosition = true;
        }

        // Run arm in position or power mode, depending on the boolean value.
        if(armRunModePosition)
        {*/
        else if (Math.abs(driver2.getRightStickY()) < Constants.MINIMUM_JOYSTICK_POWER && !armRunModeUsingEncoder)
        {
            /*if (driver2.isButtonPressed(Button.X))
            {
                motorArmLeft.setTargetPosition(Constants.ARM_GROUND);
                //motorArmLeft.setPower(0.3);
                motorArmRight.setTargetPosition(-Constants.ARM_GROUND);
                //motorArmRight.setPower(0.3);
                driveArm(Constants.MAX_ARM_POWER);
            }*/
            /*if (driver2.isButtonPressed(Button.Y))
            {
                motorArmLeft.setTargetPosition(Constants.ARM_TOP);
                //motorArmLeft.setPower(Constants.MAX_ARM_POWER);
                motorArmRight.setTargetPosition(-Constants.ARM_TOP);
                //motorArmRight.setPower(Constants.MAX_ARM_POWER);
                driveArm(Constants.MAX_ARM_POWER);

            }*/
            /*else if (driver2.isButtonPressed(Button.B))
            {
                motorArmLeft.setTargetPosition(Constants.ARM_START);
                //motorArmLeft.setPower(0.2);
                motorArmRight.setTargetPosition(-Constants.ARM_START);
                //motorArmRight.setPower(0.2);
                driveArm(Constants.MAX_ARM_POWER);
            }*/
        }
        /*else if(!armRunModePosition)
        {
            motorArmLeft.setPower(-Constants.MAX_ARM_POWER * stickCurve.getOuput(driver2.getRightStickY()));
        }*/
        /*// Run arm in position or power mode, depending on the boolean value.
        if (driver2.isButtonPressed(Button.X))
        {
            motorArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmLeft.setTargetPosition(Constants.ARM_GROUND);
            motorArmLeft.setPower(0.3);
        }
        else if (driver2.isButtonPressed(Button.Y))
        {
            motorArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmLeft.setTargetPosition(Constants.ARM_TOP);
            motorArmLeft.setPower(Constants.MAX_ARM_POWER);
        }
        else if (driver2.isButtonPressed(Button.B))
        {
            motorArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmLeft.setTargetPosition(Constants.ARM_START);
            motorArmLeft.setPower(0.2);
        }
        else if(stickCurve.getOuput(driver2.getRightStickY()) > Constants.MINIMUM_JOYSTICK_POWER)
        {
            motorArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmLeft.setPower(-Constants.MAX_ARM_POWER * stickCurve.getOuput(driver2.getRightStickY()));
        }*/


        telemetry.addData("Arm Position Left: ", motorArmLeft.getCurrentPosition());
        telemetry.addData("Arm Position Right: ", motorArmRight.getCurrentPosition());
        telemetry.addData("Arm Run Mode Using Encoder: ", armRunModeUsingEncoder);
        telemetry.addData("Arm Slow Mode: ", armSlowMode);
        telemetry.update();
    }

    // Takes driver 1 stick input and uses it to give power and direction inputs to the drive
    void driveMecanumWithJoysticks()
    {
        // Note: factors are different for translation and rotation
        // Slow mode functionality.  1st driver presses right bumper to toggle slow mode
        if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER) && !slowMode)
        {
            tFactor = Constants.SLOW_MODE_T_FACTOR;
            rFactor = Constants.SLOW_MODE_R_FACTOR;
            slowMode = true;
        }
        else if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER) && slowMode)
        {
            tFactor = Constants.T_FACTOR;
            rFactor = Constants.R_FACTOR;
            slowMode = false;
        }


        // Stick inputs must be changed from x and y to angle, drive power, and rotation power---
        double angle = Math.toDegrees(driver1.getRightStickAngle());

        double drivePower = tFactor * stickCurve.getOuput(driver1.getRightStickMagnitude());

        double rotationPower = -rFactor * stickCurve.getOuput(gamepad1.left_stick_x);
        //----------------------------------------------------------------------------------------


        // Change drive direction based on driver input
        if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && !driveReversed)
            driveReversed = true;
        else if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && driveReversed)
            driveReversed = false;

        // Drive in direction based on whether driveDirectionShift is true
        if (!driveReversed)
            driveMecanum(angle, drivePower, rotationPower);
        else
            driveMecanum(angle + 180, drivePower, rotationPower);
    }
}
