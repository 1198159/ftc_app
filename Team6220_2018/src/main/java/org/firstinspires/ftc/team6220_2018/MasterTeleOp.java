package org.firstinspires.ftc.team6220_2018;

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

    // Factor that adjusts magnitudes of vertical and horizontal movement.
    double tFactor = Constants.T_FACTOR;
    // Factor that adjusts magnitude of rotational movement.
    double rFactor = Constants.R_FACTOR;

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
        // Collect and eject minerals.  Buttons have to be held to power collector.
        if (driver2.isButtonPressed(Button.DPAD_DOWN))
            motorCollector.setPower(0.9);
        else if (driver2.isButtonPressed(Button.DPAD_UP))
            motorCollector.setPower(-0.6);
        else
            motorCollector.setPower(0);

        // Operate arm.
        motorArm.setPower(-0.2 * stickCurve.getOuput(driver2.getRightStickY()));
        //telemetry.addData("Arm Position: ", motorArm.getCurrentPosition());
        //telemetry.update();
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
