package org.firstinspires.ftc.team6220_2018;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input.
*/
abstract public class MasterTeleOp extends MasterOpMode
{
    boolean slowMode = false;
    // Tells us whether the robot's direction of movement is shifted or not
    boolean driveDirectionShift = false;

    // Factor that adjusts magnitudes of vertical and horizontal movement.
    double tFactor = Constants.T_FACTOR;
    // Factor that adjusts magnitude of rotational movement.
    double rFactor = Constants.R_FACTOR;

    // For using turning and other autonomous functionalities in TeleOp.
    MasterAutonomous masterAutonomous;


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
        if (driver1.isButtonJustPressed(Button.Y))
            driveDirectionShift = false;
        else if (driver1.isButtonJustPressed(Button.X))
            driveDirectionShift = true;


        if (driver1.getRightTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
            driveHanger(driver1.getRightTriggerValue());
        else if (driver1.getLeftTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
            driveHanger(-driver1.getLeftTriggerValue());


        // Drive in direction based on whether driveDirectionShift is true
        if (!driveDirectionShift)
            driveMecanum(angle, drivePower, rotationPower);
        else
            driveMecanum(angle + 180, drivePower, rotationPower);
    }
}
