package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.util.Range;

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


    // todo Test and implement this for competition
    // A driver assist that cuts time in TeleOp.  Raises glyphter while turning 180, then lowers
    // glyphter.
    void autoAlignToScore()
    {
        motorGlyphter.setTargetPosition(Constants.HEIGHT_3);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(0.4);    // todo Adjust

        masterAutonomous.turnTo(getAngularOrientationWithOffset() + 180);

        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(0.4);    // todo Adjust
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

        double power = tFactor * stickCurve.getOuput(driver1.getRightStickMagnitude());
        driveAccelFilter.roll(power);
        double adjustedDrivePower = driveAccelFilter.getFilteredValue();

        double rotationPower = -rFactor * stickCurve.getOuput(gamepad1.left_stick_x);
        turnAccelFilter.roll(rotationPower);
        double adjustedRotationPower = turnAccelFilter.getFilteredValue();
        //----------------------------------------------------------------------------------------


        // Change drive direction based on driver input
        if (driver1.isButtonJustPressed(Button.Y))
            driveDirectionShift = false;
        else if (driver1.isButtonJustPressed(Button.X))
            driveDirectionShift = true;


        // Drive in direction based on whether driveDirectionShift is true
        if (!driveDirectionShift)
            driveMecanum(angle, adjustedDrivePower, adjustedRotationPower);
        else
            driveMecanum(angle + Math.PI / 2, adjustedDrivePower, adjustedRotationPower);
    }
}
