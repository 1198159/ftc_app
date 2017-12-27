package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.util.Range;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input
*/
abstract public class MasterTeleOp extends MasterOpMode
{
    boolean slowMode = false;

    // Factor that adjusts magnitudes of vertical and horizontal movement
    double tFactor = Constants.T_FACTOR;
    // Factor that adjusts magnitude of rotational movement
    double rFactor = Constants.R_FACTOR;

    // Takes driver 1 stick input and uses it to give power and direction inputs to the drive
    void driveMecanumWithJoysticks()
    {
        // Allows the 2nd driver to raise the vertical jewel servo if it fails during the match
        if(driver2.isButtonPressed(Button.LEFT_BUMPER))
        {
            verticalJewelServoToggler.retract();
        }



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

        double power = driver1.getRightStickMagnitude();

        // Ensure stick magnitude is within the range {0,1}, since magnitudes are not uniform for
        // all stick positions (they have been observed to be as large as 1.1)
        if (power > 1.0)
            power = 1.0;

        // Stick inputs must be changed from x and y to angle and drive power
        double angle = Math.toDegrees(driver1.getRightStickAngle());
        double adjustedDrivePower = tFactor * stickCurve.getOuput(power);
        double rotationPower = rFactor * stickCurve.getOuput(gamepad1.left_stick_x);

        driveMecanum(angle, adjustedDrivePower, rotationPower);
    }
}
