package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes for the CapBot
 */
public class MasterTeleOpCapBot extends Master
{
    // TODO: Test this and fix if needed
    public void mecanumDrive()
    {
        // These make the code below easier to read
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        // Find out how much the joystick has moved from the center
        double joystickDisplacement = calculateDistance(x, y);

        // Used for turning
        double turnPower = gamepad1.right_stick_x;

        // Set power for motors. Ratios are correct, but needs scaling (see below)
        double powerFL = y - x + turnPower;
        double powerFR = y + x - turnPower;
        double powerBL = y + x + turnPower;
        double powerBR = y - x - turnPower;

        // When going forward, the motor powers are at about 71% when the joystick is all the way
        // at the edge of its range, so this compensates for not being at max power when requested.
        // Also scales motor powers if one goes outside of it's range
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Scaler equation above doesn't account for times when partial power is requested. This
        // compensates for that, but only when the joystick is not in the center (don't divide by
        // 0), and within a 1 unit radius of the center. Otherwise it can set the motor powers
        // higher than 1
        if(joystickDisplacement != 0 && joystickDisplacement < 1)
            scalar /= joystickDisplacement;

        // Don't divide by 0
        if(scalar != 0)
        {
            powerFL /= scalar;
            powerFR /= scalar;
            powerBL /= scalar;
            powerBR /= scalar;
        }

        // Set motor powers
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }
}
