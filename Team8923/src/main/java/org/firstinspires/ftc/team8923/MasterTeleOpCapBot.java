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
        // Find angle of left joystick. Used to make the robot drive in that direction. Needs to
        // have an offset angle added because of how mecanum wheels work. Ask Dryw for more info
        double joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);

        // Find out how much the joystick has moved from the center
        double joystickDisplacement = calculateDistance(gamepad1.left_stick_x, gamepad1.left_stick_y);

        // Used for turning
        double turningPower = gamepad1.right_stick_x;

        // Set power for motors. Ratios are correct, but needs scaling (see below)
        double powerFL = Math.cos(joystickAngle) + turningPower;
        double powerFR = Math.sin(joystickAngle) - turningPower;
        double powerBL = Math.sin(joystickAngle) + turningPower;
        double powerBR = Math.cos(joystickAngle) - turningPower;

        // In case the joystick is centered, don't let the trig functions don't do anything
        if(joystickDisplacement == 0)
        {
            powerFL = turningPower;
            powerFR = -turningPower;
            powerBL = turningPower;
            powerBR = -turningPower;
        }

        // When going forward, the motor powers are at about 71% when the joystick is all the way
        // at the edge of its range, so this compensates for not being at max power when requested.
        // Also scales motor powers if one goes outside of it's range
        double scaler = Math.max(powerFL, Math.max(powerFR, Math.max(powerBL, powerBR)));

        // Scaler equation above doesn't account for times when partial power is requested. This
        // compensates for that, but only when the joystick is not in the center (don't divide by
        // 0), and within a 1 unit radius of the center. Otherwise it can set the motor powers
        // higher than 1
        if(joystickDisplacement != 0 && joystickDisplacement < 1)
            scaler /= joystickDisplacement;

        // Don't divide by 0
        if(scaler != 0)
        {
            powerFL /= scaler;
            powerFR /= scaler;
            powerBL /= scaler;
            powerBR /= scaler;
        }

        // Set motor powers
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }
}
