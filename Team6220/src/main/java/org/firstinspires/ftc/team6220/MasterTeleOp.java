package org.firstinspires.ftc.team6220;

/**
 * Created by Colew on 9/18/2016.
 */
abstract public class MasterTeleOp extends MasterOpMode
{
    //adjusted power control for drivers
    public double powerRightStickX = 0.5 * Math.pow(gamepad1.right_stick_x , 2) + 0.5 * gamepad1.right_stick_x;
    public double powerRightStickY = 0.5 * Math.pow(gamepad1.right_stick_y , 2) + 0.5 * gamepad1.right_stick_y;
    public double powerLeftStickX = 0.5 * Math.pow(gamepad1.left_stick_x , 2) + 0.5 * gamepad1.left_stick_x;
}
