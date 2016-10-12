package org.firstinspires.ftc.team6220;

/**
 * Created by Colew on 9/18/2016.
 */
abstract public class MasterTeleOp extends MasterOpMode
{
    void driveRobotWithJoysticks()
    {
        double RightStickX = -1.0 * (0.5 * Math.pow(gamepad1.right_stick_x , 3) + 0.5 * gamepad1.right_stick_x);
        double RightStickY = -1.0 * (0.5 * Math.pow(gamepad1.right_stick_y , 3) + 0.5 * gamepad1.right_stick_y);
        double LeftStickX = -1.0 * (0.5 * Math.pow(gamepad1.left_stick_x , 3) + 0.5 * gamepad1.left_stick_x);

        drive.moveRobot(RightStickX, RightStickY, LeftStickX);
    }
}
