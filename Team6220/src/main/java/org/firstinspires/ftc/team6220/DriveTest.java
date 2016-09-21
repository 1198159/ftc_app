package org.firstinspires.ftc.team6220;

/*
    Test op mode that drives the robot with motor powers.
*/

public class DriveTest extends MasterOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        while (opModeIsActive())
        {
            drive.moveRobot(gamepad1.right_stick_x,
                            gamepad1.right_stick_y,
                            gamepad1. left_stick_x );
        }
    }
}
