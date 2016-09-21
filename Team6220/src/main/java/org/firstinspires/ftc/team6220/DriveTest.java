package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Test op mode that drives the robot with motor powers.
*/
@TeleOp(name="6220 Test Drive", group="6220")
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
