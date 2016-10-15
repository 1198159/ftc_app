package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Test op mode that drives the robot locally.
*/
@TeleOp(name="Test Drive, Local", group="Tests")
@Disabled
public class DriveTest extends MasterTeleOp
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {

            driveRobotWithJoysticks(gamepad1.right_stick_x,   //local x motion power
                                    gamepad1.right_stick_y,   //local y motion power
                                    gamepad1.left_stick_x  ); //rotation power; Rotation is reversed

            idle();
        }
    }
}
