package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Competition configuration for driving robot.
    Pilot controls:


    Co-pilot controls:



*/
@TeleOp(name="TeleOpCompetition", group="6220")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            driveRobotWithJoysticks(-gamepad1.right_stick_x,    //local x motion power; reversed
                                    gamepad1.right_stick_y,     //local y motion power
                                    -gamepad1.left_stick_x);    //rotation power; reversed

            idle();
        }
    }
}
