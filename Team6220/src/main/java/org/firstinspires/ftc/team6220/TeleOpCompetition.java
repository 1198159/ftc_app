package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Competition configuration for driving robot.
    Pilot controls:


    Co-pilot controls:



*/
@TeleOp(name="Competition", group="6220")
public class TeleOpCompetition extends MasterTeleOp
{
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            driveRobotWithJoysticks(gamepad1.right_stick_x,
                                    gamepad1.right_stick_y,
                                    gamepad1.left_stick_x  );

            idle();
        }
    }
}
