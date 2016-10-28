package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    Competition configuration for driving robot.
    Pilot controls:


    Co-pilot controls:



*/
@TeleOp(name="TeleOpCompetition", group="6220")
public class TeleOpCompetition extends MasterTeleOp
{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        timer.reset();

        while (opModeIsActive())
        {
            driveRobotWithJoysticks(-gamepad1.right_stick_x,    //local x motion power; reversed
                                    gamepad1.right_stick_y,     //local y motion power
                                    -gamepad1.left_stick_x);    //rotation power; reversed
            updateLocation();

            telemetry.addData("Time: ", timer.milliseconds());
            timer.reset();
            telemetry.update();
            idle();
        }
    }
}
