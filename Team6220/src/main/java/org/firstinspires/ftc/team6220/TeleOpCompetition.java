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
        double eTime = timer.time() - lTime;
        lTime = timer.time();
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            driveRobotWithJoysticks(gamepad1.left_stick_x,    //local x motion power
                                    gamepad1.left_stick_y,     //local y motion power
                                    gamepad1.right_stick_x/2, //rotation power; divided by 2 to reduce our robot'shigh rotational velocity
                                    driver1.isButtonPressed("Right Bumper")    );

            //intake balls with collector; drivers must hold buttons to collect
            if (driver2.isButtonPressed("X"))
            {
                collectorMotor.setPower(1.0);
            }
            else if (driver2.isButtonPressed("B"))
            {
                collectorMotor.setPower(-1.0);
            }
            else
            {
                collectorMotor.setPower(0.0);
            }


            if(driver2.isButtonPressed("Right Bumper"))
            {
                launcher.releaseParticle();
            }

            updateCallback(eTime);
            idle();
        }
    }
}
