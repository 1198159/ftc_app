package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    Competition configuration for driving robot.
    Pilot controls:
     - Right Bumper: Slow mode

    Co-pilot controls:
     - X: Collect
     - B: Reverse Collect
     - Right Bumper: Load particle


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
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();


            driveRobotWithJoysticks(gamepad1.left_stick_x,    //local x motion power
                                    gamepad1.left_stick_y,     //local y motion power
                                    gamepad1.right_stick_x/2, //rotation power; divided by 2 to reduce our robot'shigh rotational velocity
                                    driver1.isButtonPressed(Button.RIGHT_BUMPER)    );

            //intake balls with collector; drivers must hold buttons to collect
            if (driver2.isButtonPressed(Button.X))
            {
                collectorMotor.setPower(1.0);
            }
            else if (driver2.isButtonPressed(Button.B))
            {
                collectorMotor.setPower(-1.0);
            }
            else
            {
                collectorMotor.setPower(0.0);
            }


            if(driver2.isButtonPressed(Button.RIGHT_BUMPER))
            {
                launcher.loadParticle();
            }

            telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
