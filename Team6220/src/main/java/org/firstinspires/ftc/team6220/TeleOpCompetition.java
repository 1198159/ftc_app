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

    //CodeReview: Define an enum for reading/writing the elements of your lastBtn array instead of using magic numbers in your code.
    //temporary tap trigger variable
    //not currently in use
    //                                   a      b      x      y
    boolean lastBtn[] = new boolean[]{false, false, false, false};

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            driveRobotWithJoysticks(gamepad1.left_stick_x,    //local x motion power
                                    gamepad1.left_stick_y,     //local y motion power
                                    gamepad1.right_stick_x/2);    //rotation power; divided by 2 to reduce our robot's
                                                                   //high rotational velocity

            //intake balls with collector; drivers must hold buttons to collect
            if (gamepad2.x)
            {
                collectorMotor.setPower(1.0);
            }
            else if (gamepad2.b)
            {
                collectorMotor.setPower(-1.0);
            }
            else
            {
                collectorMotor.setPower(0.0);
            }

            //deploys and retracts our servo-powered gate that stops balls before moving into the launcher
            if (gamepad2.right_bumper)
            {
                collectorGateServoToggler.toggle();
            }

            idle();
        }
    }
}
