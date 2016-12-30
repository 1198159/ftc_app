package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Test configuration for driving robot with both joysticks and automation
    Pilot controls:
     - RB: Activate return to start position.

    Co-pilot controls:



*/
@TeleOp(name="Test Semi-Automation", group="6220")
public class TestSemiAutomation extends MasterTeleOp
{
    double lTime = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            //update robot location using encoders and imu
            drive.robotLocation.rot = getAngularOrientationWithOffset();
            Transform2D motion = drive.getRobotMotionFromEncoders(eTime);
            drive.robotLocation.x += eTime*( motion.x * Math.cos(drive.robotLocation.rot * Constants.DEGREE_TO_RADIAN) - motion.y * Math.sin(drive.robotLocation.rot * Constants.DEGREE_TO_RADIAN) );
            drive.robotLocation.y += eTime*( motion.x * Math.sin(drive.robotLocation.rot * Constants.DEGREE_TO_RADIAN) + motion.y * Math.cos(drive.robotLocation.rot * Constants.DEGREE_TO_RADIAN) );

            if(gamepad1.right_bumper)
            {
                //autonomously move to 0,0,0
                drive.NavigateTo(new Transform2D(0,0,0));
            }
            driveRobotWithJoysticks(gamepad1.left_stick_x,    //local x motion power
                                    gamepad1.left_stick_y,     //local y motion power
                                    gamepad1.right_stick_x/2,
                                    false);    //rotation power

            telemetry.addData("X: ", drive.robotLocation.x);
            telemetry.addData("Y: ", drive.robotLocation.y);
            idle();
        }
    }
}