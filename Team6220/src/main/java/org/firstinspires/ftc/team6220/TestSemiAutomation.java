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
            //update robot location using encoders/imu
            drive.robotLocation.rot = imu.getAngularOrientation().firstAngle;
            Transform2D motion = drive.getRobotMotionFromEncoders();
            double iTime = System.nanoTime()/1000/1000/1000;
            double eTime = iTime-lTime;
            drive.robotLocation.x += eTime*( motion.x*Math.cos(drive.robotLocation.rot/57.3) - motion.y*Math.sin(drive.robotLocation.rot/57.3) );
            drive.robotLocation.y += eTime*( motion.x*Math.sin(drive.robotLocation.rot/57.3) + motion.y*Math.cos(drive.robotLocation.rot/57.3) );

            if(gamepad1.right_bumper)
            {
                //autonomously move to 0,0,0
                drive.navigateTo(new Transform2D(0,0,0));
            }
            driveRobotWithJoysticks(-gamepad1.right_stick_x,    //local x motion power; reversed
                                    gamepad1.right_stick_y,     //local y motion power
                                    -gamepad1.left_stick_x);    //rotation power; reversed

            lTime = iTime;
            idle();
        }
    }
}