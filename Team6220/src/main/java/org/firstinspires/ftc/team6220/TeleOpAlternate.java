package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    Competition configuration for driving robot.
    Pilot controls:


    Co-pilot controls:



*/
@TeleOp(name="Alternate (Global/Pulse)", group="6220")
public class TeleOpAlternate extends MasterTeleOp
{
    ElapsedTime timer = new ElapsedTime();
    //temporary global control variables
    double newX = 0.0;
    double newY = 0.0;
    double mag = 0.0;
    double ang = 0.0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            mag = Math.pow(Math.pow(x,2)+Math.pow(y,2),0.5);
            ang = Math.atan2(y, x);
            double rAng = imu.getAngularOrientation().firstAngle/57.3;

            newX = Math.cos(ang+rAng)*mag;
            newY = Math.sin(ang+rAng)*mag;
            driveRobotWithJoysticks(newX,    //local x motion power; reversed
                                     newY,     //local y motion power
                            gamepad1.right_stick_x/2);    //rotation power; reversed


            if (gamepad2.x)
            {
                CollectorMotor.setPower(1.0);
            }
            else if (gamepad2.b)
            {
                CollectorMotor.setPower(-1.0);
            }
            else
            {
                CollectorMotor.setPower(0.0);
            }

            idle();
        }
    }
}
