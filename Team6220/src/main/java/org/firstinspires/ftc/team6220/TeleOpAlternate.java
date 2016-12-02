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

        double lTime = 0;
        DriverInput driver1 = new DriverInput(gamepad1);
        DriverInput driver2 = new DriverInput(gamepad2);
        while (opModeIsActive())
        {
            double iTime = System.nanoTime()/1000/1000/1000;
            double eTime = iTime-lTime;


            driver1.update(eTime);
            driver2.update(eTime);

            mag = driver1.getLeftStickMagnitude();
            ang = driver1.getLeftStickAngle();
            double rAng = getAngularOrientationWithOffset() * Constants.degToRadConversionFactor;

            newX = Math.cos(ang+rAng)*mag;
            newY = Math.sin(ang+rAng)*mag;
            driveRobotWithJoysticks(newX,    //local x motion power
                                     newY,     //local y motion power
                            gamepad1.right_stick_x/2);    //rotation power


            if (driver2.isButtonPressed("A"))
            {
                CollectorMotor.setPower(1.0);
            }
            else if (driver2.isButtonPressed("B"))
            {
                CollectorMotor.setPower(-1.0);
            }
            else
            {
                CollectorMotor.setPower(0.0);
            }

            driver1.roll();
            driver2.roll();
            lTime = iTime;
            idle();
        }
    }
}
