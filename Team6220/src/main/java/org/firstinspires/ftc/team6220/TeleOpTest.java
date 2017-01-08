package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
    Competition configuration for driving robot.
    Pilot controls:
     - Right Bumper: Slow mode

    Co-pilot controls:
     - X: Collect
     - B: Reverse Collect
     - Right Bumper: Load particle
*/
@TeleOp(name="Test TeleOp", group="6220")
public class TeleOpTest extends MasterTeleOp
{
    boolean launcherManualControl = false;
    double targetOrientation = 0.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();



            //intake balls with collector; drivers must hold buttons to collect
            if (driver2.isButtonPressed(Button.X))
            {
                collectorMotor.setPower(1.0);
                //we dont want the 2nd stage servo to reverse and pull out an already troughed particle
                gateServo.setPosition(0.5);
            }
            else if (driver2.isButtonPressed(Button.B))
            {
                collectorMotor.setPower(-1.0);
                gateServo.setPosition(0.0);
            }
            else
            {
                collectorMotor.setPower(0.0);
                gateServo.setPosition(0.5);
            }

            //pulls back launcher
            if (driver2.isButtonPressed(Button.DPAD_DOWN))
            {
                //launcher.pullback();
            }

            //puts a particle into the launcher
            if(driver2.isButtonPressed(Button.RIGHT_BUMPER))
            {
                launcher.loadParticle();
            }

            //shoots a particle
            if (driver2.isButtonPressed(Button.DPAD_UP))
            {
                launcher.launchParticle();
            }

            if (driver2.isButtonPressed(Button.Y))
            {
                //launcher.trimForward();
            }


            if (driver2.isButtonPressed(Button.A))
            {
                //launcher.trimBackward();
            }

            if(launcherManualControl)
            {
                if(driver2.isButtonPressed(Button.DPAD_LEFT))
                {
                    launcher.pullBackMotor.setPower(-1.0);
                }
                else if (driver2.isButtonPressed(Button.DPAD_RIGHT))
                {
                    launcher.pullBackMotor.setPower(1.0);
                }
                else
                {
                    launcher.pullBackMotor.setPower(0.0);
                }
            }
            if(driver2.isButtonJustPressed(Button.DPAD_RIGHT) || driver2.isButtonJustPressed(Button.DPAD_LEFT))
            {
                launcher.pullBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launcherManualControl = true;
            }
            else if (driver2.isButtonJustReleased(Button.DPAD_RIGHT) || driver2.isButtonJustReleased(Button.DPAD_LEFT))
            {
                launcherManualControl = false;
                launcher.pullBackMotor.setPower(0.0);
                launcher.pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            double xRate = gamepad1.left_stick_x;
            double yRate = gamepad1.left_stick_y;
            if(driver1.isButtonPressed(Button.LEFT_BUMPER))
            {
                if(driver1.isButtonPressed(Button.DPAD_LEFT)){xRate = -1.0;}
                if(driver1.isButtonPressed(Button.DPAD_RIGHT)){xRate = 1.0;}
                if(driver1.isButtonPressed(Button.DPAD_UP)){yRate = -1.0;}
                if(driver1.isButtonPressed(Button.DPAD_DOWN)){yRate = 1.0;}
            }

            /*
            driveRobotWithJoysticks(xRate,    //local x motion power
                    yRate,     //local y motion power
                    gamepad1.right_stick_x/2, //rotation power; divided by 2 to reduce our robot'shigh rotational velocity
                    driver1.isButtonPressed(Button.RIGHT_BUMPER));  //slow mode functionality on right bumper
*/

            telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
