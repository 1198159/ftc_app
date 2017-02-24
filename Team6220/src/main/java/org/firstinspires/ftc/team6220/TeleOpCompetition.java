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
@TeleOp(name="TeleOpCompetition", group="6220")
public class TeleOpCompetition extends MasterTeleOp
{
    boolean launcherManualControl = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        targetHeading = getAngularOrientationWithOffset();

        waitForStart();

        lTime = timer.seconds();

        while (opModeIsActive())
        {
            //allows driver to pick front of robot based on whether he is launching or driving around
            if(driver1.isButtonPressed(Button.DPAD_LEFT))
            {
                leftButtonPusherAsFront = true;
            }
            else if(driver1.isButtonPressed(Button.DPAD_UP))
            {
                leftButtonPusherAsFront = false;
            }

            driveRobotWithJoysticks(gamepad1.left_stick_x,    //local x motion power
                                    gamepad1.left_stick_y,     //local y motion power
                                    gamepad1.right_stick_x / 2); //rotation power; divided by 2 to reduce our robot's high rotational velocity

            //intake balls with collector; drivers must hold buttons to collect
            if (driver2.isButtonPressed(Button.X))
            {
                collectorMotor.setPower(1.0);
                //we don't want the 2nd stage servo to reverse and pull out a particle already in the trough
                collectorServo.setPosition(0.5);
            }
            else if (driver2.isButtonPressed(Button.B))
            {
                collectorMotor.setPower(-1.0);
                collectorServo.setPosition(0.0);
            }
            else
            {
                collectorMotor.setPower(0.0);
                collectorServo.setPosition(0.5);
            }

            //pulls back launcher
            if (driver2.isButtonPressed(Button.DPAD_DOWN))
            {
                launcher.pullback();
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

            idle();
        }
    }
}
