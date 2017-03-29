package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
    Standard autonomous for blue side
*/

@Autonomous(name = "AutoBlue2", group = "Autonomous")
public class AutoBlue2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        setRobotStartingOrientation(180.0);
        
        drive.robotLocation = new Transform2D(2.395, 0.210, 180.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        /*
        //vuforia is not reliably available yet, so we must use encoders at first
        //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

        pause(200);
        stopAllDriveMotors();

        drive.moveRobot(-1.0, 0.0, 0.0);
        pause(2000);

        stopAllDriveMotors();

        //pushes cap ball
        turnTo(true, 90.0);

        pause(500);
        //

        turnTo(true, 0.0);

        drive.moveRobot(0.2, -0.1, 0.0);
        pause(1200);

        stopAllDriveMotors();

        //presses beacon 1
        //pause(500);
        //vuforiaAlign(false, true, 1.524, 0.0);

        pause(500);
        activateBeacon(false, 1.524);

        drive.moveRobot(0.0, 0.1, 0.0);
        pause(2500);

        stopAllDriveMotors();
        //
        */

        //1 launch 2 particles

            //shoots ball 1
            launcher.pullback();
            pauseWhileUpdating(1.5);
            launcher.launchParticle();
            pauseWhileUpdating(2.0);
            launcher.pullBackMotor.setPower(0.0);
            //

            //loads ball 2
            collectorMotor.setPower(-1.0);
            collectorServo.setPosition(0.0);
            pause(1000);
            collectorMotor.setPower(0.0);
            collectorServo.setPosition(0.5);
            pause(600);
            //

            //shoots ball 2
            launcher.pullback();
            pauseWhileUpdating(1.5);
            launcher.loadParticle();
            pauseWhileUpdating(1.0);
            launcher.launchParticle();
            pauseWhileUpdating(2.0);
            launcher.pullBackMotor.setPower(0.0);
            //

        //2 drive forward until same y coordinate as beacon

            drive.moveRobot(-1.0, 0.0, 0.0);
            pause(2000);

        //3 rotate 180 degrees so robot is facing beacon

            turnTo(true, 0.0);

        //4 drive forward until close to beacon 1

            vuforiaDriveToPosition(0.001 * Constants.MM_FIELD_SIZE - 0.176, 1.524);

        //5 determine color of each side of beacon 1, then press button based on color

            activateBeacon(false);

        //6  back away from beacon 1

            drive.moveRobot(0.0, 0.1, 0.0);
            pause(2500);

        //7 drive to beacon 2

            drive.moveRobot(-1.0, 0.0, 0.0);
            pause(1800);

        //8 drive forward until close to beacon 2

            vuforiaDriveToPosition(0.001 * Constants.MM_FIELD_SIZE - 0.176, 2.724);

        //9 determine color of each side of beacon 2, then press button based on color

            activateBeacon(false);

        //10 back away from beacon 2

            drive.moveRobot(0.0, 0.1, 0.0);
            pause(2500);

        //11 rotate 45 degrees counterclockwise to robot is facing cap ball

            turnTo(true, 45);

        //12 drive forward to knock cap ball and park

            drive.moveRobot(0.0, -1.0, 0.0);
            pause(2000);
    }
}
