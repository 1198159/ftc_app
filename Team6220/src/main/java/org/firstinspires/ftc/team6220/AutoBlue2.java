package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
    Standard autonomous for blue side
*/

//todo: combine autonomous programs into one

@Autonomous(name = "AutoBlue2", group = "Autonomous")
public class AutoBlue2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        setRobotStartingOrientation(180.0);

        beaconActivationAngle = 0.0;

        drive.robotLocation = new Transform2D(2.395, 0.210, 180.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        //TODO redo old code and place inside outline
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
        AlignWithBeacon(false, 1.524);

        drive.moveRobot(0.0, 0.1, 0.0);
        pause(2500);

        stopAllDriveMotors();
        //
        */

        //@TODO outline for new auto program
        //1  launch 2 particles

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

        //2  drive forward until same y coordinate as beacon

        //3  rotate 180 degrees so robot is facing beacon

        //4  drive forward until close to beacon

        //5  determine color of each side of beacon

        //6  pivot servo based on beacon color

        //7  ensure beacon is correct color

        //8  back away from beacon 1

        //9  drive to beacon 2

        //10 drive forward until close to beacon

        //11 determine color of each side of beacon

        //12 pivot servo based on beacon color

        //13 back away from beacon

        //14 drive while rotating counterclockwise to knock cap ball and park

    }
}
