package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
    Standard autonomous for red side
*/

//todo: combine autonomous programs into one
@Autonomous(name = "AutoRed2", group = "Autonomous")
public class AutoRed2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        setRobotStartingOrientation(90.0);

        beaconActivationAngle = 90.0;

        drive.robotLocation = new Transform2D(0.210, 2.395, 90.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        //vuforia is not reliably available yet, so we must use encoders at first
        //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

        drive.moveRobot(0.25, 0.0, 90.0);
        pause(800);

        stopAllDriveMotors();

        //shoots a ball
        launcher.pullback();
        pauseWhileUpdating(3.0);
        launcher.loadParticle();
        pauseWhileUpdating(2.0);
        launcher.launchParticle();
        pauseWhileUpdating(2.0);
        launcher.pullBackMotor.setPower(0.0);
        //

        drive.moveRobot(0.8, 0.5, 90.0);
        pause(1500);

        stopAllDriveMotors();

        turnTo(true, 90.0);

        drive.moveRobot(0.0, 0.1, 90.0);
        pause(1200);

        stopAllDriveMotors();

        //presses beacon 1
        pause(500);
        vuforiaAlign(true, true, 1.524, 90.0);

        pause(500);
        AlignWithBeacon(true, 1.524);

        drive.moveRobot(0.0, 0.1, 90.0);
        pause(2500);

        stopAllDriveMotors();
        //

        //pushes cap ball
        drive.moveRobot(0.1, -0.2, 90.0);
        pause(2500);

        stopAllDriveMotors();

        turnTo(true, 45.0);

        turnTo(true, 90.0);

        drive.moveRobot(0.0, -0.2, 90.0);
        pause(1500);

        stopAllDriveMotors();
        //
    }
}
