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

        setRobotStartingOrientation(0.0);

        beaconActivationAngle = 90.0;

        drive.robotLocation = new Transform2D(0.210, 2.395, 0.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        //vuforia is not reliably available yet, so we must use encoders at first
        //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

        drive.moveRobot(-0.5, 1.0, 0.0);
        pause(1400);

        stopAllDriveMotors();

        turnTo(true, 90.0);

        stopAllDriveMotors();

        drive.moveRobot(0.0, 0.1, 90.0);
        pause(1500);

        //presses beacon 1
        pause(500);
        vuforiaAlign(true, true, 1.524, 90.0);

        stopAllDriveMotors();

        pause(500);
        AlignWithBeacon(true, 1.524);

        drive.moveRobot(0.0, 0.1, 90.0);
        pause(2500);

        stopAllDriveMotors();
        //

        drive.moveRobot(0.0, -0.2, 90.0);
        pause(1000);

        stopAllDriveMotors();

        drive.moveRobot(-0.25, 0.0, 90.0);
        pause(1200);

        stopAllDriveMotors();

        //presses beacon 2
        pause(1000);
        vuforiaAlign(true, true, 2.700, 90.0);

        stopAllDriveMotors();

        AlignWithBeacon(true, 2.700);

        drive.moveRobot(0.0, 0.10, 90.0);
        pause(2000);

        drive.moveRobot(0.0, -0.5, 90.0);
        pause(3000);

        stopAllDriveMotors();
        //
    }
}
