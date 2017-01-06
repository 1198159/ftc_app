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

        setRobotStartingOrientation(90.0);

        drive.robotLocation = new Transform2D(2.395, 0.210, 90.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        //vuforia is not reliably available yet, so we must use encoders at first
        //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

        drive.moveRobot(0.5, 1.0, -0.0);

        pause(1400);

        stopAllDriveMotors();

        turnTo(true, 0.0);

        stopAllDriveMotors();

        //presses beacon 1
        pause(1000);

        vuforiaAlign(false, true, 1.524, 0.0);

        drive.moveRobot(0.0, 0.2, 0.0);

        pause(1000);

        stopAllDriveMotors();

        AlignWithBeacon(false, 1.524);

        drive.moveRobot(0.0, 0.10, 0.0);

        pause(2500);

        stopAllDriveMotors();
        //

        drive.moveRobot(0.0, -0.2, 0.0);
        pause(1500);

        stopAllDriveMotors();

        drive.moveRobot(0.3, 0.0, 0.0);
        pause(3000);

        //presses beacon 2
        pause(1000);
        vuforiaAlign(false, true, 2.700, 0.0);

        stopAllDriveMotors();

        AlignWithBeacon(false, 2.700);

        drive.moveRobot(0.0, 0.10, 0.0);
        pause(2000);

        drive.moveRobot(0.0, -0.5, 0.0);
        pause(3000);

        stopAllDriveMotors();
        //
    }
}
