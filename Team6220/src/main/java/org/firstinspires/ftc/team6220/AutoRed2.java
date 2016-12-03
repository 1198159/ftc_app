package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
    Standard autonomous for red side
*/

@Autonomous(name = "AutoRed2", group = "Autonomous")
public class AutoRed2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        setRobotStartingOrientation(0.0);

        drive.robotLocation = new Transform2D(0.210, 2.395, 0.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        //vuforia is not reliably available yet, so we must use encoders at first
        //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

        drive.moveRobot(-0.5, 1.0, 0.0);

        pause(1400);

        stopAllDriveMotors();

        turnTo(90.0);

        stopAllDriveMotors();

        pause(1000);

        vuforiaDriveToPosition(1.500, 3.428, 90.0);

        pause(1000);

        ActivateBeacon(1.500);

        //2.74, 2.6
        vuforiaDriveToPosition(2.700, 3.428, 90.0);

        ActivateBeacon(2.700);

        turnTo(45);

        vuforiaDriveToPosition(1.880, 2.313, 45.0);
    }

    //used to drive to and press the beacon after determining the correct side to press; used inside
    //ActivateBeacon to shorten code
    private void DriveToBeacon(double positionOffset, double xPosition) throws InterruptedException
    {
        vuforiaDriveToPosition(xPosition + positionOffset, 3.000, 90.0);

        turnTo(-90.0);

        drive.moveRobot(0.0, -0.2, 0.0);

        pause(800);

        stopAllDriveMotors();

        //navigateUsingEncoders(new Transform2D(xPosition - 0.150, 3.318, -90.0 - headingOffset));

        //TODO replace later
        drive.moveRobot(0.0, 1.0, 0.0);

        pause(200);

        stopAllDriveMotors();

        turnTo(90.0);

        vuforiaDriveToPosition(xPosition, 2.600, 90.0);
    }

    //once at a beacon, we use this function to press it
    private void ActivateBeacon(double xPosition) throws InterruptedException
    {
        float[] leftSideColor = new float[3];
        float[] rightSideColor = new float[3];
        Color.colorToHSV(vuforiaHelper.getPixelColor(-40, 170, 30), leftSideColor);
        Color.colorToHSV(vuforiaHelper.getPixelColor(40, 170, 30), rightSideColor);

        if(leftSideColor[0] < 90)
        {
            leftSideColor[0] += 360;
        }
        if(rightSideColor[0] < 90)
        {
            rightSideColor[0] += 360;
        }

        if(leftSideColor[0] - rightSideColor[0] > 0)
        {
            DriveToBeacon(-0.110, xPosition);
        }
        else
        {
            DriveToBeacon(0.110, xPosition);
        }

    }
}
