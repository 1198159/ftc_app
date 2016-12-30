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

        turnTo("imu", 90.0);

        stopAllDriveMotors();

        pause(1000);

        vuforiaDriveToPosition(1.500, 3.428, 90.0);

        pause(1000);

        ActivateBeacon(1.500);

        //2.74, 2.6
        vuforiaDriveToPosition(2.700, 3.428, 90.0);

        ActivateBeacon(2.700);

        turnTo("imu", 45);

        vuforiaDriveToPosition(1.880, 2.313, 45.0);

        /*
        just in case the code above does not work
        drive.moveRobot(0.0, 0.5, 0.0);
        drive.moveRobot(0.0, - 0.2, 0.0);
        drive.moveRobot(0.0, 0.2, 0.0);
        */


        //testing section; use later in season to push ball
        /*
        turnTo(-15.0 - headingOffset);

        navigateUsingEncoders(new Transform2D(0.570, 3.108, 45.0 - headingOffset));

        wait(100);

        navigateUsingEncoders(new Transform2D(1.800, 2.658, 0.0 - headingOffset));

        navigateUsingEncoders(new Transform2D(1.500, 2.658, 90.0 - headingOffset));

        ActivateBeacon(1.500);

        navigateUsingEncoders(new Transform2D(2.743, 2.395, 90.0 - headingOffset));

        ActivateBeacon(2.743);

        vuforiaDriveToPosition(2.800, 2.100, 90.0 - headingOffset);
        */
    }

    //once at a beacon, we use this function to press it
    private void ActivateBeacon(double xPosition) throws InterruptedException
    {
        int colorLeftSide = vuforiaHelper.getPixelColor(-40, 230, 30);
        int colorRightSide = vuforiaHelper.getPixelColor(40, 230, 30);

        if(Color.blue(colorRightSide) > Color.blue(colorLeftSide))
        {
            vuforiaDriveToPosition(xPosition - 0.110, 3.000, 90.0);

            turnTo("imu", -90.0);

            drive.moveRobot(0.0, -0.2, 0.0);

            pause(800);

            stopAllDriveMotors();

            //navigateUsingEncoders(new Transform2D(xPosition- 0.150, 3.318, -90.0 - headingOffset));

            //TODO replace later
            drive.moveRobot(0.0, 1.0, 0.0);

            pause(200);

            stopAllDriveMotors();

            turnTo("imu", 90.0);

            vuforiaDriveToPosition(xPosition, 3.428, 90.0);

        }
        else
        {
            vuforiaDriveToPosition(xPosition + 0.110, 3.000, 90.0);

            turnTo("imu", -90.0);

            drive.moveRobot(0.0, -0.2, 0.0);

            pause(800);

            stopAllDriveMotors();

            //navigateUsingEncoders(new Transform2D(xPosition + 0.150, 3.318, -90.0-headingOffset));

            //TODO replace later
            drive.moveRobot(0.0, 1.0, 0.0);

            pause(200);

            stopAllDriveMotors();

            turnTo("imu", 90.0);

            vuforiaDriveToPosition(xPosition, 3.428, 90.0);
        }

    }
}
