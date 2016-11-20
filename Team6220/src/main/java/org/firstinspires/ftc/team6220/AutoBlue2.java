package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Colew on 11/19/2016.
 */

@Autonomous(name = "AutoBlue2", group = "Autonomous")
public class AutoBlue2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        //this is used to add absolute orientation to each autonomous program
        headingOffset = 0.0;

        drive.robotLocation = new Transform2D(0.210, 2.395, 0.0 - headingOffset);

        waitForStart();

        //vuforia is not reliably available yet, so we must use encoders at first
        //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

        drive.moveRobot(0.7, 1.0, 0.0);

        pause(1500);

        stopAllDriveMotors();

        turnTo(90.0 - headingOffset);

        drive.moveRobot(0.5, 0.0, 0.0);

        pause(800);

        stopAllDriveMotors();

        pause(1000);

        ActivateBeacon(1.500);

        vuforiaDriveToPosition(2.743, 2.600, 90.0 - headingOffset);

        ActivateBeacon(2.743);

        turnTo(45 - headingOffset);

        vuforiaDriveToPosition(1.880, 2.313, 45 - headingOffset);

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
        int colorLeftSide = vuforiaHelper.getPixelColor(-60, 230, 30);
        int colorRightSide = vuforiaHelper.getPixelColor(60, 230, 30);

        if(Color.blue(colorRightSide) > Color.blue(colorLeftSide))
        {
            vuforiaDriveToPosition(xPosition - 0.150, 3.318, 90.0 - headingOffset);

            turnTo(-90.0 - headingOffset);

            drive.moveRobot(-0.2, 0.0, 0.0);

            pause(1000);

            stopAllDriveMotors();

            //navigateUsingEncoders(new Transform2D(xPosition- 0.150, 3.318, -90.0 - headingOffset));

            //TODO replace later
            drive.moveRobot(1.0, 0.0, 0.0);

            pause(200);

            stopAllDriveMotors();

            turnTo(90.0 - headingOffset);

            vuforiaDriveToPosition(xPosition, 2.600, 90.0 - headingOffset);

        }
        else
        {
            vuforiaDriveToPosition(xPosition + 0.150, 3.318, 90.0 - headingOffset);

            turnTo(-90.0 - headingOffset);

            drive.moveRobot(-0.2, 0.0, 0.0);

            pause(1000);

            stopAllDriveMotors();

            //navigateUsingEncoders(new Transform2D(xPosition + 0.150, 3.318, -90.0-headingOffset));

            //TODO replace later
            drive.moveRobot(1.0, 0.0, 0.0);

            pause(200);

            stopAllDriveMotors();

            turnTo(90.0 - headingOffset);

            vuforiaDriveToPosition(xPosition, 2.600, 90.0 - headingOffset);
        }

    }
}
