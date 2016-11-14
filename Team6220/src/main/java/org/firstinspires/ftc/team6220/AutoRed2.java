package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This opmode is used to experiment with autonomous ideas before implementing them in our competition code.
 */

@Autonomous(name = "AutoRed2", group = "Autonomous")
public class AutoRed2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        headingOffset = 0.0;

        drive.robotLocation = new Transform2D(0.210, 2.395, 0.0 + headingOffset);

        waitForStart();

        //vuforia is not reliably available yet, so we must use encoders at first
        navigateUsingEncoders(new Transform2D(1.524, 2.395, 90));

        ActivateBeacon();

        vuforiaDriveToPosition(2.743, 2.395, 90);

        ActivateBeacon();

        vuforiaDriveToPosition(2.800, 2.100, 90);
    }

    //once at a beacon, we use this function to press it
    private void ActivateBeacon() throws InterruptedException
    {
        int colorLeftSide = vuforiaHelper.getPixelColor(-60, 230, 30);
        int colorRightSide = vuforiaHelper.getPixelColor(60, 230, 30);

        if(Color.blue(colorRightSide) > Color.blue(colorLeftSide))
        {
            vuforiaDriveToPosition(1.350, 3.318, 90);

            turnTo(180);

            drive.moveRobot(0.2, 0.0, 0.0);

            wait(1000);

            navigateUsingEncoders(new Transform2D(1.350, 3.318, 180));

            //this turnTo(90) is necessary since otherwise, vuforia will not see the vision target
            //when attempting to move, and thus will not work
            turnTo(90);
        }
        else
        {
            vuforiaDriveToPosition(1.650, 3.318, 90);

            turnTo(180);

            drive.moveRobot(0.2, 0.0, 0.0);

            wait(1000);

            navigateUsingEncoders(new Transform2D(1.650, 3.318, 180));

            //see comment above
            turnTo(90);
        }
    }
}
