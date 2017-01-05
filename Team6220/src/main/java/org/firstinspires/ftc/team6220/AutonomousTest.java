package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    TeleOp program used for testing functions such as turnTo and NavigateTo; extends MasterAutonomous to allow access to
    turning and vuforia navigation
*/
@TeleOp(name="Test Autonomous", group="6220")
public class AutonomousTest extends MasterAutonomous
{
    ElapsedTime timer = new ElapsedTime();

    //CodeReview: Define an enum for reading/writing the elements of your lastBtn array instead of using magic numbers in your code.
    //temporary tap trigger variable
    //                                   a      b      x      y
    boolean lastBtn[] = new boolean[]{false, false, false, false};

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        //initialize vuforia
        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();

        //the robot is placed in front of the blue1 beacon when starting the test
        drive.robotLocation = new Transform2D(2.638, 1.500, 0.0);

        setRobotStartingOrientation(0.0);

        beaconActivationAngle = 0.0;

        waitForStart();

        //Start tracking vision targets
        vuforiaHelper.startTracking();

        //delay to allow vuforia to get ready
        pause(1000);

        while (opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            //TODO adjust encoder function
            //values are displayed for testing purposes
            //updateLocationUsingEncoders(eTime);

            //@TODO test; not working
            //navigation test for y direction (relative to the robot)
            if (gamepad2.left_stick_y < -0.1)
            {
                vuforiaDriveToPosition(3.300, 1.500, 0.0);
            }

            //@TODO test; not working
            //navigation test for x direction (relative to the robot)
            if (gamepad2.left_bumper)
            {
                vuforiaAlign(false, true, 1.524, 0.0);

                drive.moveRobot(0.0, 0.2, 0.0);

                pause(1000);

                stopAllDriveMotors();

                AlignWithBeacon(1.524);

                drive.moveRobot(0.0, 0.10, 0.0);

                pause(2500);

                stopAllDriveMotors();
            }

            //navigation test for rotation
            if (gamepad2.right_bumper)
            {
                turnTo(true, 90.0);
            }

            /*
            if (gamepad2.x && !lastBtn[2])
            {
                motorToggler.toggleMotor();
            }

            if (gamepad2.b && !lastBtn[1])
            {
                motorTogglerReverse.toggleMotor();
            }
            */

            lastBtn[0] = gamepad2.a;
            lastBtn[1] = gamepad2.b;
            lastBtn[2] = gamepad2.x;
            lastBtn[3] = gamepad2.y;

            idle();
        }
    }

    //CodeReview: This method is used in several autonomous opmodes. It should probably
    //            move into MasterAutonomous.
    //once at a beacon, we use this function to align with it
    //HERE FOR AUTONOMOUS TESTING PURPOSES
    public void AlignWithBeacon(double yPosition) throws InterruptedException
    {
        //int colorLeftSide = vuforiaHelper.getPixelColor(-40, 230, 30);
        //int colorRightSide = vuforiaHelper.getPixelColor(40, 230, 30);

        pause(500);

        turnTo(false, 0.0);

        float[] colorLeftSide = new float[3];
        float[] colorRightSide = new float[3];

        pause(1000);

        Color.colorToHSV(vuforiaHelper.getPixelColor(-50, 185, 30), colorLeftSide);
        Color.colorToHSV(vuforiaHelper.getPixelColor(50, 185, 30), colorRightSide);

        //Red can be anywhere from 270 to 360 or 0 to 90.  Adding 360 ensures that the red side's
        //value is always greater than the blue side's, thus creating a positive value when blue is
        //subtracted from red and allowing the robot to drive to the correct side of the beacon
        if(colorLeftSide[0] < 90)
        {
            colorLeftSide[0] += 360;
        }
        if(colorRightSide[0] < 90)
        {
            colorRightSide[0] += 360;
        }

        //picks a side and navigates based on the color of the beacon
        if(colorLeftSide[0] - colorRightSide[0] < 0)
        {
            vuforiaAlign(false, true, yPosition + Constants.BEACON_PRESS_OFFSET, 0.0);
        }
        else if(colorLeftSide[0] - colorRightSide[0] > 0)
        {
            vuforiaAlign(false, true, yPosition - Constants.BEACON_PRESS_OFFSET, 0.0);
        }
        else
        {
            //if vuforia didn't find the color of the beacon, it tries again
            AlignWithBeacon(yPosition);
        }

        stopAllDriveMotors();
    }
}
