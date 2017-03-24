package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    TeleOp program used for testing functions such as turnTo and navigateTo; extends MasterAutonomous to allow access to
    turning and vuforia navigation
*/
@TeleOp(name="Test Autonomous", group="6220")
public class AutonomousTest extends MasterAutonomous {
    ElapsedTime timer = new ElapsedTime();

    //CodeReview: Define an enum for reading/writing the elements of your lastBtn array instead of using magic numbers in your code.
    //temporary tap trigger variable
    //                                   a      b      x      y
    boolean lastBtn[] = new boolean[]{false, false, false, false};

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        //the robot is placed in front of the blue1 beacon when starting the test
        drive.robotLocation = new Transform2D(2.638, 1.500, 0.0);

        setRobotStartingOrientation(0.0);

        waitForStart();

        //Start tracking vision targets
        vuforiaHelper.startTracking();

        //delay to allow vuforia to get ready
        pause(1000);

        while (opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            //TODO: adjust encoder function
            //values are displayed for testing purposes
            //updateLocationUsingEncoders(eTime);

            //navigation test for y direction (relative to the robot)
            if (gamepad2.left_stick_y < -0.1)
            {
                vuforiaDriveToPosition(3.300, 1.500);
            }
            //button pressing test
            if (gamepad2.left_bumper)
            {
                vuforiaDriveToPosition(0.001 * Constants.MM_FIELD_SIZE - 0.176, 1.524);
                activateBeacon(false);
            }
            if (gamepad2.dpad_down)
            {
                double a[] = drive.navigateTo(1.0, 0.0);
                telemetry.addData("Navigate To: ", a[0]);
                telemetry.addData("Navigate To: ", a[1]);
                telemetry.addData("Naviagte To: ", a[2]);
                telemetry.update();
            }
            if (gamepad2.a)
            {
                drive.moveRobot(0.0, 0.1, 0.0);
            }
            if (gamepad2.y)
            {
                drive.moveRobot(0.0, -0.1, 0.0);
            }
            if (gamepad2.b)
            {
                drive.moveRobot(0.1, 0.0, 0.0);
            }
            if (gamepad2.x)
            {
                drive.moveRobot(-0.1, 0.0, 0.0);
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

    //HERE FOR AUTONOMOUS TESTING PURPOSES
    //CodeReview: This method is used in several autonomous opmodes. It should probably
    //            move into MasterAutonomous.
    //once at a beacon, we use this function to align with it
    //we use this function to determine the color of either side of the beacon and activate it for the proper side
    public void activateBeacon(boolean redSide /*, double position*/) throws InterruptedException
    {
        if (redSide)
        {
            pause(1000);

            turnTo(true, 90.0);

            float[] colorLeftSide = new float[]{0,0,0};
            float[] colorRightSide = new float[]{0,0,0};

            pause(1000);

            //old beacon color sampling locations in comments
            //Color.colorToHSV(vuforiaHelper.getPixelColor(-50, 185, 30), colorLeftSide);
            //Color.colorToHSV(vuforiaHelper.getPixelColor(50, 185, 30), colorRightSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(-127, 92, 0), colorLeftSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(127, 92, 0), colorRightSide);

            //TODO: implement sampling of many pixels
            /*for(int i = 1; i <=40; i++)
            {
                for(int j = 1; j <= 40; j++)
                {
                    float[] tempColorLeft = new float[3];
                    Color.colorToHSV(vuforiaHelper.getPixelColor(-127 - j, 92 -i, 0), tempColorLeft);
                    Color.colorToHSV(vuforiaHelper.getPixelColor(127-j, 92-i, 0), colorRightSide);
                    int weight = i + j;
                    colorLeftSide[0] = ((colorLeftSide[0] * weight) + tempColorLeft[0]) / (weight + 1);
                }
            }*/

            //Red can be anywhere from 270 to 360 or 0 to 90.  Adding 360 ensures that the red side's
            //value is always greater than the blue side's, thus creating a positive value when blue is
            //subtracted from red and allowing the robot to activate the correct side of the beacon
            if(colorLeftSide[0] <= 90)
            {
                colorLeftSide[0] += 360;
            }
            if(colorRightSide[0] <= 90)
            {
                colorRightSide[0] += 360;
            }

            //picks a side and activates beacon based on the color of the beacon
            if(colorLeftSide[0] - colorRightSide[0] < 0)  //if right side is red
            {
                //vuforiaAlign(true, true, position + Constants.BEACON_PRESS_OFFSET, 0.0);
                beaconServo.setPosition(-0.5);
            }
            else if(colorLeftSide[0] - colorRightSide[0] > 0)  //if left side is red
            {
                //vuforiaAlign(true, true, position - Constants.BEACON_PRESS_OFFSET, 0.0);
                beaconServo.setPosition(0.5);
            }
            else
            {
                //if vuforia didn't find the color of the beacon, it tries again
                activateBeacon(true);
            }

            stopAllDriveMotors();
        }
        else
        {
            pause(1000);

            turnTo(true, 0.0);

            float[] colorLeftSide = new float[3];
            float[] colorRightSide = new float[3];

            pause(1000);

            //Color.colorToHSV(vuforiaHelper.getPixelColor(-50, 185, 30), colorLeftSide);
            //Color.colorToHSV(vuforiaHelper.getPixelColor(50, 185, 30), colorRightSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(-127, 92, 0), colorLeftSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(127, 92, 0), colorRightSide);

            /*for(int i = 1; i <=40; i++)
            {
                for(int j = 1; j <= 40; j++)
                {
                    Color.colorToHSV(vuforiaHelper.getPixelColor(-127 - j, 92 -i, 0), colorLeftSide);
                    Color.colorToHSV(vuforiaHelper.getPixelColor(127-j, 92-i, 0), colorRightSide);
                }
            }*/

            //Red can be anywhere from 270 to 360 or 0 to 90.  Adding 360 ensures that the red side's
            //value is always greater than the blue side's, thus creating a positive value when blue is
            //subtracted from red and allowing the robot to drive to the correct side of the beacon
            if(colorLeftSide[0] <= 90)
            {
                colorLeftSide[0] += 360;
            }
            if(colorRightSide[0] <= 90)
            {
                colorRightSide[0] += 360;
            }

            //picks a side and navigates based on the color of the beacon
            if(colorLeftSide[0] - colorRightSide[0] < 0)  //if right side is red
            {
                //vuforiaAlign(false, true, position + Constants.BEACON_PRESS_OFFSET, 0.0);
                beaconServo.setPosition(0.5);
            }
            else if(colorLeftSide[0] - colorRightSide[0] > 0)  //if left side is red
            {
                //vuforiaAlign(false, true, position - Constants.BEACON_PRESS_OFFSET, 0.0);
                beaconServo.setPosition(-0.5);
            }
            else
            {
                //if vuforia didn't find the color of the beacon, it tries again
                activateBeacon(false);
            }

            stopAllDriveMotors();
        }
    }
}
