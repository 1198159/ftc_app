package org.firstinspires.ftc.team6220;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    TeleOp program used for testing functions such as turnTo and navigateTo; extends MasterAutonomous to allow access to
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
                vuforiaAlign("blue", "x", 1.524, 0.0);

                AlignWithBeacon(1.524);

                pause(1000);

                vuforiaAlign("blue", "y", 3.200, 0.0);

                //drive.moveRobotAtConstantHeading(0.0, -0.2, 0.0, 0.0);

                pause(800);

                stopAllDriveMotors();
            }

            //navigation test for rotation
            if (gamepad2.right_bumper)
            {
                turnTo(90.0);
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
    //once at a beacon, we use this function to press it
    //HERE FOR AUTONOMOUS TESTING PURPOSES
    public void AlignWithBeacon(double yPosition) throws InterruptedException
    {
        int colorLeftSide = vuforiaHelper.getPixelColor(-40, 230, 30);
        int colorRightSide = vuforiaHelper.getPixelColor(40, 230, 30);

        if(Color.blue(colorRightSide) < Color.blue(colorLeftSide))
        {
            vuforiaAlign("blue", "x", 1.800, 0.0);

            stopAllDriveMotors();
        }
        else
        {
            vuforiaAlign("blue", "x", 1.200, 0.0);

            stopAllDriveMotors();
        }
    }
}
