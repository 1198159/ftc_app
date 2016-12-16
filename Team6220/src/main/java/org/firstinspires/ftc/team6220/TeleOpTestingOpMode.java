package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    TeleOp program used for testing functions such as turnTo and navigateTo; extends MasterAutonomous to allow access to
    turning and vuforia navigation
*/
@TeleOp(name="TeleOpTestingOpMode", group="6220")
public class TeleOpTestingOpMode extends MasterAutonomous
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
        drive.robotLocation = new Transform2D(2.438, 1.500, 0.0);

        setRobotStartingOrientation(0.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        //delay to allow vuforia to get ready
        pause(1000);

        while (opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            //values are displayed for testing purposes
            updateLocationUsingEncoders(eTime);

            //@TODO test; not working
            //navigation test for y direction
            while ((gamepad2.left_stick_y > 0.1) && opModeIsActive())
            {
                float[] l = vuforiaHelper.getRobotLocation();

                //we use this to convert our location from an array to a transform
                drive.robotLocation.SetPositionFromFloatArray(l);

                //location directly in front of beacon 1
                double[] m = drive.navigateTo(new Transform2D(3.428, 1.500, 0.0));

                telemetry.addData("robot location: ", drive.robotLocation);
                telemetry.update();

                idle();
            }

            //@TODO test; not working
            //navigation test for x direction
            while ((gamepad2.left_stick_x > 0.1) && opModeIsActive())
            {
                float[] l = vuforiaHelper.getRobotLocation();

                //we use this to convert our location from an array to a transform
                drive.robotLocation.SetPositionFromFloatArray(l);

                //location near center of field and to the right of beacon 1
                double[] m = drive.navigateTo(new Transform2D(2.438, 1.800, 0.0));

                telemetry.addData("robot location: ", drive.robotLocation);
                telemetry.update();

                idle();
            }

            //navigation test for rotation
            if(gamepad2.right_bumper)
            {
                //same as start location, but with different rotation
                double[] m = drive.navigateTo(new Transform2D(2.438, 1.500, 90.0));

                //turnTo(180.0);

                telemetry.addData("FrontRightPow: ", driveAssemblies[FRONT_RIGHT]);
                telemetry.update();
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
}
