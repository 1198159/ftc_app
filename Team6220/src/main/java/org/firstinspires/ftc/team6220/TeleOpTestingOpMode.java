package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    TeleOp program used for testing functions such as turnTo and navigateTo
*/
@TeleOp(name="TeleOpTestingOpMode", group="6220")
public class TeleOpTestingOpMode extends MasterTeleOp
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

        vuforiaHelper.setupVuforia();

        //the robot is placed in this location when starting the test
        drive.robotLocation = new Transform2D(1.500, 2.438, 90.0);

        setRobotStartingOrientation(90.0);

        waitForStart();

        //delay to allow vuforia to get ready
        pause(1000);

        while (opModeIsActive())
        {
            //values are displayed for testing purposes
            updateLocationUsingEncoders();

            //@TODO test; not working
            //navigation test for y direction; y stick is reversed
            while (gamepad2.left_stick_y < -0.1)
            {
                float[] l = vuforiaHelper.getRobotLocation();

                //we use this to convert our location from an array to a transform
                drive.robotLocation.SetPositionFromFloatArray(l);

                //location directly in front of beacon 1
                double[] m = drive.navigateTo(new Transform2D(1.500, 3.428, 90.0));

                telemetry.addData("robot location: ", drive.robotLocation);
                telemetry.update();

                idle();
            }

            //@TODO test; not working
            //navigation test for x direction
            while (gamepad2.left_stick_x > 0.1)
            {
                float[] l = vuforiaHelper.getRobotLocation();

                //we use this to convert our location from an array to a transform
                drive.robotLocation.SetPositionFromFloatArray(l);

                //location near center of field and to the right of beacon 1
                double[] m = drive.navigateTo(new Transform2D(1.800, 2.438, 90.0));

                telemetry.addData("robot location: ", drive.robotLocation);
                telemetry.update();

                idle();
            }

            //navigation test for rotation
            //TODO test; not working
            if(gamepad2.right_bumper)
            {
                //location 2 feet out from the wall with same x and y coordinate as AutoRed2 starting position, but different rot
                //double[] m = drive.navigateTo(new Transform2D(0.609, 2.395, -90.0));
                turnTo(0.0);
            }

            //intake balls with collector; drivers must hold buttons to collect
            if (gamepad2.x)
            {
                collectorMotor.setPower(1.0);
            }
            else if (gamepad2.b)
            {
                collectorMotor.setPower(-1.0);
            }
            else
            {
                collectorMotor.setPower(0.0);
            }

            lastBtn[0] = gamepad2.a;
            lastBtn[1] = gamepad2.b;
            lastBtn[2] = gamepad2.x;
            lastBtn[3] = gamepad2.y;

            telemetry.addData("LeftStickY: ", gamepad2.left_stick_y);
            telemetry.addData("RightStickY: ", gamepad2.right_stick_y);
            telemetry.addData("RightBumper: ", gamepad2.right_bumper);
            telemetry.update();
            
            idle();
        }
    }
}
