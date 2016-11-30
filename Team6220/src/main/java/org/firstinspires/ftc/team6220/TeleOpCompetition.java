package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    Competition configuration for driving robot.
    Pilot controls:


    Co-pilot controls:



*/
@TeleOp(name="TeleOpCompetition", group="6220")
public class TeleOpCompetition extends MasterTeleOp
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

        drive.robotLocation = new Transform2D(0.609, 2.395, 0.0);

        //@TODO test
        //navigation test for y direction
        while(gamepad2.left_stick_y > 0.1)
        {
            pause(1000);

            float[] l = vuforiaHelper.getRobotLocation();

            //we use this to convert our location from an array to a transform
            drive.robotLocation.SetPositionFromFloatArray(l);

            //location directly in front of beacon 1
            double[] m = drive.navigateTo(new Transform2D(1.500, 3.428, 90.0));

            telemetry.addData("robot location: ", drive.robotLocation);
            telemetry.update();
        }

        //@TODO test
        //navigation test for x direction
        while(gamepad2.left_stick_x > 0.1)
        {
            pause(1000);

            float[] l = vuforiaHelper.getRobotLocation();

            //we use this to convert our location from an array to a transform
            drive.robotLocation.SetPositionFromFloatArray(l);

            //location near center of field and to the right of beacon 1
            double[] m = drive.navigateTo(new Transform2D(1.800, 2.438, 90.0));

            telemetry.addData("robot location: ", drive.robotLocation);
            telemetry.update();
        }

        //navigation test for rotation
        if(gamepad2.right_stick_x > 0.1)
        {
            //location 2 feet out from the wall with same x and y coordinate as AutoRed2 starting position, but different rot
            //double[] m = drive.navigateTo(new Transform2D(0.609, 2.395, -90.0));
            //turnTo(90.0);
        }

        waitForStart();

        while (opModeIsActive())
        {
            driveRobotWithJoysticks(-gamepad1.left_stick_x,    //local x motion power; reversed
                                     gamepad1.left_stick_y,     //local y motion power
                                     gamepad1.right_stick_x/2);    //rotation power; reversed

            //values are displayed for testing purposes
            //updateLocationUsingEncoders();

            if (gamepad2.x && !lastBtn[2])
            {
                motorToggler.toggleMotor();
            }

            if (gamepad2.b && !lastBtn[1])
            {
                motorTogglerReverse.toggleMotor();
            }

            lastBtn[0] = gamepad2.a;
            lastBtn[1] = gamepad2.b;
            lastBtn[2] = gamepad2.x;
            lastBtn[3] = gamepad2.y;

            idle();
        }
    }
}
