package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Red", group = "Swerve")
// @Disabled

public class AutonomousRed extends MasterAutonomous
{
    VuforiaDetection VuforiaDetect = new VuforiaDetection();

    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        super.initializeHardware();
        VuforiaDetect.initVuforia(); // initialize Vuforia
        telemetry.addData("Done: ", "initializing");
        telemetry.update();

        while (!isStarted())
        {
            // select position left or right, from drivers facing the field
            if (gamepad1.x) isPosLeft = true;
            if (gamepad1.b) isPosLeft = false;

            if (isPosLeft) telemetry.addData("Alliance: ", "Red Left");
            else telemetry.addData("Alliance: ", "Red Right");

            VuforiaDetect.GetVumark();
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", VuforiaDetect.vuMark);

            telemetry.update();
            idle();
        }
        telemetry.update();

        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

// Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoRuntime.reset(); // set the 30 second timer

// START OF AUTONOMOUS

        Kmove = 1.0/1200.0;
        MINSPEED = 0.3;
        TOL = 60;
        TOL_ANGLE = 3;
        Kpivot = 1/140.0;
        PIVOT_MINSPEED = 0.15;

        if (VuforiaDetect.isVisible())
        {
            VuforiaDetect.GetVumark(); // move to initialization
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", VuforiaDetect.vuMark);
                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
        }
        else
        {
            telemetry.addData("VuMark", "not visible");
            telemetry.update();
        }

        // lower the servos, putting jewel manipulator into position
        servoJewel.setPosition(JEWEL_LOW);
        sleep(200);

        VuforiaDetect.GetLeftJewelColor(); // calculate if the left jewel is blue or not

        // display the color values for each jewel color
        telemetry.addData("leftHue ", VuforiaDetect.avgLeftJewelColor);
        telemetry.addData("rightHue ", VuforiaDetect.avgRightJewelColor);
        telemetry.update();

        if(VuforiaDetect.isLeftJewelBlue) // if the left jewel is blue,
        {
            pivotWithReference(17, refAngle, 0.15,0.5); // then pivot right
            sleep(200);
            servoJewel.setPosition(JEWEL_INIT);
            sleep(200);
            pivotWithReference(0, refAngle, 0.15,0.5); // then pivot back
            sleep(200);
        }
        else // if the left jewel is red,
        {
            pivotWithReference(-17, refAngle, 0.15, 0.5); // then pivot left
            sleep(200);
            servoJewel.setPosition(JEWEL_INIT);
            sleep(200);
            pivotWithReference(0, refAngle, 0.15, 0.5); // then pivot back
            sleep(200);
        }

        if (isPosLeft) // RED LEFT
        {
            moveTimed(-0.6, 0, 1750); // left
            sleep(200);
            moveTimed(0, 0.4, 650); // forwards
        }
        else // RED RIGHT
        {
            moveTimed(-0.6, 0, 1500); // left
            sleep(200);
            moveTimed(0, -0.3, 550); // back
        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
