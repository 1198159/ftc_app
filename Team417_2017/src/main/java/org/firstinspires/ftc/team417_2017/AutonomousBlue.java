package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Blue", group = "Swerve")
// @Disabled

public class AutonomousBlue extends MasterAutonomous
{
    VuforiaDetection VuforiaDetect = new VuforiaDetection();
    double parkSpeed = 0.5;


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

            if (isPosLeft) telemetry.addData("Alliance: ", "Blue Left");
            else telemetry.addData("Alliance: ", "Blue Right");

            telemetry.update();
            idle();
        }
        telemetry.update();

        if (isPosLeft)
        {
            // OPTION BLUE LEFT
            parkSpeed = -0.6;
        }
        else
        {
            // OPTION BLUE RIGHT
            parkSpeed = 0.6;
        }

// Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoRuntime.reset(); // set the 30 second timer

// START OF AUTONOMOUS

        // lower the servos, putting jewel manipulator into position
        servoJewelStore.setPosition(JEWEL_STORE_LOW);
        sleep(200);
        servoJewelDrop.setPosition(JEWEL_DROP_LOW);
        sleep(200);

        Kmove = 1.0/1200.0;
        MINSPEED = 0.3;
        TOL_ANGLE = 3.0;
        TOL = 60;
        Kpivot = 1/150.0;
        PIVOT_MINSPEED = 0.2;

        if (VuforiaDetect.isVisible()) // if vuMark seen is not unknown,
        {
            VuforiaDetect.GetVumark();
            VuforiaDetect.GetLeftJewelColor();
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", VuforiaDetect.vuMark);
                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (VuforiaDetect.pose != null) // if the pose is NOT null,
            {
                telemetry.addData("leftHue ", VuforiaDetect.avgLeftJewelColor);
                telemetry.addData("rightHue ", VuforiaDetect.avgRightJewelColor);
                telemetry.addData("isLeftJewelBlue", VuforiaDetect.isLeftJewelBlue);
            }
        }
        else
        {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();

        sleep(200);

        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle;

        if(VuforiaDetect.isLeftJewelBlue) // if the left jewel is blue,
        {
            pivotWithReference(-30, refAngle, 0.5); // then pivot right
            sleep(200);
            servoJewelStore.setPosition(JEWEL_STORE_INIT);
            sleep(200);
            servoJewelDrop.setPosition(JEWEL_DROP_INIT);
            sleep(200);
            pivotWithReference(0, refAngle, 0.5); // then pivot back
            sleep(200);
        }
        else // if the left jewel is red,
        {
            pivotWithReference(30, refAngle, 0.5); // then pivot left
            sleep(200);
            servoJewelStore.setPosition(JEWEL_STORE_INIT);
            sleep(200);
            servoJewelDrop.setPosition(JEWEL_DROP_INIT);
            sleep(200);
            pivotWithReference(0, refAngle, 0.5); // then pivot back
            sleep(200);
        }


        if (isPosLeft) // BLUE LEFT
        {
            moveTimed(0.6, 0, 1650); // right
            sleep(200);
            moveTimed(0, -0.3, 550); // back
        }
        else // BLUE RIGHT
        {
            moveTimed(0.6, 0, 1750); // right
            sleep(200);
            moveTimed(0, 0.4, 650); // forwards
        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();

        sleep(6000);

    }
}
