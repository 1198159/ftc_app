package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Blue", group = "Swerve")
// @Disabled

public class AutonomousBlue extends MasterAutonomous
{
    VuforiaDetection VuforiaDetect = new VuforiaDetection();


    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        super.initializeHardware();
        VuforiaDetect.initVuforia(); // initialize Vuforia
        VuforiaDetect.GetVumark(); // gets the vuMark
        if (VuforiaDetect.isVisible()) // if vuMark seen is not unknown,
        {
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


// Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoRuntime.reset(); // set the 30 second timer

// START OF AUTONOMOUS

        // lower the servos, putting jewel manipulator into position
        servoJewelStore.setPosition(JEWEL_STORE_LOW);
        sleep(300);
        servoJewelDrop.setPosition(JEWEL_DROP_LOW);
        sleep(5000);

        Kmove = 1.0/1200.0;
        MINSPEED = 0.3;
        TOL_ANGLE = 3.0;
        TOL = 60;
        Kpivot = 1/150.0;
        PIVOT_MINSPEED = 0.2;

        if (VuforiaDetect.isVisible()) // if vuMark seen is not unknown,
        {
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



        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle;

        if(VuforiaDetect.GetLeftJewelColor()) // if the left jewel is blue,
        {
            pivotWithReference(-90, refAngle, 0.5); // then pivot right
        }
        else // if the left jewel is red,
        {
            pivotWithReference(90, refAngle, 0.5); // then pivot left
        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
