package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Blue Glyph", group = "Swerve")
// @Disabled

public class AutonomousBlueGlyph extends MasterAutonomous
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

            if (isPosLeft) telemetry.addData("Alliance: ", "Blue Left");
            else telemetry.addData("Alliance: ", "Blue Right");

            VuforiaDetect.GetVumark();
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", VuforiaDetect.vuMark);

            telemetry.update();
            idle();
        }
        telemetry.update();

// Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoRuntime.reset(); // set the 30 second timer

        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

// START OF AUTONOMOUS

        // grab the glyph
        motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        closeGG();
        sleep(200);
        raiseGM();
        sleep(200);

        Kmove = 1.0/1200.0;
        TOL = 100.0;
        TOL_ANGLE = 2;
        Kpivot = 1/100.0;

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
            pivotWithReference(-15, refAngle, 0.15, 0.5); // then pivot right
            sleep(200);
            servoJewel.setPosition(JEWEL_INIT); // move servo back
            sleep(200);
            pivotWithReference(0, refAngle, 0.15, 0.5); // then pivot back
            sleep(200);
        }
        else // if the left jewel is red,
        {
            pivotWithReference(15, refAngle, 0.15, 0.5); // then pivot left
            sleep(200);
            servoJewel.setPosition(JEWEL_INIT); // move servo back
            sleep(200);
            pivotWithReference(0, refAngle, 0.15, 0.5); // then pivot back
            sleep(200);
        }

        if (isPosLeft) // BLUE LEFT
        {
            moveTimed(0.6, 0, 1500); // right
            sleep(200);
            moveTimed(0, -0.3, 550); // back
        }
        else // BLUE RIGHT
        {
            move(280, 0, 0.2, 0.4, 2.5); // move off the ramp
            sleep(200);
            pivotWithReference(0, refAngle, 0.15, 0.5);
            sleep(200);
            Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
            moveMaintainHeading(230, 0, 0, refAngle, 0.15, 0.6, 6);
            sleep(200);
            Kpivot = 1/100.0;
            pivotWithReference(181, refAngle, 0.15, 0.55);
            sleep(200);
            move(0, -200, 0.1, 0.3, 2.5); // push the glyph in
            sleep(200);
        }

        motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        openGG(-500); // open the GG a little bit

        sleep(200);
        move(0, 150, 0.1, 0.3, 0.7); // back up from the cryptobox
        motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        openGG(minGGPos);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
