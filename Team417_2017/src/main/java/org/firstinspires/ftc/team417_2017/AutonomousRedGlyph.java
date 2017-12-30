package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Autonomous Red Glyph", group = "Swerve")
// @Disabled

public class AutonomousRedGlyph extends MasterAutonomous
{
    VuforiaDetection VuforiaDetect = new VuforiaDetection();
    public RelicRecoveryVuMark VuMark;

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

            VuMark = VuforiaDetect.GetVumark();
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", VuMark);

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
            if (VuMark == RelicRecoveryVuMark.CENTER)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the reference angle
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-100, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                sleep(200);
                Kpivot = 1/100.0;
                pivotWithReference(-140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(200);

                // ALIGN TO CORRECT COLUMN
                move(-115, 0, 0.15, 0.5, 1); // move right
                sleep(200);
                move(0, -220, 0.1, 0.3, 2.5); // push the glyph in
                sleep(200);
            }
            else if (VuMark == RelicRecoveryVuMark.RIGHT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the reference angle
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-310, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                sleep(200);
                Kpivot = 1/100.0;
                pivotWithReference(140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(200);

                // ALIGN TO CORRECT COLUMN
                //move(-60, 0, 0.15, 0.5, 1); // move right
                sleep(200);
                move(0, -260, 0.1, 0.3, 2.5); // push the glyph in
                sleep(200);
            }
            else if (VuMark == RelicRecoveryVuMark.LEFT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-100, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                sleep(200);
                Kpivot = 1/100.0;
                pivotWithReference(-140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(200);

                move(-40, 0, 0.15, 0.5, 1); // move right
                sleep(200);
                move(0, -255, 0.1, 0.3, 2.5); // push the glyph in more, since we didn't shift right
                sleep(200);
            }
            else // If the VuMark is not visible we go for the center
            {
                telemetry.addData("Vumark", "not visible, going for center");

                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-100, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                sleep(200);
                Kpivot = 1/100.0;
                pivotWithReference(-140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(200);

                // ALIGN TO CORRECT COLUMN
                move(-115, 0, 0.15, 0.5, 1); // move right
                sleep(200);
                move(0, -220, 0.1, 0.3, 2.5); // push the glyph in
                sleep(200);
            }

            // BACK UP FROM THE CRYPTOBOX
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            openGG(-500); // open the GG a little bit
            sleep(200);
            move(0, 160, 0.1, 0.3, 0.7); // back up from the cryptobox
            // release the glyph
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            openGG(minGGPos);
            pivotWithReference(0, refAngle, 0.15, 0.55);
            // back up a bit more, to make sure that the robot is not touching the deposited glyph
            move(0, -50, 0.1, 0.3, 0.7);
        }
        else // RED RIGHT
        {
            if (VuMark == RelicRecoveryVuMark.CENTER)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-100, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(200);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                move(-70, 0, 0.15, 0.5, 1); // move right
                move(0, -220, 0.1, 0.3, 2.5); // push the glyph in
                sleep(200);
            }
            else if (VuMark == RelicRecoveryVuMark.RIGHT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-80, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(200);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                move(-185, 0, 0.15, 0.5, 1); // move right
                move(0, -100, 0.1, 0.3, 2.5); // push the glyph in
                sleep(200);
            }
            else if (VuMark == RelicRecoveryVuMark.LEFT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-100, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(200);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                move(55, 0, 0.15, 0.5, 1); // move left
                move(0, -230, 0.1, 0.3, 2.5); // push the glyph in
                sleep(200);
            }
            else // if the VuMark is not visible, just go for the center column
            {
                telemetry.addData("Vumark", "not visible, going for center");
                /*
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(200);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(200);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-100, 0, 0, refAngle, 0.15, 0.6, 2.5); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(200);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                move(-70, 0, 0.15, 0.5, 1); // move right
                move(0, -220, 0.1, 0.3, 2.5); // push the glyph in
                sleep(200);
                */
            }

            // BACK UP FROM THE CRYPTOBOX
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            openGG(-500); // open the GG a little bit
            sleep(200);
            move(0, 175, 0.1, 0.3, 0.7); // back up from the cryptobox
            sleep(100);
            // release the glyph
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            openGG(minGGPos);
            sleep(100);
            pivotWithReference(0, refAngle, 0.15, 0.55);
            sleep(200);
            // back up a bit more, to make sure that the robot is not touching the deposited glyph
            move(85, 0, 0.1, 0.3, 0.7);
            sleep(200);
            if (VuMark == RelicRecoveryVuMark.RIGHT) move(0, -450, 0.1, 0.3, 2.0);
            else telemetry.addData("Autonomous", "Done");
            sleep(100);
        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }

}
