package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

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
        autoInitializeRobot();

        // specify camera name for Vuforia
        // camera name was set in MasterOpMode, and is used in initVuforia() method
        VuforiaDetect.webcamName = webcamName;

        VuforiaDetect.initVuforia(); // initialize Vuforia
        telemetry.addData("Done: ", "initializing");
        telemetry.update();

        CameraDevice.getInstance().setFlashTorchMode(true); // turn on phone light

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
        CameraDevice.getInstance().setFlashTorchMode(false); // turn off phone light

        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

// Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoRuntime.reset(); // set the 30 second timer

// START OF AUTONOMOUS

        // grab the glyph
        motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        closeGG();
        sleep(100);
        raiseGM();
        sleep(100);

        Kmove = 1.0/1200.0;
        TOL = 100.0;
        TOL_ANGLE = 2;
        Kpivot = 1/200.0;

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
        sleep(100);

        VuforiaDetect.GetLeftJewelColor(); // calculate if the left jewel is blue or not

        // display the color values for each jewel color
        telemetry.addData("leftHue ", VuforiaDetect.avgLeftJewelColor);
        telemetry.addData("rightHue ", VuforiaDetect.avgRightJewelColor);
        telemetry.update();

        if(VuforiaDetect.isLeftJewelBlue) // if the left jewel is blue,
        {
            pivotWithReference(17, refAngle, 0.15,0.3); // then pivot right
            sleep(200);
            servoJewel.setPosition(JEWEL_INIT);
            sleep(200);
            pivotWithReference(0, refAngle, 0.15,0.5); // then pivot back
            sleep(200);
        }
        else // if the left jewel is red,
        {
            pivotWithReference(-17, refAngle, 0.15, 0.3); // then pivot left
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
                moveTimed(-0.5, 0, 1200); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the reference angle
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-125, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                sleep(100);
                Kpivot = 1/100.0;
                pivotWithReference(-140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                move(-120, 0, 0.15, 0.5, 1); // move right
                sleep(100);
                move(0, -225, 0.1, 0.3, 2.5); // push the glyph in
                sleep(100);
            }
            else if (VuMark == RelicRecoveryVuMark.RIGHT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1200); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the reference angle
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-310, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                sleep(100);
                Kpivot = 1/100.0;
                pivotWithReference(140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                move(-30, 0, 0.15, 0.5, 1); // move right
                sleep(100);
                move(0, -275, 0.1, 0.3, 2.5); // push the glyph in
                sleep(100);
            }
            else if (VuMark == RelicRecoveryVuMark.LEFT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1200); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-110, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                sleep(100);
                Kpivot = 1/100.0;
                pivotWithReference(-140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                move(-45, 0, 0.15, 0.5, 1); // move right
                sleep(100);
                move(0, -265, 0.1, 0.3, 2.5); // push the glyph in more, since we didn't shift right
                sleep(100);
            }
            else // If the VuMark is not visible we go for the center
            {
                telemetry.addData("Vumark", "not visible, going for center");

                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1200); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the reference angle
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-125, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                sleep(100);
                Kpivot = 1/100.0;
                pivotWithReference(-140, refAngle, 0.15, 0.55); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                move(-120, 0, 0.15, 0.5, 1); // move right
                sleep(100);
                move(0, -225, 0.1, 0.3, 2.5); // push the glyph in
                sleep(100);
            }

            // BACK UP FROM THE CRYPTOBOX
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            openGG(-400); // open the GG a little bit
            sleep(100);
            move(0, 160, 0.1, 0.3, 0.7); // back up from the cryptobox
            // release the glyph
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            openGG(minGGPos);
            pivotWithReference(-90, refAngle, 0.15, 0.55); // robot left side faces the deposited glyph
            sleep(100);
            move(90, 0, 0.3, 0.5, 2);
            sleep(100);
            if (VuMark != RelicRecoveryVuMark.RIGHT)
            {
                move(0, -145, 0.1, 0.6, 1.5);
            }
            else
            {
                move(0, 145, 0.1, 0.6, 1.5);
            }
            sleep(100);
            move(-170, 0, 0.4, 0.9, 2.3); // push the glyph into Cryptobox with side of robot
            sleep(100);
            move(70, 0, 0.3, 0.5, 1);
            sleep(100);
            lowerGM();
            if (VuMark == RelicRecoveryVuMark.LEFT) // if it's the left column, back up before deploying the GG extensions
            {
                move(0, 120, 0.4, 0.7, 1);
                sleep(100);
            }
            deployGGExtensions();
            pivotWithReference(0, refAngle, 0.15, 0.55); // robot GG faces the Glyph pile
        }
        else // RED RIGHT
        {
            if (VuMark == RelicRecoveryVuMark.CENTER)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-120, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                //move(-70, 0, 0.15, 0.5, 1); // move right
                move(0, -210, 0.1, 0.3, 2.5); // push the glyph in
                sleep(100);
            }
            else if (VuMark == RelicRecoveryVuMark.RIGHT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-80, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                move(-160, 0, 0.15, 0.5, 1); // move right
                move(0, -100, 0.1, 0.3, 2.5); // push the glyph in
                sleep(100);
            }
            else if (VuMark == RelicRecoveryVuMark.LEFT)
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-100, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                move(70, 0, 0.15, 0.5, 1); // move left
                move(0, -240, 0.1, 0.3, 2.5); // push the glyph in
                sleep(100);
            }
            else // if the VuMark is not visible, just go for the center column
            {
                // MOVE TOWARDS THE CRYPTOBOX
                moveTimed(-0.5, 0, 1100); // move left off the balancing stone
                sleep(100);
                pivotWithReference(0, refAngle, 0.15, 0.5); // fix the robot heading
                sleep(100);
                Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
                moveMaintainHeading(-120, 0, 0, refAngle, 0.15, 0.6, 1.6); // move left towards the cryptobox
                Kpivot = 1/50; // more aggressive pivot because we're turning a smaller angle
                pivotWithReference(-45, refAngle, 0.15, 0.5); // turn to face the cryptobox
                sleep(100);

                // ALIGN TO CORRECT COLUMN
                Kpivot = 1/100.0; // reset Kpivot
                //move(-70, 0, 0.15, 0.5, 1); // move right
                move(0, -210, 0.1, 0.3, 2.5); // push the glyph in
                sleep(100);
            }

            // BACK UP FROM THE CRYPTOBOX
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            openGG(-400); // open the GG a little bit
            sleep(100);
            move(0, 190, 0.1, 0.3, 0.7); // back up from the cryptobox
            sleep(100);
            // release the glyph
            motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            openGG(minGGPos);
            sleep(100);
            pivotWithReference(0, refAngle, 0.15, 0.55); // side of the robot is facing
            sleep(100);
            move(85, 0, 0.1, 0.3, 0.7);
            sleep(100);
            if (VuMark == RelicRecoveryVuMark.RIGHT)
            {
                move(0, -170, 0.1, 0.3, 2.0);
                sleep(100);
            }
            else if (VuMark == RelicRecoveryVuMark.CENTER || VuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                move(0, -150, 0.1, 0.3, 2.0);
                sleep(100);
            }
            else if (VuMark == RelicRecoveryVuMark.LEFT)
            {
                move(0, -150, 0.1, 0.3, 2.0);
                sleep(100);
            }
            // push the glyph in further
            move(-170, 0, 0.4, 0.9, 2.3);
            sleep(100);
            // move away from the scored glyph
            move(70, 0, 0.3, 0.5, 1);
            sleep(100);
            lowerGM();
            deployGGExtensions();
        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }

}
