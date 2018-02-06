package org.firstinspires.ftc.team8923_2017;

import android.annotation.TargetApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Autonomous Blue 1", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
@TargetApi(21)
public class AutonomousCompetitionBluePos1 extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia

        waitForStart();
        GetVumark();
        // turn on flash light if needed in bad lighting conditions
        //CameraDevice.getInstance().setFlashTorchMode(true);
        // set false to turn off light
        sleep(700);
        closeGG(); // Close GG (Glyph Grabber) lift claws to grasp glyph
        sleep(700);
        moveGG(1500); // Raise GG so glyph doesn't hit ramp when robot backs off ramp
        DropJJ(); // Lower JJ (Jewel Jostler) onto jewels
        sleep(500);
        stopGG(); // Stop GG lift movement
        GetLeftJewelColorCount(); // Get the color of the left jewel
        //CameraDevice.getInstance().setFlashTorchMode(false); // Turn off light if light was turned on
        double referenceAngle =  imu.getAngularOrientation().firstAngle; // Get a reference ange from the IMU for future movements using IMU
        if (isLeftJewelRed == true)
        {
            //If the left jewel is red, robot pivots left
            IMUPivot(referenceAngle, 15, 0.5, 0.015);
            RetrieveJJ(); // Raise JJ
            IMUPivot(referenceAngle, 0, 0.5, 0.015); // Pivot back to initial position
        }
        else
        {
            //If the left jewel isn't red, robot pivots right
            IMUPivot(referenceAngle, -15, 0.5, 0.015);
            RetrieveJJ(); // Raise JJ
            IMUPivot(referenceAngle, 0, 0.5, 0.015); // Pivot back to initial position
        }
        sleep(700);
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 2.3);//Go towards parking spot
        IMUPivot(referenceAngle, 90, 0.25, 0.015); // Pivot towards the cryptobox
        referenceAngle += 90.0; // Updates reference angle
        referenceAngle = adjustAngles(referenceAngle);
        sleep(500);
        moveGG(-750); // Lower GG lift
        sleep(700);
        stopGG(); // Stop GG lift movement
        sleep(300);
        alignOnLine55(0.4, 3.0, 0.3); // Align on the lines in front of the cryptobox
        // Robot is now aligned on the lines in front of the cryptobox (In front of the middle of crypobox)
        sleep(300);
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            // If VuMark is for left column, robot backs up then translates left
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.3);
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.65);
            telemetry.addData("Stage", "Left"); // Update telemetry
            telemetry.update();
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            // If VuMark is for center column. robot does nothing since it is already aligned
            telemetry.addData("Stage", "Center");
            telemetry.update();
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            // If VuMark is for right column, robot backs up then translates right
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.3);
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.71);
            telemetry.addData("Stage", "Right");
            telemetry.update();
        }
        else
        {
            // If the VuMark wasn't visible, the robot delivers in the center column
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.05);
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
        }
        sleep(500);
        // Robot moves forward and delivers glyph
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.8);
        // Robot backs up slightly so less pressure is applied on the field wall
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.1);
        sleep(700);
        moveGG(-750); // Lower GG lift
        sleep(700);
        stopGG(); // Stop GG lift movement
        sleep(500);
        openGG(); // Open GG claws
        sleep(500);
        // RObot backs up
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.55, 0.22);
        //sleep(500);
        // Robot moves forward to make sure the glyph is in the column
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 1.5);
        //sleep(500);
        // Robot backs up again
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.45, 0.5);
        // Based on which column the robot delivered in, robot does a series of translation to move back to center of cryptobox
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.43);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.12);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.05);
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.35);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.2);
        }
        else
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.05);
        }
        // Robot pivots to face the glyph pile
        IMUPivot(referenceAngle,  -178, 0.45, 0.015);
        stopDriving();
        while (opModeIsActive())
        {
            idle();
        }

    }
}
