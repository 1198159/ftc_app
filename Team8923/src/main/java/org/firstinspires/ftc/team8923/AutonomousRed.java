package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 *  Runs an Autonomous program for the second position on the wall of the field that the
 *  red alliance drivers are stationed at.
 */
@Autonomous(name = "Auto Red 2", group = "Autonomous")
public class AutonomousRed extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        robotX = RED_2_START_X;
        robotY = RED_2_START_Y;
        robotAngle = RED_2_START_ANGLE;
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        waitForStart();

        vuforiaLocator.startTracking();

        // Go in front of left beacon and give Vuforia a chance to start tracking the target
        driveToPoint(1700, 3200, robotAngle);
        turnToAngle(90.0);
        sleep(1000);

        // Get colors of both sides of beacon
        int colorLeft = vuforiaLocator.getPixelColor(-60, 230, 30);
        int colorRight = vuforiaLocator.getPixelColor(60, 230, 30);

        // TODO: Should the code below go into it's own method for pressing the beacon?
        // Check which side is more blue to determine which side is which color. The red value
        // doesn't change as much as blue for some reason, so we compare the blue values
        if(Color.blue(colorRight) > Color.blue(colorLeft))
        {
            // Press left side if it's red
            telemetry.log().add("Left is red");
            // Go in front of left button
            driveToPoint(1385, 3550, 90.0);
            // Move forward to press button
            driveToPoint(1385, 3630, 90.0);
            sleep(500);
            // Back away from beacon
            driveToPoint(1450, 3200, 90.0);
        }
        else
        {
            // Press right side if it's red
            telemetry.log().add("Right is red");
            // Go in front of right button
            driveToPoint(1515, 3550, 90.0);
            // Move forward to press button
            driveToPoint(1515, 3625, 90.0);
            sleep(500);
            // Back away from beacon
            driveToPoint(1450, 3200, 90.0);
        }

        // TODO: Press other beacon button

        // TODO: Remove if no longer used. Only here in case we need it
//        // Go in front of left side of beacon
//        goToLocation(1385, 3550, 90.0);
//        sleep(500);
//        // Check color of left side
//        if(colorSensor.red() >= colorSensor.blue())
//        {
//            // Press left side if it's red
//            telemetry.log().add("Left is red");
//            goToLocation(1385, 3630, 90.0);
//            sleep(500);
//            goToLocation(1385, 3550, 90.0);
//            sleep(500);
//        }
//        else
//        {
//            // Press right side if it's blue
//            telemetry.log().add("Left is blue");
//            goToLocation(1515, 3550, 90.0);
//            sleep(500);
//            goToLocation(1515, 3625, 90.0);
//            sleep(500);
//            goToLocation(1515, 3550, 90.0);
//            sleep(500);
//        }

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(5000);
    }
}
