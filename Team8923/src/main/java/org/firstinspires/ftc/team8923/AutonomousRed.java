package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 *  Autonomous OpMode for red alliance. The OpMode is setup with a gamepad during initialization,
 *  so robot can start at one of two locations, and can complete any objective in any order
 */
@Autonomous(name = "Auto Red", group = "Autonomous")
public class AutonomousRed extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // TODO: Add code to use gamepad to setup autonomous routine

        initHardware();

        robotX = RED_LEFT_START_X;
        robotY = RED_LEFT_START_Y;
        robotAngle = RED_LEFT_START_ANGLE;
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        waitForStart();

        vuforiaLocator.startTracking();

        pressLeftBeacon();
        pressRightBeacon();

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

    private void pressLeftBeacon() throws InterruptedException
    {
        pressBeacon(1524, 3657.6);
    }

    private void pressRightBeacon() throws InterruptedException
    {
        pressBeacon(2743.2, 3657.6);
    }

    private void pressBeacon(double beaconX, double beaconY) throws InterruptedException
    {
        double angleToEndOfTape = Math.atan2(beaconX - robotX, beaconY - robotY - 450);

        // Go to the end of the tape in front of the beacon
        driveToPoint(beaconX, beaconY - 450, angleToEndOfTape);
        turnToAngle(90);
        // Give Vuforia a chance to start tracking the target
        sleep(1000);

        // Get colors of both sides of beacon. Parameters are in mm from center of vision target
        int colorLeft = vuforiaLocator.getPixelColor(-60, 230, 30);
        int colorRight = vuforiaLocator.getPixelColor(60, 230, 30);

        // Check which side is more blue to determine which side is which color. The red value
        // doesn't change as much as blue for some reason, so we compare the blue values
        if(Color.blue(colorRight) > Color.blue(colorLeft))
        {
            // Press left side if it's red
            telemetry.log().add("Left is red");
            // Go in front of left button
            driveToPoint(beaconX - 140, beaconY - 100, 90);
            // Move forward to press button
            driveToPoint(beaconX - 140, beaconY - 25, 90);
            sleep(500);
        }
        else
        {
            // Press right side if it's red
            telemetry.log().add("Right is red");
            // Go in front of right button
            driveToPoint(beaconX - 10, beaconY - 100, 90.0);
            // Move forward to press button
            driveToPoint(beaconX - 10, beaconY - 25, 90.0);
            sleep(500);
        }

        // Back away from beacon
        driveToPoint(beaconX, beaconY - 450, 90);
    }
}
