package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

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
        allaince = Allaince.RED;

        initAuto();

        waitForStart();

        // Wait for requested number of milliseconds
        sleep(delayTime * 1000);

        vuforiaLocator.startTracking();

        // Completes each objective in the routine in order
        for(Objectives objective : routine)
        {
            // Get the next objective from the routine, and run it
            switch(objective)
            {
                // Only complete the requested objective
                case BEACON_LEFT:
                    pressLeftBeacon();
                    break;
                case BEACON_RIGHT:
                    pressRightBeacon();
                    break;
                case PARK_RAMP:
                    parkOnRamp();
                    break;
                case PARK_CENTER:
                    parkOnCenter();
                    break;
            }
        }

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(10000);
    }

    // TODO: Should we make these generic, or keep them alliance specific?
    // TODO: Should the numbers below use constants? At least some, like vision target locations?
    private void parkOnRamp() throws InterruptedException
    {
        double angleToRamp = Math.atan2(600 - robotY, 3300 - robotX);
        turnToAngle(angleToRamp);
        driveToPoint(600, 3300, angleToRamp);
    }

    // TODO: Test me
    // This also knocks off the cap ball if it's present
    private void parkOnCenter() throws InterruptedException
    {
        double angleToCenter = Math.atan2(1500 - robotY, 2000 - robotX);
        turnToAngle(angleToCenter);
        driveToPoint(1500, 2000, angleToCenter);
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
        double angleToEndOfTape = Math.atan2(beaconY - robotY - 450, beaconX - robotX);

        // Go to the end of the tape in front of the beacon
        turnToAngle(angleToEndOfTape);
        driveToPoint(beaconX, beaconY - 450, angleToEndOfTape);
        turnToAngle(90);

        // Give Vuforia a chance to start tracking the target
        sleep(1000);

        // Only actually looks if vision target isn't visible
        lookForVisionTarget();

        // Reposition after tracking target
        driveToPoint(beaconX, beaconY - 450, 90);

        // Get colors of both sides of beacon. Parameters are in mm from center of vision target
        int colorLeft = vuforiaLocator.getPixelColor(-60, 230, 30);
        int colorRight = vuforiaLocator.getPixelColor(60, 230, 30);

        // Check which side is more blue to determine the color of each side. The red value
        // doesn't change as much as blue for some reason, so we compare the blue values
        if(Color.blue(colorRight) > Color.blue(colorLeft))
        {
            // Press left side if it's red
            telemetry.log().add("Left is red");
            // Go in front of left button
            driveToPoint(beaconX - 140, beaconY - 100, 90);
            // Move forward to press button
            driveToPoint(beaconX - 140, beaconY - 35, 90);
            sleep(500);
        }
        else
        {
            // Press right side if it's red
            telemetry.log().add("Right is red");
            // Go in front of right button
            driveToPoint(beaconX - 10, beaconY - 100, 90.0);
            // Move forward to press button
            driveToPoint(beaconX - 10, beaconY - 35, 90.0);
            sleep(500);
        }

        // Back away from beacon
        driveToPoint(beaconX, beaconY - 450, 90);
    }
}
