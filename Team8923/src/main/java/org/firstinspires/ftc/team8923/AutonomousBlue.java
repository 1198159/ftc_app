package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

/*
 *  Autonomous OpMode for blue alliance. The OpMode is setup with a gamepad during initialization,
 *  so robot can start at one of two locations, and can complete any objective in any order
 */
@Autonomous(name = "Auto Blue", group = "Autonomous")
public class AutonomousBlue extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        allaince = Allaince.BLUE;

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
                default:

            }
        }

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(10000);
    }

    private void parkOnRamp() throws InterruptedException
    {
        double angleToRamp = Math.atan2(3300 - robotY, 600 - robotX);
        turnToAngle(angleToRamp);
        driveToPoint(3300, 600, angleToRamp);
    }

    // TODO: Test me
    private void parkOnCenter() throws InterruptedException
    {
        double angleToCenter = Math.atan2(2000 - robotY, 1500 - robotX);
        turnToAngle(angleToCenter);
        driveToPoint(2000, 1500, angleToCenter);
    }

    private void pressLeftBeacon() throws InterruptedException
    {
        pressBeacon(3657.6, 2743.2);
    }

    private void pressRightBeacon() throws InterruptedException
    {
        pressBeacon(3657.6, 1524);
    }

    private void pressBeacon(double beaconX, double beaconY) throws InterruptedException
    {
        // TODO: Do we need sleep commands in here?
        double angleToEndOfTape = Math.atan2(beaconY - robotY, beaconX - robotX - 450);

        // Go to the end of the tape in front of the beacon
        turnToAngle(angleToEndOfTape);
        driveToPoint(beaconX - 450, beaconY, angleToEndOfTape);
        turnToAngle(0);

        // Give Vuforia a chance to start tracking the target
        sleep(1000);

        // Only actually looks if vision target isn't visible
        lookForVisionTarget();

        // Reposition after tracking target
        driveToPoint(beaconX - 450, beaconY, 0);

        // Get colors of both sides of beacon. Parameters are in mm from center of vision target
        int colorLeft = vuforiaLocator.getPixelColor(-60, 230, 30);
        int colorRight = vuforiaLocator.getPixelColor(60, 230, 30);

        // Check which side is more blue to determine which side is which color. The red value
        // doesn't change as much as blue for some reason, so we compare the blue values
        if(Color.blue(colorRight) > Color.blue(colorLeft))
        {
            // Press right side if it's blue
            telemetry.log().add("Right is blue");
            // Go in front of right button
            driveToPoint(beaconX - 100, beaconY + 10, 0.0);
            // Move forward to press button
            driveToPoint(beaconX - 25, beaconY + 10, 0.0);
            sleep(500);
        }
        else
        {
            // Press left side if it's blue
            telemetry.log().add("Left is blue");
            // Go in front of left button
            driveToPoint(beaconX - 100, beaconY + 140, 0);
            // Move forward to press button
            driveToPoint(beaconX - 25, beaconY + 140, 0);
            sleep(500);
        }

        // Back away from beacon
        driveToPoint(beaconX - 450, beaconY, 0);
    }
}
