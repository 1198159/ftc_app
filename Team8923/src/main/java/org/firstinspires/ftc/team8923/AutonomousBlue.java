package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        alliance = Alliance.BLUE;

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

    // TODO: Should the numbers below use constants? At least some, like vision target locations?
    private void parkOnRamp() throws InterruptedException
    {
        double angleToRamp = Math.toDegrees(Math.atan2(600 - robotY, 3300 - robotX));
        turnToAngle(angleToRamp);
        driveToPoint(3300, 600, angleToRamp);
    }

    // This also knocks off the cap ball if it's present
    private void parkOnCenter() throws InterruptedException
    {
        double angleToCenter = Math.toDegrees(Math.atan2(1500 - robotY, 2000 - robotX));
        turnToAngle(angleToCenter);
        driveToPoint(2000, 1500, angleToCenter);
    }

    private void pressLeftBeacon() throws InterruptedException
    {
        pressBeacon(VuforiaLocator.TargetLocations.BLUE_LEFT_X.val, VuforiaLocator.TargetLocations.BLUE_LEFT_Y.val);
    }

    private void pressRightBeacon() throws InterruptedException
    {
        pressBeacon(VuforiaLocator.TargetLocations.BLUE_RIGHT_X.val, VuforiaLocator.TargetLocations.BLUE_RIGHT_Y.val);
    }

    private void pressBeacon(double beaconX, double beaconY) throws InterruptedException
    {
        double angleToEndOfTape = Math.toDegrees(Math.atan2(beaconY - robotY, beaconX - robotX - 700));

        // Go to the end of the tape in front of the beacon
        turnToAngle(angleToEndOfTape);
        driveToPoint(beaconX - 700, beaconY, angleToEndOfTape);
        turnToAngle(0);

        // Give Vuforia a chance to start tracking the target
        sleep(500);

        // Only actually looks if vision target isn't visible
        lookForVisionTarget();

        // Reposition after tracking target
        driveToPoint(beaconX - 700, beaconY, 0);

        // Get colors of both sides of beacon. Parameters are in mm from center of vision target
        // Sample location is lowest inside corner of beacon's colored regions
        // Float arrays are in HSV format, where 0 index is hue (which we care about)
        float[] colorLeft = new float[3];
        float[] colorRight = new float[3];
        Color.colorToHSV(vuforiaLocator.getPixelColor(-40, 170, 30), colorLeft);
        Color.colorToHSV(vuforiaLocator.getPixelColor(40, 170, 30), colorRight);

        /*
         * Compare the hues of each side. The hue color wheel has red at 360 degrees, and blue at
         * 240 degrees. Subtracting the angles from each other results in some positive or negative
         * number. The sign can tell us which side is red and blue. In this case, the left hue is
         * subtracted from the right hue; a positive sign means left is red, negative mean right.
         */
        if(subtractAngles(colorLeft[0], colorRight[0]) > 0)
        {
            // Press right side if it's blue
            telemetry.log().add("Right is blue");
            // Go in front of right button
            driveToPoint(beaconX - 700, beaconY - 65, 0, 0.3);
            // Move forward to press button
            driveToPoint(beaconX - 40, beaconY - 65, 0, 0.3);
            sleep(500);
        }
        else
        {
            // Press left side if it's blue
            telemetry.log().add("Left is blue");
            // Go in front of left button
            driveToPoint(beaconX - 700, beaconY + 65, 0, 0.3);
            // Move forward to press button
            driveToPoint(beaconX - 40, beaconY + 65, 0, 0.3);
            sleep(500);
        }

        // Back away from beacon
        driveToPoint(beaconX - 700, beaconY, 0, 0.3);
    }
}
