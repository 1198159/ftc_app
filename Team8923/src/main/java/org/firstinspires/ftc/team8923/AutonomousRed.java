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
        alliance = Alliance.RED;

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
        double angleToRamp = Math.toDegrees(Math.atan2(3300 - robotY, 600 - robotX));
        turnToAngle(angleToRamp);
        driveToPoint(600, 3300, angleToRamp);
    }

    // This also knocks off the cap ball if it's present
    private void parkOnCenter() throws InterruptedException
    {
        double angleToCenter = Math.toDegrees(Math.atan2(2000 - robotY, 1500 - robotX));
        turnToAngle(angleToCenter);
        driveToPoint(1500, 2000, angleToCenter);
    }

    private void pressLeftBeacon() throws InterruptedException
    {
        pressBeacon(VuforiaLocator.TargetLocations.RED_LEFT_X.val, VuforiaLocator.TargetLocations.RED_LEFT_Y.val);
    }

    private void pressRightBeacon() throws InterruptedException
    {
        pressBeacon(VuforiaLocator.TargetLocations.RED_RIGHT_X.val, VuforiaLocator.TargetLocations.RED_RIGHT_Y.val);
    }

    private void pressBeacon(double beaconX, double beaconY) throws InterruptedException
    {
        // Distance from which we look at the vision target and beacon in mm
        double observationDistance = 300;

        double angleToEndOfTape = Math.toDegrees(Math.atan2(beaconY - observationDistance - robotY, beaconX - robotX));

        // Go to the end of the tape in front of the beacon
        turnToAngle(angleToEndOfTape);
        driveToPoint(beaconX, beaconY - observationDistance, angleToEndOfTape);
        turnToAngle(90);

        // Give Vuforia a chance to start tracking the target
        sleep(500);

        // Only actually looks if vision target isn't visible
        lookForVisionTarget();

        // Reposition after tracking target
        driveToPoint(beaconX, beaconY - observationDistance, 90);

        // Get colors of both sides of beacon. Parameters are in mm from center of vision target
        // Sample location is lowest inside corner of beacon's colored regions
        // Float arrays are in HSV format, where 0 index is hue (which we care about)
        float[] colorLeft = new float[3];
        float[] colorRight = new float[3];
        Color.colorToHSV(vuforiaLocator.getPixelColor(-40, 170, 30), colorLeft);
        Color.colorToHSV(vuforiaLocator.getPixelColor(40, 170, 30), colorRight);

        // Red value will sometimes be near 0 rather than 360. If so, make it above 360
        // We never get any values near 90 degrees, so anything lower must be red
        if(colorLeft[0] < 90)
            colorLeft[0] += 360;
        if(colorRight[0] < 90)
            colorRight[0] += 360;

        telemetry.log().add("Left hue: " + colorLeft[0]);
        telemetry.log().add("Right hue: " + colorRight[0]);

        /*
         * Compare the hues of each side. The hue color wheel has red at 360 degrees, and blue at
         * 240 degrees. Subtracting the angles from each other results in some positive or negative
         * number. The sign can tell us which side is red and blue. In this case, the left hue is
         * subtracted from the right hue; a positive sign means left is red, negative mean right.
         */
        double buttonDistance;
        if(colorLeft[0] - colorRight[0] > 0)
        {
            telemetry.log().add("Left is red");
            // Left button is -65 mm from center of beacon
            buttonDistance = -65;
        }
        else
        {
            telemetry.log().add("Right is red");
            // Right button is 65 mm from center of beacon
            buttonDistance = 65;
        }

        // Line up with button
        driveToPoint(beaconX + buttonDistance, beaconY - observationDistance, 90, 0.3);
        // Move forward to press button
        driveToPoint(beaconX + buttonDistance, beaconY - 40, 90, 0.3);
        sleep(500); // TODO: Is this needed?
        // Back away from beacon
        driveToPoint(beaconX, beaconY - observationDistance, 90, 0.3);
    }
}
