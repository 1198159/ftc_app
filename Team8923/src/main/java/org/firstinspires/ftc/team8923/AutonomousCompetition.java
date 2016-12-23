package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 *  Autonomous OpMode for both alliances. The OpMode is setup with a gamepad during initialization,
 *  so robot can start at one of two locations, and can complete any objective in any order
 */
@Autonomous(name = "Auto Competition", group = "Competition")
public class AutonomousCompetition extends MasterAutonomous
{
    private double beaconX;
    private double beaconY;
    private double beaconAngle;

    @Override
    public void runOpMode() throws InterruptedException
    {
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
                    pressBeacon(Objectives.BEACON_LEFT);
                    break;
                case BEACON_RIGHT:
                    pressBeacon(Objectives.BEACON_RIGHT);
                    break;
                case PARK_RAMP:
                    parkOnObjective(Objectives.PARK_RAMP);
                    break;
                case PARK_CENTER:
                    parkOnObjective(Objectives.PARK_CENTER);
                    break;
                case SHOOT_CENTER:
                    shootInCenter();
                    break;
            }
        }

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(10000);
    }

    // Drives the robot onto either the center platform or corner vortex ramp based on alliance
    // and chosen parking location
    private void parkOnObjective(Objectives objective) throws InterruptedException
    {
        double locationX;
        double locationY;

        switch(alliance)
        {
            case RED:
                switch(objective)
                {
                    case PARK_RAMP:
                        locationX = 600;
                        locationY = 3300;
                        break;
                    case PARK_CENTER:
                        locationX = 1500;
                        locationY = 2000;
                        break;
                    // Something is a bogus value, so run away
                    default: return;
                }
                break;
            case BLUE:
                switch(objective)
                {
                    case PARK_RAMP:
                        locationX = 3300;
                        locationY = 600;
                        break;
                    case PARK_CENTER:
                        locationX = 2000;
                        locationY = 1500;
                        break;
                    // Something is a bogus value, so run away
                    default: return;
                }
                break;
            // Something is a bogus value, so run away
            default: return;
        }

        turnAndDrive(locationX, locationY);
    }

    // Each alliance has its own method for the beacons, because they are hard to combine. This
    // method runs the corresponding method, and also sets the coordinates based on selected beacon
    private void pressBeacon(Objectives objective) throws InterruptedException
    {
        switch(alliance)
        {
            case RED:
                switch(objective)
                {
                    case BEACON_LEFT:
                        beaconX = VuforiaLocator.TargetLocations.RED_LEFT_X.val;
                        beaconY = VuforiaLocator.TargetLocations.RED_LEFT_Y.val;
                        break;
                    case BEACON_RIGHT:
                        beaconX = VuforiaLocator.TargetLocations.RED_RIGHT_X.val;
                        beaconY = VuforiaLocator.TargetLocations.RED_RIGHT_Y.val;
                        break;
                    // Something is a bogus value, so run away
                    default: return;
                }
                beaconAngle = 90.0;
                break;
            case BLUE:
                switch(objective)
                {
                    case BEACON_LEFT:
                        beaconX = VuforiaLocator.TargetLocations.BLUE_LEFT_X.val;
                        beaconY = VuforiaLocator.TargetLocations.BLUE_LEFT_Y.val;
                        break;
                    case BEACON_RIGHT:
                        beaconX = VuforiaLocator.TargetLocations.BLUE_RIGHT_X.val;
                        beaconY = VuforiaLocator.TargetLocations.BLUE_RIGHT_Y.val;
                        break;
                    // Something is a bogus value, so run away
                    default: return;
                }
                beaconAngle = 0.0;
                break;
            // Something is a bogus value, so run away
            default: return;
        }
        pressBeacon();
    }

    private void pressBeacon() throws InterruptedException
    {
        // Distance from which we look at the vision target and beacon in mm
        double observationDistance = 300;

        // Drive in front of the beacon, then face vision target
        switch(alliance)
        {
            case RED:
                turnAndDrive(beaconX, beaconY - observationDistance);
                turnToAngle(90.0);
                break;
            case BLUE:
                turnAndDrive(beaconX - observationDistance, beaconY);
                turnToAngle(0.0);
                break;
            // Something is a bogus value, so run away
            default: return;
        }

        // Give Vuforia a chance to start tracking the target
        sleep(500);

        // Only actually looks if vision target isn't visible
        lookForVisionTarget();

        // Reposition after tracking target
        driveRelativeToBeacon(0.0, observationDistance);

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

        // Buttons on beacon are 65mm from center
        double buttonDistance = 65;

        /*
         * Compare the hues of each side. The hue color wheel has red at 360 degrees, and blue at
         * 240 degrees. Subtracting the angles from each other results in some positive or negative
         * number. The sign can tell us which side is red and blue. In this case, the left hue is
         * subtracted from the right hue; a positive sign means left is red, negative mean right.
         */
        // Make button distance negative if we need to go to the other side
        if(colorLeft[0] - colorRight[0] > 0)
            buttonDistance *= -1;
        if(alliance == Alliance.BLUE)
            buttonDistance *= -1;

        // Extend pusher to press button
        servoBeaconPusher.setPosition(ServoPositions.BEACON_EXTEND.pos);
        // Drive slower for more accuracy
        slowModeDivisor = 2.0;

        // Line up with button
        driveRelativeToBeacon(buttonDistance, observationDistance);
        // Move forward to press button
        driveRelativeToBeacon(buttonDistance, 155);
        // Back away from beacon
        driveRelativeToBeacon(buttonDistance, observationDistance);

        // Retract pusher to prevent damage or anything else bad
        servoBeaconPusher.setPosition(ServoPositions.BEACON_RETRACT.pos);
        // Reset slow mode
        slowModeDivisor = 1.0;
    }

    // TODO: Test me
    // Shoots balls into the center vortex. Should only be used as the first objective, because
    // robot will be placed correctly at start. Robot's position won't be as accurate later on
    private void shootInCenter()
    {
        double goalX;
        double goalY;

        // Choose goal based on alliance color
        switch(alliance)
        {
            case RED:
                goalX = 5 * 12 * 25.4;
                goalY = 7 * 12 * 25.4;
                break;
            case BLUE:
                goalX = 7 * 12 * 25.4;
                goalY = 5 * 12 * 25.4;
                break;
            // Something is a bogus value, so run away
            default: return;
        }

        setFlywheelPowerAndAngle(calculateDistance(goalX - robotX, goalY - robotY));

        // Wait for servo to move and flywheel to speed up
        sleep(1500);

        // Shoot twice
        for(int i = 0; i < 2; i++)
        {
            servoFinger.setPosition(ServoPositions.FINGER_EXTEND.pos);
            sleep(500); // Give servo time to move
            servoFinger.setPosition(ServoPositions.FINGER_RETRACT.pos);
            sleep(500); // Give servo time to move
        }

        // Stop flywheel
        motorFlywheel.setPower(0.0);
    }

    // Drives robot with coordinates relative to beacon. Parameters are coordinates intrinsic to
    // beacon, which are then converted to extrinsic coordinates to which the robot drives
    private void driveRelativeToBeacon(double targetX, double targetY) throws InterruptedException
    {
        // Y input is always positive, but we need it to be negative for the math
        targetY *= -1;

        // Convert intrinsic values to extrinsic
        double relativeX = targetX * Math.sin(Math.toRadians(beaconAngle)) + targetY * Math.cos(Math.toRadians(beaconAngle));
        double relativeY = targetX * -Math.cos(Math.toRadians(beaconAngle)) + targetY * Math.sin(Math.toRadians(beaconAngle));

        driveToPoint(beaconX + relativeX, beaconY + relativeY, beaconAngle);
    }
}
