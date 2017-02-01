package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        setUpRoutine();
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
                        locationX = 1750;
                        locationY = 2400;
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
        double observationDistance = 400;

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
        if(!lookForVisionTarget())
            // Vision target wasn't found, so abort
            return;

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

        // Line up with button
        driveRelativeToBeacon(buttonDistance, observationDistance);
        // Move in front of button to double check color
        driveRelativeToBeacon(buttonDistance, 200);

        // If color isn't correct, abort
        if(!correctColor())
        {
            // Retract pusher to prevent damage or anything else bad
            servoBeaconPusher.setPosition(ServoPositions.BEACON_RETRACT.pos);
            // Back away from beacon
            driveRelativeToBeacon(buttonDistance, observationDistance);
            return;
        }

        // Move forward to press button
        driveRelativeToBeacon(buttonDistance, 100);
        // Back away from beacon
        driveRelativeToBeacon(buttonDistance, observationDistance);

        // Retract pusher to prevent damage or anything else bad
        servoBeaconPusher.setPosition(ServoPositions.BEACON_RETRACT.pos);
    }

    // TODO: Test me
    // Shoots balls into the center vortex
    private void shootInCenter() throws InterruptedException
    {
        // Distance from the goal at which the robot shoots
        double shootingDistance = 1000;

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

        /*
         * A circle exists around the center goal from which the robot can shoot. The fastest way
         * for the robot to get on that circle is to follow the radial line from the goal to the
         * robot, then drive directly to/away from the goal. This point on the field is calculated,
         * then the robot drives to that point.
         */
        // Find angle from robot to goal
        double angleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        // Find closest point to the robot on shooting circle
        double shootPosX = goalX - Math.cos(Math.toRadians(angleToGoal)) * shootingDistance;
        double shootPosY = goalY - Math.sin(Math.toRadians(angleToGoal)) * shootingDistance;

        // Go to shooting location
        turnAndDrive(shootPosX, shootPosY);

        // Catapult shoots 90 degrees from front of robot
        turnToAngle(angleToGoal - 90);

        // Drop collector so the hopper isn't blocked
        servoCollectorHolder.setPosition(ServoPositions.COLLECTOR_HOLDER_UP.pos);

        // TODO: The code below here doesn't seem to work right; it seems to skip the loops. Fix me
        // Zero the catapult
        telemetry.log().add("Zeroing");
        motorCatapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCatapult.setPower(1.0);
        while(!catapultButton.isPressed())
        {
            sendTelemetry();
            idle();
        }
        catapultZero = motorCatapult.getCurrentPosition();
        motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Launch catapult and arm again
        telemetry.log().add("Launching first");
        motorCatapult.setTargetPosition(catapultZero + 1680 * 3);
        while(Math.abs(motorCatapult.getCurrentPosition() - motorCatapult.getTargetPosition()) < 20)
        {
            sendTelemetry();
            idle();
        }

        // Load next particle
        telemetry.log().add("Moving servo");
        servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
        sleep(500);
        servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_FIRST.pos);
        sleep(500);

        // Launch catapult
        telemetry.log().add("Launching second");
        motorCatapult.setTargetPosition(catapultZero + 1680 * 3 + 3000);
        while(Math.abs(motorCatapult.getCurrentPosition() - motorCatapult.getTargetPosition()) < 20)
        {
            sendTelemetry();
            idle();
        }

        motorCatapult.setPower(0.0);
    }

    // Drives robot with coordinates relative to beacon. Parameters are coordinates intrinsic to
    // beacon, which are then converted to extrinsic coordinates to which the robot drives
    private void driveRelativeToBeacon(double targetX, double targetY) throws InterruptedException
    {
        // Beacon pusher and phone camera are offset from center of robot
        targetX += 40;
        targetY += 160;

        // Y input is always positive, but we need it to be negative for the math
        targetY *= -1;

        // Convert intrinsic values to extrinsic
        double relativeX = targetX * Math.sin(Math.toRadians(beaconAngle)) + targetY * Math.cos(Math.toRadians(beaconAngle));
        double relativeY = targetX * -Math.cos(Math.toRadians(beaconAngle)) + targetY * Math.sin(Math.toRadians(beaconAngle));

        driveToPoint(beaconX + relativeX, beaconY + relativeY, beaconAngle);
    }
}
