package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 *  Autonomous OpMode for both alliances. The OpMode is setup with a gamepad during initialization,
 *  so robot can start at one of two locations, and can complete any objective in any order
 */
@Autonomous(name = "Auto Competition", group = "Competition")
public class AutonomousCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setUpRoutine();
        initAuto();

        waitForStart();

        // Wait for requested number of milliseconds
        sleep(delayTime * 1000);

        vuforiaLocator.startTracking();
        // We only want to use Vuforia with the beacons for reliability concerns
        useVuforia = false;

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
                    shootInCenter(numberOfShots);
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
                        locationX = 1600;
                        locationY = 2100;
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
                        locationX = 2100;
                        locationY = 1600;
                        break;
                    // Something is a bogus value, so run away
                    default: return;
                }
                break;
            // Something is a bogus value, so run away
            default: return;
        }

        // Running into the cap ball with the lift can prematurely
        // deploy the arms, so we prevent this by running into
        // it with the other side of the robot
        if(objective == Objectives.PARK_CENTER)
        {
            reverseDrive(false);
            headingOffset += 180;
            lastEncoderBL = motorBL.getCurrentPosition();
            lastEncoderFL = motorFL.getCurrentPosition();
            lastEncoderBR = motorBR.getCurrentPosition();
            lastEncoderFR = motorFR.getCurrentPosition();
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
        double observationDistance = 500;
        useVuforia = true;

        // Drive in front of the beacon, then face vision target
        switch(alliance)
        {
            // For some reason, different movements work better for different sides
            case RED:
                //driveRelativeToBeacon(0.0, observationDistance);
                turnAndDrive(beaconX - 180, beaconY - observationDistance);
                turnToAngle(90.0);
                break;
            case BLUE:
                turnAndDrive(beaconX - observationDistance, beaconY + 180);
                turnToAngle(0.0);
                break;
            // Something is a bogus value, so run away
            default: return;
        }

        // Give Vuforia a chance to start tracking the target
        sleep(750);

        // Only actually looks if vision target isn't visible
        if(!lookForVisionTarget())
            // Vision target wasn't found, so abort
            return;

        // Extend pusher to position color sensors in front of beacon
        servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_EXTEND.pos);
        ElapsedTime servoTimer = new ElapsedTime();

        // Once we find the target, go right in front of the beacon to get the colors
        driveRelativeToBeacon(0.0, 250);

        // Give time for beacon pusher to fall down if no correction is needed. Otherwise don't wait
        sleep((long) Range.clip(500 - servoTimer.milliseconds(), 0, 500));

        sleep(500);
        /*
         * Here is where we compare the colors of each side of the beacon. The color sensors give
         * us information in an rgb format. We could just directly compare the red and blue values,
         * but we haven't had reliable results with that approach. Instead, we convert it to an hsv
         * format and use the hue for comparisons. From a hue color wheel, the blue value is at 240
         * degrees, and red is at 360/0. The color class converts the rgb values from the color
         * sensor (all of the values are stored in a single integer) to hsv as a float array with
         * the 0 index being the hue.
         */
        float leftHue = getHueFromSensor(colorSensorLeft);
        float rightHue = getHueFromSensor(colorSensorRight);

        telemetry.log().add("Left hue: " + leftHue);
        telemetry.log().add("Right hue: " + rightHue);

        // Figure out on which side the beacon pusher needs to be depending on beacon and alliance
        // colors. The robot assumes it's on the red alliance, then goes to the other side if not
        boolean leftSideIsRed = leftHue > rightHue;
        boolean leftSide = false;
        if(leftSideIsRed)
            leftSide = true;
        if(alliance == Alliance.BLUE)
            leftSide = !leftSide;
        // Move the servo to the desired side
        if(leftSide)
        {
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_LEFT.pos);
            telemetry.log().add("Choosing Left");
        }
        else
        {
            servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_RIGHT.pos);
            telemetry.log().add("Choosing Right");
        }

        // Wait for servo to move
        sleep(250);

        // Move forward to press button
        driveRelativeToBeacon(0.0, 110);
        useVuforia = false;
        // Back away from beacon
        driveRelativeToBeacon(0.0, observationDistance);

        // Retract and center pusher to prevent damage or anything else bad
        servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_CENTER.pos);
        servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_RETRACT.pos);
    }

    // Shoots balls into the center vortex
    private void shootInCenter(int numberOfShots) throws InterruptedException
    {
        // Distance from the goal at which the robot shoots
        double shootingDistance = 700;

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

        // Catapult shoots to the side of the robot
        turnToAngle(angleToGoal - 115);

        armCatapult();

        if(numberOfShots == 1)
        {
            fireCatapult();
        }
        else
        {
            // Drop collector so the hopper isn't blocked and run the collector backwards to help
            servoCollectorHolder.setPosition(ServoPositions.COLLECTOR_HOLDER_UP.pos);
            motorCollector.setPower(-0.5);
            // Fire the first particle
            armCatapult();
            // Push second particle into the catapult
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
            // Stop the collector
            motorCollector.setPower(0.0);

            // Wait for the second particle to settle
            sleep(750);
            // Put the sweeper servo back
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
            // Launch second particle
            fireCatapult();
        }
    }
}
