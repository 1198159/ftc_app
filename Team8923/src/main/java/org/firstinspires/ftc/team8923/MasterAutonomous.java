package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/*
 * This class contains all objects and methods that should be accessible by all Autonomous OpModes
 * The axes of the field are defined where the origin is the corner between the driver stations,
 * positive x is along the blue wall, positive y is the red wall, and 0 degrees is positive x
 */
abstract class MasterAutonomous extends Master
{
    enum Alliance
    {
        BLUE,
        RED
    }

    private enum StartLocations
    {
        // Values of these 2 are arbitrary. Intended to be generic to allow one method for both
        // alliances. Other values should be used for actually setting the coordinates
        LEFT(0),
        RIGHT(0),

        RED_LEFT_START_X(17 * 25.4),
        RED_LEFT_START_Y(100 * 25.4),
        RED_LEFT_START_ANGLE(45.0),

        RED_RIGHT_START_X(17 * 25.4),
        RED_RIGHT_START_Y(52 * 25.4),
        RED_RIGHT_START_ANGLE(45.0),

        BLUE_LEFT_START_X(52 * 25.4),
        BLUE_LEFT_START_Y(17 * 25.4),
        BLUE_LEFT_START_ANGLE(45.0),

        BLUE_RIGHT_START_X(100 * 25.4),
        BLUE_RIGHT_START_Y(17 * 25.4),
        BLUE_RIGHT_START_ANGLE(45.0);

        public final double val;
        StartLocations(double i)
        {
            val = i;
        }
    }

    // Constants for robot in autonomous
    // Max drive power is less than 1 to ensure speed controller works
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double MIN_DRIVE_POWER = 0.2;
    private static final double TURN_POWER_CONSTANT = 1.0 / 150.0;
    private static final double DRIVE_POWER_CONSTANT = 1.0 / 1000.0;

    enum Objectives
    {
        BEACON_LEFT,
        BEACON_RIGHT,
        PARK_RAMP,
        PARK_CENTER,
        SHOOT_CENTER
    }

    // Information on robot's location. Units are millimeters and degrees
    double robotX = 0.0, robotY = 0.0, robotAngle = 0.0;

    // Used to calculate distance traveled between loops
    private int lastEncoderFL = 0;
    private int lastEncoderFR = 0;
    private int lastEncoderBL = 0;
    private int lastEncoderBR = 0;

    VuforiaLocator vuforiaLocator = new VuforiaLocator();

    // Variables used for autonomous routine
    ArrayList<Objectives> routine = new ArrayList<>();
    private StartLocations startLocation = StartLocations.LEFT;
    Alliance alliance = Alliance.RED;
    int delayTime = 0; // In seconds

    private void setUpRoutine()
    {
        telemetry.log().add("");
        telemetry.log().add("Alliance Blue/Red: X/B");
        telemetry.log().add("Starting Position Left/Right: D-Pad Left/Right");
        telemetry.log().add("Delay Time Up/Down: D-Pad Up/Down");
        telemetry.log().add("Beacon Left/Right: Bumper Left/Right");
        telemetry.log().add("Park Ramp/Center: Y/A");
        telemetry.log().add("Reset Routine: Back");
        telemetry.log().add("");
        telemetry.log().add("After routine is complete and robot is on field, press Start");

        boolean settingUp = true;

        while(settingUp)
        {
            // Select alliance
            if(gamepad1.x)
                alliance = Alliance.BLUE;
            else if(gamepad1.b)
                alliance = Alliance.RED;

            // Select start location
            else if(gamepad1.dpad_left)
                startLocation = StartLocations.LEFT;
            else if(gamepad1.dpad_right)
                startLocation = StartLocations.RIGHT;

            // Change delay time
            else if(gamepad1.dpad_up)
                delayTime += 1;
            else if(gamepad1.dpad_down)
                delayTime -= 1;

            // Select objectives to complete
            else if(gamepad1.left_bumper)
                routine.add(Objectives.BEACON_LEFT);
            else if(gamepad1.right_bumper)
                routine.add(Objectives.BEACON_RIGHT);
            else if(gamepad1.y)
                routine.add(Objectives.PARK_RAMP);
            else if(gamepad1.a)
                routine.add(Objectives.PARK_CENTER);
            else if(gamepad1.guide)
                routine.add(Objectives.SHOOT_CENTER);

            // Clear objectives if a mistake was made
            else if(gamepad1.back)
                routine.clear();

            // Finish setup and initialization. Should only be run when robot has been placed
            // in starting location, because the encoders and IMU need to initialize there
            else if(gamepad1.start)
                settingUp = false;

            while(!buttonsAreReleased(gamepad1))
            {
                // Wait for all buttons to be release before continuing. Otherwise buttons are read
                // continuously in setup, and add many objectives
                idle();
            }

            // Ensure delay isn't negative
            if(delayTime < 0)
                delayTime = 0;

            // Display current routine
            telemetry.addData("Alliance", alliance.name());
            telemetry.addData("Start Location", startLocation.name());
            telemetry.addData("Delay Seconds", delayTime);
            // Get the next objective in the routine, and add to telemetry
            // The + 1 is to shift from 0 index to 1 index for display
            if(routine != null)
                for(Objectives objective : routine)
                    telemetry.addData("Objective " + (routine.indexOf(objective) + 1), objective.name());

            telemetry.update();
            idle();
        }

        // We could clear the telemetry at this point, but the drivers may want to see it
        
        telemetry.log().add("Setup complete. Initializing...");

        // Set coordinates based on alliance and starting location
        if(startLocation == StartLocations.LEFT)
        {
            if(alliance == Alliance.RED)
            {
                robotX = StartLocations.RED_LEFT_START_X.val;
                robotY = StartLocations.RED_LEFT_START_Y.val;
                robotAngle = StartLocations.RED_LEFT_START_ANGLE.val;
            }
            else if(alliance == Alliance.BLUE)
            {
                robotX = StartLocations.BLUE_LEFT_START_X.val;
                robotY = StartLocations.BLUE_LEFT_START_Y.val;
                robotAngle = StartLocations.BLUE_LEFT_START_ANGLE.val;
            }
        }
        else if(startLocation == StartLocations.RIGHT)
        {
            if(alliance == Alliance.RED)
            {
                robotX = StartLocations.RED_RIGHT_START_X.val;
                robotY = StartLocations.RED_RIGHT_START_Y.val;
                robotAngle = StartLocations.RED_RIGHT_START_ANGLE.val;
            }
            else if(alliance == Alliance.BLUE)
            {
                robotX = StartLocations.BLUE_RIGHT_START_X.val;
                robotY = StartLocations.BLUE_RIGHT_START_Y.val;
                robotAngle = StartLocations.BLUE_RIGHT_START_ANGLE.val;
            }
        }
    }

    void initAuto()
    {
        setUpRoutine();
        initHardware();

        // Set last known encoder values
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        // Set IMU heading offset
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        telemetry.log().add("Initialized. Ready to start!");
    }

    // Turns to the specified angle
    void turnToAngle(double targetAngle) throws InterruptedException
    {
        updateRobotLocation();

        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double ANGLE_TOLERANCE = 5.0; // In degrees

        while(Math.abs(deltaAngle) > ANGLE_TOLERANCE && opModeIsActive())
        {
            updateRobotLocation();

            // Recalculate how far away we are
            deltaAngle = subtractAngles(targetAngle, robotAngle);

            // Slow down as we approach target
            double turnPower = Range.clip(deltaAngle * TURN_POWER_CONSTANT, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

            // Make sure turn power doesn't go below minimum power
            if(turnPower > 0 && turnPower < MIN_DRIVE_POWER)
                turnPower = MIN_DRIVE_POWER;
            else if(turnPower < 0 && turnPower > -MIN_DRIVE_POWER)
                turnPower = -MIN_DRIVE_POWER;

            // Set drive motor power
            driveMecanum(0.0, 0.0, turnPower);

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("RobotAngle", robotAngle);
            sendTelemetry();
            idle();
        }
        stopDriving();
    }

    // Makes robot drive to a point on the field
    void driveToPoint(double targetX, double targetY, double targetAngle) throws InterruptedException
    {
        driveToPoint(targetX, targetY, targetAngle, MAX_DRIVE_POWER);
    }

    // Makes robot drive to a point on the field
    void driveToPoint(double targetX, double targetY, double targetAngle, double maxPower) throws InterruptedException
    {
        updateRobotLocation();

        // Calculate how far we are from target point
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double DISTANCE_TOLERANCE = 45; // In mm
        double ANGLE_TOLERANCE = 5; // In degrees

        // Run until robot is within tolerable distance and angle
        while(!(distanceToTarget < DISTANCE_TOLERANCE && deltaAngle < ANGLE_TOLERANCE) && opModeIsActive())
        {
            updateRobotLocation();

            // In case robot drifts to the side
            double driveAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX)) - robotAngle;

            // Decrease power as robot approaches target. Ensure it doesn't exceed power limits
            double drivePower = Range.clip(distanceToTarget * DRIVE_POWER_CONSTANT, MIN_DRIVE_POWER, maxPower);

            // In case the robot turns while driving
            deltaAngle = subtractAngles(targetAngle, robotAngle);
            double turnPower = deltaAngle * TURN_POWER_CONSTANT;

            // Set drive motor powers
            driveMecanum(driveAngle, drivePower, turnPower);

            // Recalculate distance for next check
            distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);

            // Inform drivers of robot location
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("RobotAngle", robotAngle);
            sendTelemetry();
            idle();
        }
        stopDriving();
    }

    void turnAndDrive(double x, double y) throws InterruptedException
    {
        double driveAngle = Math.toDegrees(Math.atan2(y - robotY, x - robotX));
        turnToAngle(driveAngle);
        driveToPoint(x, y, driveAngle);
    }

    // Robot sometimes won't see the vision targets when it should. This is to be used in places
    // where we need to be sure that we're tracking the target
    void lookForVisionTarget() throws InterruptedException
    {
        boolean trackingOtherAllianceTarget = false;

        if(alliance == Alliance.RED)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Blue Left")
                    || vuforiaLocator.getTargetName().equals("Target Blue Right");
        else if(alliance == Alliance.BLUE)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Red Left")
                    || vuforiaLocator.getTargetName().equals("Target Red Right");

        //TODO: This won't always find the target, so make better
        // Turn until target is found
        while(!(vuforiaLocator.isTracking() && !trackingOtherAllianceTarget) && opModeIsActive())
        {
            turnToAngle(robotAngle - 10);
            sleep(500);
        }
    }

    // Updates robot's coordinates and angle
    void updateRobotLocation()
    {
        boolean trackingOtherAllianceTarget = false;

        if(alliance == Alliance.RED)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Blue Left")
                    || vuforiaLocator.getTargetName().equals("Target Blue Right");
        else if(alliance == Alliance.BLUE)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Red Left")
                    || vuforiaLocator.getTargetName().equals("Target Red Right");

        // Use Vuforia if a it's tracking something. We only use the robot's own alliance's
        // vision targets, because the others give us bogus numbers for some reason
        if(vuforiaLocator.isTracking() && !trackingOtherAllianceTarget)
        {
            float[] location = vuforiaLocator.getRobotLocation();
            robotX = location[0];
            robotY = location[1];

            robotAngle = vuforiaLocator.getRobotAngle();
        }
        // Otherwise, use encoders and IMU to determine distance travelled and angle
        else
        {
            robotAngle = imu.getAngularOrientation().firstAngle - headingOffset;

            int deltaFL = motorFL.getCurrentPosition() - lastEncoderFL;
            int deltaFR = motorFR.getCurrentPosition() - lastEncoderFR;
            int deltaBL = motorBL.getCurrentPosition() - lastEncoderBL;
            int deltaBR = motorBR.getCurrentPosition() - lastEncoderBR;

            // Take average of encoders ticks, and convert to mm. Some are negative because of 45
            // degree roller angle X value is divided by root 2, because the rollers spin when
            // going sideways. The rollers are stagnant when going forwards, so root 2 doesn't apply
            double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4 * MM_PER_TICK / Math.sqrt(2);
            double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4 * MM_PER_TICK;

            // Delta x and y are intrinsic to robot, so make extrinsic and update robot location
            robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
            robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));
        }

        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();
    }

    // If you subtract 359 degrees from 0, you would get -359 instead of 1. This method handles
    // cases when one angle is multiple rotations away from the other
    double subtractAngles(double first, double second)
    {
        double delta = first - second;
        while(delta > 180)
            delta -= 360;
        while(delta <= -180)
            delta += 360;
        return delta;
    }
}
