package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/*
 * This class contains all objects and methods that should be accessible by all Autonomous OpModes
 * The axes of the field are defined where the origin is the corner between the driver stations,
 * positive x is along the blue wall, positive y is the red wall, and 0 degrees is positive x
 */
abstract class MasterAutonomous extends Master
{
    // These are not in Master, because they take longer to initialize, which slows down TeleOp
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;
    BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // We sometimes don't want to use Vuforia even if we see the vision targets
    boolean useVuforia = true;

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

        RED_LEFT_START_X(200),
        RED_LEFT_START_Y(2200),
        RED_LEFT_START_ANGLE(0.0),

        RED_RIGHT_START_X(200),
        RED_RIGHT_START_Y(1000),
        RED_RIGHT_START_ANGLE(0.0),

        BLUE_LEFT_START_X(1000),
        BLUE_LEFT_START_Y(200),
        BLUE_LEFT_START_ANGLE(90.0),

        BLUE_RIGHT_START_X(2200),
        BLUE_RIGHT_START_Y(200),
        BLUE_RIGHT_START_ANGLE(90.0);

        public final double val;
        StartLocations(double i)
        {
            val = i;
        }
    }

    // Constants for robot in autonomous
    // Max drive power is less than 1 to ensure speed controller works
    private static final double MAX_DRIVE_POWER = 1.0;
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double TURN_POWER_CONSTANT = 1.0 / 175.0;
    private static final double DRIVE_POWER_CONSTANT = 1.0 / 1750.0;

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
    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    VuforiaLocator vuforiaLocator = new VuforiaLocator();

    // Variables used for autonomous routine
    ArrayList<Objectives> routine = new ArrayList<>();
    private StartLocations startLocation = StartLocations.LEFT;
    Alliance alliance = Alliance.RED;
    int delayTime = 0; // In seconds
    int numberOfShots = 0; // Number of shots in the center vortex

    void setUpRoutine()
    {
        telemetry.log().add("Alliance Blue/Red: X/B");
        telemetry.log().add("Starting Position Left/Right: D-Pad Left/Right");
        telemetry.log().add("Delay Time Up/Down: D-Pad Up/Down");
        telemetry.log().add("Beacon Left/Right: Bumper Left/Right");
        telemetry.log().add("Park Ramp/Center: Y/A");
        telemetry.log().add("Shoot in center vortex: Center / Guide button");
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
            {
                if (numberOfShots == 0)
                    routine.add(Objectives.SHOOT_CENTER);
                if (numberOfShots < 2)
                    numberOfShots++;
            }

            // Clear objectives if a mistake was made
            else if(gamepad1.back)
            {
                routine.clear();
                numberOfShots = 0;
            }
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
            telemetry.addData("Number of shots in center", numberOfShots);
            telemetry.addData("", "");
            // Get the next objective in the routine, and add to telemetry
            // The + 1 is to shift from 0 index to 1 index for display
            if(routine != null)
                for(Objectives objective : routine)
                    telemetry.addData("Objective " + (routine.indexOf(objective) + 1), objective.name());
            telemetry.addData("", "");

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
        initHardware();

        reverseDrive(true);

        // Set last known encoder values
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
        double DISTANCE_TOLERANCE = 50; // In mm
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

    // We often need to first turn to where we're going, then go there. This makes our lives simpler
    void turnAndDrive(double x, double y) throws InterruptedException
    {
        double driveAngle = Math.toDegrees(Math.atan2(y - robotY, x - robotX));
        turnToAngle(driveAngle);
        driveToPoint(x, y, driveAngle);
    }

    // Used to
    double beaconX;
    double beaconY;
    double beaconAngle;

    // Drives robot with coordinates relative to beacon. Parameters are coordinates intrinsic to
    // beacon, which are then converted to extrinsic coordinates to which the robot drives
    void driveRelativeToBeacon(double targetX, double targetY) throws InterruptedException
    {
        // Beacon pusher and phone camera are offset from center of robot
        targetX -= 140;
        targetY += 160;

        // Y input is always positive, but we need it to be negative for the math
        targetY *= -1;

        // Convert intrinsic values to extrinsic
        double relativeX = targetX * Math.sin(Math.toRadians(beaconAngle)) + targetY * Math.cos(Math.toRadians(beaconAngle));
        double relativeY = targetX * -Math.cos(Math.toRadians(beaconAngle)) + targetY * Math.sin(Math.toRadians(beaconAngle));

        driveToPoint(beaconX + relativeX, beaconY + relativeY, beaconAngle);
    }

    // Robot sometimes won't see the vision targets when it should. This is to be used in places
    // where we need to be sure that we're tracking the target
    boolean lookForVisionTarget() throws InterruptedException
    {
        // We only use the robot's own alliance's vision targets, because the others give us bogus
        // numbers for some reason
        boolean trackingOtherAllianceTarget = false;
        if(alliance == Alliance.RED)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Blue Left")
                    || vuforiaLocator.getTargetName().equals("Target Blue Right");
        else if(alliance == Alliance.BLUE)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Red Left")
                    || vuforiaLocator.getTargetName().equals("Target Red Right");

        int count = 0;
        double referenceAngle = robotAngle;
        double deltaAngle = 5; // In degrees
        ElapsedTime timer = new ElapsedTime();
        int maxSearchTime = 5; // In seconds

        // Turn until target is found or timer runs out
        while(!(vuforiaLocator.isTracking() && !trackingOtherAllianceTarget) && opModeIsActive())
        {
            // Back up before searching for target to help utilize the full field of view of the phone camera
            if(count == 0)
                driveRelativeToBeacon(0, 650);
            //Failed to find vision target in time
            if(timer.seconds() > maxSearchTime)
                return false;

            // Increases search angle by delta angle each loop
            count++;
            double targetAngle = count * deltaAngle;

            // Look left then right every other loop
            if(count % 2 == 0)
                targetAngle *= -1;

            // Add onto original angle
            targetAngle += referenceAngle;

            // Turn to that angle
            turnToAngle(targetAngle);

            // Give Vuforia a chance to find the vision target
            sleep(500);

            // Update target names to ensure we don't look at the wrong ones
            if(alliance == Alliance.RED)
                trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Blue Left")
                        || vuforiaLocator.getTargetName().equals("Target Blue Right");
            else if(alliance == Alliance.BLUE)
                trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Red Left")
                        || vuforiaLocator.getTargetName().equals("Target Red Right");
        }
        // Vision target found
        return true;
    }

    // Updates robot's coordinates and angle
    void updateRobotLocation()
    {
        // We only use the robot's own alliance's vision targets, because the others give us bogus
        // numbers for some reason
        boolean trackingOtherAllianceTarget = false;
        if(alliance == Alliance.RED)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Blue Left")
                    || vuforiaLocator.getTargetName().equals("Target Blue Right");
        else if(alliance == Alliance.BLUE)
            trackingOtherAllianceTarget = vuforiaLocator.getTargetName().equals("Target Red Left")
                    || vuforiaLocator.getTargetName().equals("Target Red Right");

        // Use Vuforia if a it's tracking something
        if(vuforiaLocator.isTracking() && !trackingOtherAllianceTarget && useVuforia)
        {
            float[] location = vuforiaLocator.getRobotLocation();

            // Update coordinates and angle
            robotX = location[0];
            robotY = location[1];
            robotAngle = vuforiaLocator.getRobotAngle();
        }
        // Otherwise, use encoders and IMU to determine distance travelled and angle
        else
        {
            // Update robot angle
            robotAngle = imu.getAngularOrientation().firstAngle - headingOffset;

            // Calculate how far each motor has turned since last time
            int deltaFL = motorFL.getCurrentPosition() - lastEncoderFL;
            int deltaFR = motorFR.getCurrentPosition() - lastEncoderFR;
            int deltaBL = motorBL.getCurrentPosition() - lastEncoderBL;
            int deltaBR = motorBR.getCurrentPosition() - lastEncoderBR;

            // Take average of encoder ticks to find translational x and y components. FR and BL are
            // negative because of the direction at which they turn when going sideways
            double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4;
            double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4;

            // Convert to mm. X is divided by root 2 because the rollers turn when going sideways.
            // They do not turn when going forwards, so y doesn't need the division
            deltaX *= MM_PER_TICK;
            deltaY *= MM_PER_TICK;

            // TODO: Figure out why this is funky and fix it
            // This is a temporary hack. For some reason, the robot doesn't drive far enough
            // sideways when on the blue alliance only. Don't know why, but it works.
            if(alliance == Alliance.BLUE)
                deltaX /= 1.2;

            /*
             * Delta x and y are intrinsic to the robot, so they need to be converted to extrinsic.
             * Each intrinsic component has 2 extrinsic components, which are added to find the
             * total extrinsic components of displacement. The extrinsic displacement components
             * are then added to the previous position to set the new coordinates
             */
            robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
            robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));
        }

        // Set last encoder values for next loop
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();
    }

    // Move catapult forward to the armed state just before it fires
    void armCatapult()
    {
        telemetry.log().add("Arming Catapult: " + getRuntime());
        motorCatapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCatapult.setPower(1.0);
        catapultButtonLast = catapultButton.isPressed();

        // Wait until the catapult finishes moving
        while(!(!catapultButton.isPressed() && catapultButtonLast) && opModeIsActive())
        {
            catapultButtonLast = catapultButton.isPressed();
            sendTelemetry();
            idle();
        }

        motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCatapult.setTargetPosition(motorCatapult.getCurrentPosition());
        motorCatapult.setPower(1.0);
    }

    // Moves the catapult forward a bit from the armed position to launch a particle
    void fireCatapult()
    {
        telemetry.log().add("Firing Catapult: " + getRuntime());
        motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCatapult.setTargetPosition(motorCatapult.getCurrentPosition() + CATAPULT_TICKS_PER_CYCLE / 2);
        motorCatapult.setPower(1.0);
        // Wait until the catapult finishes moving
        while(!motorIsAtTarget(motorCatapult) && opModeIsActive())
        {
            sendTelemetry();
            idle();
        }
        // Turn off motor
        motorCatapult.setPower(0.0);
    }

    // The Adafruit color sensors give 2 byte RGB values, but Java's built in Color class expects
    // only 1, which causes overflow issues. Since we only really want the hue from the color
    // sensors, this method scales the sensor's rgb values to 1 byte and returns the hue
    float getHueFromSensor(ColorSensor sensor)
    {
        // We use doubles because we need to divide the numbers with floating point division. We
        // also don't care about the green value, and it causes some issues, so it's not used
        double red = sensor.red();
        double blue = sensor.blue();

        // Find the max value and use it to rescale each value to 0-255
        double max = Math.max(red, blue);
        red *= 255 / max;
        blue *= 255 / max;

        // Convert to a hue
        int argb = Color.argb(0, (int) red, 0, (int) blue);
        float[] hsv = new float[3];
        Color.colorToHSV(argb, hsv);

        // Red values can sometimes go near 0 rather than 360. We need it to be near 360 for our
        // comparisons, so if it's below 90 (greenish, which we never expect to get), add 360
        if(hsv[0] < 90)
            hsv[0] += 360;

        return hsv[0];
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
