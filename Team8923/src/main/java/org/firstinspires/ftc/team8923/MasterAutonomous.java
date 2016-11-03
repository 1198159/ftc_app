package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.util.Range;

/*
 * This class contains all objects and methods that should be accessible by all Autonomous OpModes
 * The axes of the field are defined where the origin is the corner between the driver stations,
 * positive x is along the blue wall, positive y is the red wall, and 0 degrees is positive x
 */
abstract class MasterAutonomous extends Master
{
    // TODO: Check values of starting locations
    // Starting locations for robot. Measurements are in millimeters and degrees
    static final double RED_LEFT_START_X = 17 * 25.4;
    static final double RED_LEFT_START_Y = 100 * 25.4;
    static final double RED_LEFT_START_ANGLE = 45.0;

    static final double RED_RIGHT_START_X = 17 * 25.4;
    static final double RED_RIGHT_START_Y = 52 * 25.4;
    static final double RED_RIGHT_START_ANGLE = 45.0;

    static final double BLUE_LEFT_START_X = 52 * 25.4;
    static final double BLUE_LEFT_START_Y = 17 * 25.4;
    static final double BLUE_LEFT_START_ANGLE = 45.0;

    static final double BLUE_RIGHT_START_X = 100 * 25.4;
    static final double BLUE_RIGHT_START_Y = 17 * 25.4;
    static final double BLUE_RIGHT_START_ANGLE = 45.0;

    // Constants for robot in autonomous
    // Max drive power is less than 1 to ensure speed controller works
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double MIN_DRIVE_POWER = 0.1;
    private static final double TURN_POWER_CONSTANT = 1.0 / 150.0;
    private static final double DRIVE_POWER_CONSTANT = 1.0 / 1000.0;

    // Information on robot's location. Units are millimeters and degrees
    double robotX = 0.0, robotY = 0.0, robotAngle = 0.0;

    // TODO: Should location code go into Master? We may use it for TeleOp, and it would be more convenient to set these in hardware init
    // Used to calculate distance traveled between loops
    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    VuforiaLocator vuforiaLocator = new VuforiaLocator();

    // Turns to the specified angle
    void turnToAngle(double targetAngle) throws InterruptedException
    {
        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double ANGLE_TOLERANCE = 2.0; // In degrees

        while(Math.abs(deltaAngle) > ANGLE_TOLERANCE)
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

            telemetry.addData("RobotAngle", robotAngle);
            sendTelemetry();
            idle();
        }
        stopDriving();
    }

    // Makes robot drive to a point on the field
    void driveToPoint(double targetX, double targetY, double targetAngle) throws InterruptedException
    {
        // Calculate how far we are from target point
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double DISTANCE_TOLERANCE = 10; // In mm

        while(distanceToTarget > DISTANCE_TOLERANCE)
        {
            updateRobotLocation();

            // In case robot drifts to the side
            double driveAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX)) - robotAngle;

            // Decrease power as robot approaches target. Ensure it doesn't exceed power limits
            double drivePower = Range.clip(distanceToTarget * DRIVE_POWER_CONSTANT, MIN_DRIVE_POWER, MAX_DRIVE_POWER);

            // In case the robot turns while driving
            double turnPower = subtractAngles(targetAngle, robotAngle) * TURN_POWER_CONSTANT;

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

    // Updates robot's coordinates and angle
    void updateRobotLocation()
    {
        // Use Vuforia if a it's tracking something
        if(vuforiaLocator.isTracking())
        {
            float[] location = vuforiaLocator.getRobotLocation();
            robotX = location[0];
            robotY = location[1];

            robotAngle = vuforiaLocator.getRobotAngle();
        }
        // Otherwise, use other sensors to determine distance travelled and angle
        else
        {
            int deltaFL = motorFL.getCurrentPosition() - lastEncoderFL;
            int deltaFR = motorFR.getCurrentPosition() - lastEncoderFR;
            int deltaBL = motorBL.getCurrentPosition() - lastEncoderBL;
            int deltaBR = motorBR.getCurrentPosition() - lastEncoderBR;

            // Take average of encoders ticks, and convert to mm. Some are negative because of 45 degree roller angle
            double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4 * MM_PER_TICK;
            double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4 * MM_PER_TICK;

            // Delta x and y are intrinsic to robot, so make extrinsic and update robot location
            robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
            robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));

            robotAngle = imu.getAngularOrientation().firstAngle - headingOffset;
        }

        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();
    }

    // If you subtract 359 degrees from 0, you would get -359 instead of 1. This method handles
    // cases when one angle is multiple rotations away from the other
    private double subtractAngles(double first, double second)
    {
        double delta = first - second;
        while(delta > 180)
            delta -= 360;
        while(delta < -180)
            delta += 360;
        return delta;
    }
}
