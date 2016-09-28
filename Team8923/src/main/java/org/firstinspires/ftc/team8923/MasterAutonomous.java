package org.firstinspires.ftc.team8923;

/*
 * This class contains all objects and methods that should be accessible by all Autonomous OpModes
 * for the CapBot
 */
abstract class MasterAutonomous extends Master
{
    // Drive power is less than 1 to allow encoder PID loop to function
    private static final double DRIVE_POWER = 0.8;

    // Information on robot's location. Units are millimeters and degrees
    private double robotX = 0.0, robotY = 0.0, robotAngle = 0.0;

    // TODO: Make an object for Vuforia

    void goToLocation(double targetX, double targetY, double targetAngle)
    {
        // TODO: Test me
        double angleToTarget = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));

        // Point at target location
        turnToAngle(angleToTarget);

        // Drive to target location
        driveToPoint(targetX, targetY);

        // Set robot angle to desired angle
        turnToAngle(targetAngle);
    }

    private void turnToAngle(double targetAngle)
    {
        double deltaAngle = targetAngle - robotAngle;

        // TODO: Test and fix magic number (5 is an angle tolerance)
        while(Math.abs(deltaAngle) > 5.0)
        {
            updateRobotAngle();
            deltaAngle = targetAngle - robotAngle;
            driveMecanum(0.0, 0.0, DRIVE_POWER * Math.signum(deltaAngle));
        }
        stopDriving();
    }

    private void driveToPoint(double targetX, double targetY)
    {
        double angleToTarget;

        // TODO: Test and fix magic number (10 is a distance tolerance)
        while(calculateDistance(targetX - robotX, targetY - robotY) > 10.0)
        {
            updateRobotLocation();
            angleToTarget = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));
            driveMecanum(angleToTarget, DRIVE_POWER, 0.0);
        }
        stopDriving();
    }

    private void updateRobotLocation()
    {
        // TODO: Should try Vuforia first, then other sensors
        robotX = 0.0;
        robotY = 0.0;
    }

    private void updateRobotAngle()
    {
        // TODO: Should try Vuforia first, then other sensors
        robotAngle = 0.0;
    }
}
