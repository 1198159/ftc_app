package org.firstinspires.ftc.team8923;

/*
 * This class contains all objects and methods that should be accessible by all Autonomous OpModes
 * The axes of the field are defined where the origin is the corner between the driver stations,
 * poitive x is along the blue wall, positive y is the red wall, and 0 degrees is positive x
 */
abstract class MasterAutonomous extends Master
{
    // TODO: Might want to change naming style
    // TODO: Add locations of other starting spots
    // Starting locations for robot. Measurements are in millimeters and degrees
    static final double RED_2_START_X = 200.0;
    static final double RED_2_START_Y = 1880.0;
    static final double RED_2_START_ANGLE = 0.0;

    // Drive power is less than 1 to allow encoder PID loop to function
    private static final double DRIVE_POWER = 0.8;

    // Information on robot's location. Units are millimeters and degrees
    double robotX = 0.0, robotY = 0.0, robotAngle = 0.0;

    // TODO: Do these need to be set at the beginning?
    // Used to calculate distance traveled
    private int lastEncoderFL = 0;
    private int lastEncoderFR = 0;
    private int lastEncoderBL = 0;
    private int lastEncoderBR = 0;

    VuforiaLocator vuforiaLocator = new VuforiaLocator();

    void goToLocation(double targetX, double targetY, double targetAngle) throws InterruptedException
    {
        // TODO: Test me
        // TODO: Use turn method when it works
        double angleToTarget = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));

        // Point at target location
        //turnToAngle(angleToTarget);

        // Drive to target location
        driveToPoint(targetX, targetY, targetAngle);

        // Set robot angle to desired angle
        //turnToAngle(targetAngle);
    }

    private void turnToAngle(double targetAngle) throws InterruptedException
    {
        double deltaAngle = targetAngle - robotAngle;

        // TODO: Test and fix magic number (5 is an angle tolerance)
        while(Math.abs(deltaAngle) > 5.0)
        {
            updateRobotLocation();
            deltaAngle = targetAngle - robotAngle;
            driveMecanum(0.0, 0.0, DRIVE_POWER * Math.signum(deltaAngle));

            sendTelemetry();
            idle();
        }
        stopDriving();
    }

    private void driveToPoint(double targetX, double targetY, double targetAngle) throws InterruptedException
    {
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double DISTANCE_TOLERANCE = 20; // In mm TODO: Is 20 a good value?

        while(distanceToTarget > DISTANCE_TOLERANCE)
        {
            updateRobotLocation();
            double driveAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX)) - robotAngle;
            driveMecanum(driveAngle, DRIVE_POWER / 4, (targetAngle - robotAngle) / 100); // TODO: Make power slow down as it approaches target

            // TODO: Remove this when testing is done
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("RobotAngle", robotAngle);
            telemetry.addData("TargetDistance", distanceToTarget);
            telemetry.addData("DriveAngle", driveAngle);

            distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
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
            resetHeadingOffset();
        }
        // Otherwise, use other sensors to determine distance travelled and angle
        else
        {
            // TODO: Make sure this works

            int deltaFL = motorFL.getCurrentPosition() - lastEncoderFL;
            int deltaFR = motorFR.getCurrentPosition() - lastEncoderFR;
            int deltaBL = motorBL.getCurrentPosition() - lastEncoderBL;
            int deltaBR = motorBR.getCurrentPosition() - lastEncoderBR;

            lastEncoderFL = motorFL.getCurrentPosition();
            lastEncoderFR = motorFR.getCurrentPosition();
            lastEncoderBL = motorBL.getCurrentPosition();
            lastEncoderBR = motorBR.getCurrentPosition();

            // Take average of encoders. Some are negative because of 45 degree roller angle
            double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4;
            double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4;

            // Change ticks to mm, and compensate for 45 degree mounting angle of rollers with sqrt(2)
            deltaX *= MM_PER_TICK / Math.sqrt(2);
            deltaY *= MM_PER_TICK / Math.sqrt(2);

            // Delta x and y are intrinsic to robot, so make extrinsic
            robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
            robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));

            robotAngle = imu.getAngularOrientation().firstAngle - headingOffset;
        }
    }

    void resetHeadingOffset()
    {
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;
    }
}
