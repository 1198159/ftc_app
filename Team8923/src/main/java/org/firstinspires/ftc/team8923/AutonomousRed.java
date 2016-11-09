package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

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
        telemetry.log().add("Starting Position: Press x for left, b for right");
        telemetry.log().add("Press start button when robot is on field");
        telemetry.update();

        // Set defaults in case they're not set below
        robotX = RED_LEFT_START_X;
        robotY = RED_LEFT_START_Y;
        robotAngle = RED_LEFT_START_ANGLE;

        // TODO: Add code to use gamepad to setup autonomous routine
        // Used to setup autonomous routine
        while(true)
        {
            if(gamepad1.x)
            {
                // Robot will start on left
                robotX = RED_LEFT_START_X;
                robotY = RED_LEFT_START_Y;
                robotAngle = RED_LEFT_START_ANGLE;
                telemetry.log().add("Left Selected");
            }
            else if(gamepad1.b)
            {
                // Robot will start on right
                robotX = RED_RIGHT_START_X;
                robotY = RED_RIGHT_START_Y;
                robotAngle = RED_RIGHT_START_ANGLE;
                telemetry.log().add("Right Selected");
            }
            // Start button should only be pressed after robot is placed in starting position. Init
            // auto assumes the robot is in it's starting position
            else if(gamepad1.start)
            {
                telemetry.log().add("Setup complete. Initializing...");
                break;
            }
            telemetry.update();
            idle();
        }

        initHardware();
        initAuto();

        telemetry.log().add("Initialized. Ready to start!");

        waitForStart();

        vuforiaLocator.startTracking();

        pressLeftBeacon();
        pressRightBeacon();

        // TODO: Remove if no longer used. Only here in case we need it
//        // Go in front of left side of beacon
//        goToLocation(1385, 3550, 90.0);
//        sleep(500);
//        // Check color of left side
//        if(colorSensor.red() >= colorSensor.blue())
//        {
//            // Press left side if it's red
//            telemetry.log().add("Left is red");
//            goToLocation(1385, 3630, 90.0);
//            sleep(500);
//            goToLocation(1385, 3550, 90.0);
//            sleep(500);
//        }
//        else
//        {
//            // Press right side if it's blue
//            telemetry.log().add("Left is blue");
//            goToLocation(1515, 3550, 90.0);
//            sleep(500);
//            goToLocation(1515, 3625, 90.0);
//            sleep(500);
//            goToLocation(1515, 3550, 90.0);
//            sleep(500);
//        }

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(10000);
    }

    private void pressLeftBeacon() throws InterruptedException
    {
        pressBeacon(1524, 3657.6);
    }

    private void pressRightBeacon() throws InterruptedException
    {
        pressBeacon(2743.2, 3657.6);
    }

    private void pressBeacon(double beaconX, double beaconY) throws InterruptedException
    {
        // TODO: Do we need sleep commands in here?
        double angleToEndOfTape = Math.atan2(beaconY - robotY - 450, beaconX - robotX);

        // Go to the end of the tape in front of the beacon
        turnToAngle(angleToEndOfTape);
        driveToPoint(beaconX, beaconY - 450, angleToEndOfTape);
        turnToAngle(90);

        // Give Vuforia a chance to start tracking the target
        sleep(1000);

        // Only actually looks if vision target isn't visible
        //lookForVisionTarget();

        // Reposition after tracking target
        driveToPoint(beaconX, beaconY - 450, 90);

        // Get colors of both sides of beacon. Parameters are in mm from center of vision target
        int colorLeft = vuforiaLocator.getPixelColor(-60, 230, 30);
        int colorRight = vuforiaLocator.getPixelColor(60, 230, 30);

        // Check which side is more blue to determine the color of each side. The red value
        // doesn't change as much as blue for some reason, so we compare the blue values
        if(Color.blue(colorRight) > Color.blue(colorLeft))
        {
            // Press left side if it's red
            telemetry.log().add("Left is red");
            // Go in front of left button
            driveToPoint(beaconX - 140, beaconY - 100, 90);
            // Move forward to press button
            driveToPoint(beaconX - 140, beaconY - 35, 90);
            sleep(500);
        }
        else
        {
            // Press right side if it's red
            telemetry.log().add("Right is red");
            // Go in front of right button
            driveToPoint(beaconX - 10, beaconY - 100, 90.0);
            // Move forward to press button
            driveToPoint(beaconX - 10, beaconY - 35, 90.0);
            sleep(500);
        }

        // Back away from beacon
        driveToPoint(beaconX, beaconY - 450, 90);
    }

    // TODO: The methods below should only be temporary until we find a better solution to other alliance's targets giving bogus numbers
    // Updates robot's coordinates and angle. Only to be used when on red alliance. The blue
    // alliance's vision targets have been giving us some not so useful numbers, so we make sure to
    // only use the red targets
    void updateRobotLocationRed()
    {
        // Use Vuforia if a it's tracking something, and ensure it's not a blue target
        if(vuforiaLocator.isTracking()
                && !vuforiaLocator.getTargetName().equals("Target Blue Left")
                && !vuforiaLocator.getTargetName().equals("Target Blue Right"))
        {
            float[] location = vuforiaLocator.getRobotLocation();
            robotX = location[0];
            robotY = location[1];

            robotAngle = vuforiaLocator.getRobotAngle();
        }
        // Otherwise, use other sensors to determine distance travelled and angle
        else
        {
            robotAngle = imu.getAngularOrientation().firstAngle - headingOffset;

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
        }

        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();
    }

    // Turns to the specified angle
    void turnToAngle(double targetAngle) throws InterruptedException
    {
        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double ANGLE_TOLERANCE = 2.0; // In degrees

        while(Math.abs(deltaAngle) > ANGLE_TOLERANCE && opModeIsActive())
        {
            updateRobotLocationRed();

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
        // Calculate how far we are from target point
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double DISTANCE_TOLERANCE = 10; // In mm

        while(distanceToTarget > DISTANCE_TOLERANCE && opModeIsActive())
        {
            updateRobotLocationRed();

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
}
