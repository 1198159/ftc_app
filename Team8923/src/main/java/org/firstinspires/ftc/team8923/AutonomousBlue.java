package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

/*
 *  Autonomous OpMode for blue alliance. The OpMode is setup with a gamepad during initialization,
 *  so robot can start at one of two locations, and can complete any objective in any order
 */
@Autonomous(name = "Auto Blue", group = "Autonomous")
public class AutonomousBlue extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.log().add("Starting Position: Press x for left, b for right");
        telemetry.log().add("Press start button when robot is on field");
        telemetry.update();

        // TODO: Add code to use gamepad to setup autonomous routine
        // Used to setup autonomous routine
        while(opModeIsActive())
        {
            if(gamepad1.x)
            {
                // Robot will start on left
                robotX = BLUE_LEFT_START_X;
                robotY = BLUE_LEFT_START_Y;
                robotAngle = BLUE_LEFT_START_ANGLE;
                telemetry.log().add("Left Selected");
            }
            else if(gamepad1.b)
            {
                // Robot will start on right
                robotX = BLUE_RIGHT_START_X;
                robotY = BLUE_RIGHT_START_Y;
                robotAngle = BLUE_RIGHT_START_ANGLE;
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

        pressRightBeacon();
        pressLeftBeacon();

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(5000);
    }

    private void pressLeftBeacon() throws InterruptedException
    {
        pressBeacon(3657.6, 2743.2);
    }

    private void pressRightBeacon() throws InterruptedException
    {
        pressBeacon(3657.6, 1524);
    }

    private void pressBeacon(double beaconX, double beaconY) throws InterruptedException
    {
        // TODO: Do we need sleep commands in here?
        double angleToEndOfTape = Math.atan2(beaconX - robotX - 450, beaconY - robotY);

        // Go to the end of the tape in front of the beacon
        driveToPoint(beaconX - 450, beaconY, angleToEndOfTape);
        turnToAngle(0);
        // Give Vuforia a chance to start tracking the target
        sleep(1000);

        // Get colors of both sides of beacon. Parameters are in mm from center of vision target
        int colorLeft = vuforiaLocator.getPixelColor(-60, 230, 30);
        int colorRight = vuforiaLocator.getPixelColor(60, 230, 30);

        // Check which side is more blue to determine which side is which color. The red value
        // doesn't change as much as blue for some reason, so we compare the blue values
        if(Color.blue(colorRight) > Color.blue(colorLeft))
        {
            // Press right side if it's blue
            telemetry.log().add("Right is blue");
            // Go in front of right button
            driveToPoint(beaconX - 100, beaconY + 10, 0.0);
            // Move forward to press button
            driveToPoint(beaconX - 25, beaconY + 10, 0.0);
            sleep(500);
        }
        else
        {
            // Press left side if it's blue
            telemetry.log().add("Left is blue");
            // Go in front of left button
            driveToPoint(beaconX - 100, beaconY + 140, 0);
            // Move forward to press button
            driveToPoint(beaconX - 25, beaconY + 140, 0);
            sleep(500);
        }

        // Back away from beacon
        driveToPoint(beaconX - 450, beaconY, 0);
    }

    // TODO: The methods below should only be temporary until we find a better solution to other alliance's targets giving bogus numbers
    // Updates robot's coordinates and angle. Only to be used when on blue alliance. The red
    // alliance's vision targets have been giving us some not so useful numbers, so we make sure to
    // only use the blue targets
    void updateRobotLocationBlue()
    {
        // Use Vuforia if a it's tracking something, and ensure it's not a red target
        if(vuforiaLocator.isTracking()
                && !vuforiaLocator.getTargetName().equals("Target Red Left")
                && !vuforiaLocator.getTargetName().equals("Target Red Right"))
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
            updateRobotLocationBlue();

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
            updateRobotLocationBlue();

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
