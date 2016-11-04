package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
}
