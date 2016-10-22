package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 *  Runs an Autonomous program for the second position on the wall of the field that the
 *  red alliance drivers are stationed at.
 */
@Autonomous(name = "Auto Red 2", group = "Autonomous")
public class AutonomousRed2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        robotX = RED_2_START_X;
        robotY = RED_2_START_Y;
        robotAngle = RED_2_START_ANGLE;
        resetHeadingOffset();

        waitForStart();

        vuforiaLocator.startTracking();

        // Go to left side of beacon
        goToLocation(1385, 3550, 90.0);
        sleep(500);
        // Check color of left side
        if(colorSensor.red() >= colorSensor.blue())
        {
            // Press left side if it's red
            telemetry.log().add("Left is red");
            goToLocation(1385, 3630, 90.0);
            sleep(500);
            goToLocation(1385, 3550, 90.0);
            sleep(500);
        }
        else
        {
            // Press right side if it's blue
            telemetry.log().add("Left is blue");
            goToLocation(1515, 3550, 90.0);
            sleep(500);
            goToLocation(1515, 3630, 90.0);
            sleep(500);
            goToLocation(1515, 3550, 90.0);
            sleep(500);
        }

        // TODO: Press other beacon button

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(5000);
    }
}
