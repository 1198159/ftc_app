package org.firstinspires.ftc.team8923;

/*
    Runs an Autonomous program for the second position on the wall of the field that the red alliance drivers are stationed at.
 */
public class Autonomous_RED_2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();
        // ALL MEASUREMENTS ARE IN MILLIMETERS
        double RED_2_START_X = 200.0;
        double RED_2_START_Y = 1880.0;

        RED_2_START_X = robotX;
        RED_2_START_Y = robotY;

        waitForStart();

        while(opModeIsActive())
        {
            goToLocation(1505, 1880, 90.0);

            goToLocation(1505, 3350, 90.0);

            telemetry.update();
            idle();
        }


    }
}
