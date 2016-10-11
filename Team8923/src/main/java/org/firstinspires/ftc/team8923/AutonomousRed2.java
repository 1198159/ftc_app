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

        waitForStart();

        goToLocation(1505, 1880, 90.0);

        goToLocation(1505, 3350, 90.0);

        // TODO: Press beacon button
    }
}
