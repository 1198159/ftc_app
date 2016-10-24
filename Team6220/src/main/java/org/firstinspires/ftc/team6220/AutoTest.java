package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Colew on 10/23/2016.
 */

@Autonomous(name = "AutoTest", group = "Autonomous")
public class AutoTest extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //@TODO: use super to encapsulate initialization
        initializeHardware();

        while (opModeIsActive())
        {
            //drives to a location (units in meters)
            DriveTo(1.0, 3.0);

            idle();
        }
    }
}
