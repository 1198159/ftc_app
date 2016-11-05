package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This opmode is used to experiment with autonomous ideas before implementing them in our competition code.
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
            drive.navigateTo(new Transform2D(1.0,1.0,0));

            idle();
        }
    }
}
