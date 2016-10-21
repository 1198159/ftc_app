package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Colew on 10/18/2016.
 */

@Autonomous(name = "AutoCompetition", group = "Autonomous")
public class AutonomousCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {



            idle();
        }
    }
}
