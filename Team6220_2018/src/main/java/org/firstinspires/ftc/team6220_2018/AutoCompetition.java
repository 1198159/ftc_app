package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 *  Note:  This program is not currently in use.  It may be useful as a template for the future, however.
 *  This program uses gamepad input prior to initialization to offer different routine options.
 */

// todo Update code
// todo Reimplement encoder navigation when it is ready
@Autonomous(name = "AutoCompetition", group = "Autonomous")
@Disabled

public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        runSetup();

        waitForStart();


        if (isBlueSide)
        {

        }
        else
        {

        }
    }
}
