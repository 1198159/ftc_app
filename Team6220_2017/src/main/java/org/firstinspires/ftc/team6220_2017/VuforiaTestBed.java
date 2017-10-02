package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Cole Welch on 10/1/2017.
 */

@Autonomous(name = "AutoCompetition", group = "Autonomous")
public class VuforiaTestBed extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        vuforiaHelper.startTracking();

        waitForStart();

        
    }
}
