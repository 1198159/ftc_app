package org.firstinspires.ftc.team8923_2017;

/**
 * Runable shell for Master Autonomous code
 */

public class AutonomousCompetition extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        ChooseOptions();

        InitAuto();

        waitForStart();

        while (opModeIsActive())
        {
            Run();
        }

    }
}
