package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by coles on 11/28/2017.
 */

@Autonomous(name="Test Autonomous Distance")
public class TestAutonomous extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        InitAuto();
        waitForStart();

        while (opModeIsActive())
        {
            driveToPoint(500, 0, 0, 0.8);
            sleep(3000);
            driveToPoint(500, -500, 0, 0.8);
            sleep(3000);
            driveToPoint(0, -500, 0, 0.8);
            sleep(3000);
            driveToPoint(0, 0, 0, 0.8);
            sleep(3000);
        }
    }
}
