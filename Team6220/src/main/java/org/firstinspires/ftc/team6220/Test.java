package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
    Move code to for test how long a loop takes
 */
public class Test extends LinearOpMode
{
    public void runOpMode() throws InterruptedException
    {
        double initialTime = 0.0;
        double newTime = 0.0;

        while(opModeIsActive())
        {
            newTime = (System.nanoTime() - initialTime)/1000/1000;
            initialTime = System.nanoTime();
            telemetry.addData("Time: ", newTime);
            telemetry.update();
            idle();
        }

    }
}
