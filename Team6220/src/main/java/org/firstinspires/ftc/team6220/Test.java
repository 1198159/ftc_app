package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Mridula on 11/14/2016.
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
            idle();
        }

        telemetry.addData("Time: ", newTime);
        telemetry.update();
    }
}
