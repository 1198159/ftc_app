package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FilterTest", group = "Swerve")
public class FilterTest extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        double[] weights1 = new double[20];
        // set the 20 weights
        for (int i = 0; i < 20; i++)
        {
            weights1[i] = 1.0/10.0; // so you don't have to divide inside of the getRunningTotal method
        }
        // create an instance of the weighted moving
        WeightedMovingAverage filter1 = new WeightedMovingAverage(weights1);
        filter1.addNewValue(1.0);
        telemetry.addData("Filter1", filter1.getRunningTotal());
        telemetry.update();
        sleep(5000);
    }
}