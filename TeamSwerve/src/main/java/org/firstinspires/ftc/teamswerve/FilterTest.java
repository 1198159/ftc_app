package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by user on 8/22/2017.
 */

@TeleOp(name="FilterTest", group = "Swerve")
public class FilterTest extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        double[] weights1 = new double[10];
        for (int i = 0; i < 10; i++)
        {
            weights1[i] = 1.0;
        }
        WeightedMovingAverage filter1 = new WeightedMovingAverage(weights1);
        filter1.addNewValue(1.0);
        telemetry.addData("Filter1",filter1.getWeightedAverage());
        telemetry.update();
        sleep(5000);
    }
}