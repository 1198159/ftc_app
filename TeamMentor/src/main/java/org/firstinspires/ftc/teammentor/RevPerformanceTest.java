package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rev Perf Loop", group = "Autonomous")

/*
   Observed Data

                     median         max        average
   Moto G Play      0.0033ms     1.2018ms      0.00367ms
+ expansion hub     0.0039ms     0.6520ms      0.00429ms
                    0.0040ms     2.2790ms      0.00420ms


   ZTE Speed        0.0038ms     4.0434ms      0.0045ms
+ expansion hub     0.0038ms     7.8806ms      0.0045ms
                    0.0039ms     1.6442ms      0.0043ms

   Rev Control Hub  0.0032ms     1.0332ms     0.0035ms
   (with moto ds)   0.0032ms     1.5031ms     0.0035ms
                    0.0032ms     1.7779ms     0.0034ms



 */



public class RevPerformanceTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //we want 100001 measurements to make the calculation of median easy
        final int NUM_LOOPS = 100001;
        int loops = NUM_LOOPS;

        double[] loopTimesMS = new double[loops];

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        time.reset();

        idle(); //force a loop during which nothing happens for the first measurement

        while (opModeIsActive())
        {

            while (loops > 0)
            {
                loops--;

                loopTimesMS[loops] = time.milliseconds(); //store this loop time

                time.reset(); //reset the timer for the next loop

                idle();
            }

            //perform analysis
            //sort the array to make calculating stats easier
            java.util.Arrays.sort(loopTimesMS);
            double median = loopTimesMS[NUM_LOOPS/2];
            double max = loopTimesMS[NUM_LOOPS -1];

            double sum = 0;
            for (int i = 0; i< NUM_LOOPS; i++) sum += loopTimesMS[i];

            double average  = sum / NUM_LOOPS;

            while (opModeIsActive())
            {
                telemetry.addData("med", median);
                telemetry.addData("max", max);
                telemetry.addData("avg", average);
                telemetry.update();
                idle();
            }

        }

    }
}
