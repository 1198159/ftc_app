package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rev Perf Motor", group = "Autonomous")

/*
   Observed Data

                     median         max        average
   Moto G Play      3.0065ms      10.3746ms     3.1120ms
+ expansion hub     3.0126ms      11.5055ms     3.1195ms
                    3.0014ms       9.9144ms     3.1030ms


   ZTE Speed        3.7333ms      21.5240ms     3.9167ms
+ expansion hub     3.6964ms      35.7192ms     3.8869ms
                    3.6409ms      18.7912ms     3.8416ms



   Rev Control Hub  2.7073ms      19.0841ms     3.0068ms
   (with Moto ds)   2.7108ms      14.4369ms     3.0065ms
                    2.7190ms      14.9063ms     3.0044ms



 */



public class RevPerformanceTestMotor extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //we want 100001 measurements to make the calculation of median easy
        final int NUM_LOOPS = 10001;
        int loops = NUM_LOOPS;

        double[] loopTimesMS = new double[loops];

        ElapsedTime time = new ElapsedTime();

        DcMotor motor = hardwareMap.dcMotor.get("motor");

        //use two different motor powers so we're always writing a different value than the last loop
        double[] motorPower = new double[2];
        motorPower[0] = 0.5;
        motorPower[1] = 0.4;

        waitForStart();

        time.reset();

        motor.setPower(motorPower[0]);
        idle(); //force a loop during which nothing happens for the first measurement

        while (opModeIsActive())
        {

            while (loops > 0)
            {
                loops--;

                loopTimesMS[loops] = time.milliseconds(); //store this loop time

                time.reset(); //reset the timer for the next loop

                motor.setPower(motorPower[loops % 2]);
                idle();
            }

            motor.setPower(0);

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
