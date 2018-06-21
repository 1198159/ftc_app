package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rev Perf Motor", group = "Autonomous")

/*
   Observed Data

                     median         max        average
   Moto G Play




   ZTE Speed



   Rev Control Hub  



 */



public class RevPerformanceTestMotor extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //we want 100001 measurements to make the calculation of median easy
        final int NUM_LOOPS = 100001;
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
