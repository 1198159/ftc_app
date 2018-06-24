package org.firstinspires.ftc.teammentor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rev Perf IMU", group = "Autonomous")

/*
   Observed Data

                     median         max        average
   Moto G Play      18.0505ms     28.2150ms    17.9056ms
+ expansion hub     18.0230ms     26.6713ms    17.8500ms
                    18.0544ms     27.4475ms    17.9331ms


   ZTE Speed        --trial 1 took so long that I eventually cancelled it
+ expansion hub     19.1030ms    214.5130ms    19.2723ms
                    19.5522ms     43.2745ms    19.5698ms
                    --trial 4 took so long that I eventually cancelled it
                    --trial 5 took over 20 minutes so I cancelled it


   Rev Control Hub  17.8799ms    147.6815ms    18.0032ms
   with moto g ds   17.9993ms     36.7189ms    18.1117ms
                    17.9812ms     37.3044ms    18.0364ms


 */



public class RevPerformanceTestIMU extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //we want 100001 measurements to make the calculation of median easy
        final int NUM_LOOPS = 10001;
        int loops = NUM_LOOPS;

        double[] loopTimesMS = new double[loops];

        ElapsedTime time = new ElapsedTime();

        //configure the IMU
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        double heading = 0.0;

        waitForStart();

        time.reset();

        heading = imu.getAngularOrientation().firstAngle;
        idle(); //force a loop during which nothing happens for the first measurement

        while (opModeIsActive())
        {

            while (loops > 0)
            {
                loops--;

                loopTimesMS[loops] = time.milliseconds(); //store this loop time

                time.reset(); //reset the timer for the next loop

                heading = imu.getAngularOrientation().firstAngle;
                idle();
            }

            //perform analysis


                //output the data to a file before transforming it
                FileWriter file = new FileWriter("RevPerf.csv");

                for (int i=0; i < NUM_LOOPS; i++)
                {
                    file.println(loopTimesMS[i] + "");
                }

                file.closeFile();


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
