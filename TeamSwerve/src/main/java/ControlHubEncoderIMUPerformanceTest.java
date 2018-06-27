import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


// This class exists for the purpose of testing the length of time required to retrieve imu and
// encoder values from the REV hub.  Currently, it is set up to use a 4 motor robot for testing.
// Many samples are taken over a large number of loops to get more meaningful data.  Retrieval
// times of encoder values and the z-axis orientation of the imu are measured separately and together.

@Autonomous(name = "Encoder/IMU Perf Test", group = "Autonomous")
public class ControlHubEncoderIMUPerformanceTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        final int NUM_LOOPS = 1001;
        int loops = 0;
        double curEncoderValue1 = 0;
        double curEncoderValue2 = 0;
        double curEncoderValue3 = 0;
        double curEncoderValue4 = 0;
        double encoderSum = 0;
        double[] loopTimesMS = new double[NUM_LOOPS];
        ElapsedTime loopTime = new ElapsedTime();
        double heading = 0;


        // Set up motors in configuration; some motors may or may not be used for measurements
        DcMotor motorBackLeft;
        DcMotor motorBackRight;
        DcMotor motorFrontLeft;
        DcMotor motorFrontRight;
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();
        loopTime.reset();


        while (opModeIsActive())
        {
            // Go through a preset number of loops while creating an array of the loop times
            while (loops < NUM_LOOPS)
            {
                curEncoderValue1 = motorBackLeft.getCurrentPosition();
                curEncoderValue2 = motorBackRight.getCurrentPosition();
                curEncoderValue3 = motorFrontLeft.getCurrentPosition();
                curEncoderValue4 = motorFrontRight.getCurrentPosition();

                //heading = imu.getAngularOrientation().firstAngle;
                //telemetry.addData("Heading: ", heading);
                //telemetry.update();

                loopTimesMS[loops] = loopTime.milliseconds();    // Get time elapsed for each loop
                loopTime.reset();   // Reset timer so we only get time elapsed for individual loops

                loops++;
                idle();
            }

        // Calculate statistics for the data set of times

            for (int i = 0; i < NUM_LOOPS; i++)
                encoderSum += loopTimesMS[i];

            // Sort the array to make calculations easier
            java.util.Arrays.sort(loopTimesMS);
            double averageTime = encoderSum / NUM_LOOPS;
            double medianTime = loopTimesMS[(NUM_LOOPS - 1)/ 2];
            double std = 0;
            double maxTime = loopTimesMS[NUM_LOOPS - 1];
            double minTime = loopTimesMS[0];

            for (int i = 0; i < NUM_LOOPS; i++)
            {
                std += Math.pow(loopTimesMS[i] - averageTime, 2);
            }

            std = Math.sqrt(std / (NUM_LOOPS - 1));

            // Display statistics
            while (opModeIsActive())
            {
                telemetry.addData("avg: ", averageTime);
                telemetry.addData("med: ", medianTime);
                telemetry.addData("std: ", std);
                telemetry.addData("min: ", minTime);
                telemetry.addData("max: ", maxTime);
                telemetry.update();
                idle();
            }

            idle();
        }
    }
}
