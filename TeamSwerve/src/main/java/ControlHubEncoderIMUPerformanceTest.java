import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


// This class exists for the purpose of testing the length of time required to retrieve imu and
// encoder values from the REV hub.  Currently, it is set up to use a 4 motor robot for testing.
// Many samples are taken over a large number of loops to get more meaningful data.  Retrieval
// times of encoder values and the z-axis orientation of the imu are measured separately and together.

@Autonomous(name = "EncoderIMUPerfTest", group = "Autonomous")
public class ControlHubEncoderIMUPerformanceTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        final int NUM_LOOPS = 1001;
        int loops = 0;
        double curEncoderValue;
        double encoderSum = 0;
        double[] loopTimesMS = new double[loops];
        ElapsedTime loopTime = new ElapsedTime();


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


        waitForStart();
        loopTime.reset();


        while (opModeIsActive())
        {
            // Go through a preset number of loops while creating an array of the loop times
            while (loops < NUM_LOOPS)
            {
                curEncoderValue = motorBackLeft.getCurrentPosition();

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
            double maxTime = loopTimesMS[NUM_LOOPS - 1];

            // Display statistics
            while (opModeIsActive())
            {
                telemetry.addData("med: ", medianTime);
                telemetry.addData("max: ", maxTime);
                telemetry.addData("avg: ", averageTime);
                telemetry.update();
                idle();
            }

            idle();
        }
    }
}
