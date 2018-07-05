import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;


// The purpose of this class is to test a simpler case of the step response program.  The resulting
// data will be used to determine what is incorrect in the theoretical model that is currently
// being used to predict the behavior of FTC motors.

@Autonomous(name = "3 Wheel Test Robot", group = "Autonomous")
public class ThreeWheelTestRobot extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        final int NUM_LOOPS = 200;
        ElapsedTime loopTime = new ElapsedTime();
        // For plotting distance vs. time
        double[] encValues = new double[NUM_LOOPS];
        // For plotting angular velocity vs. time
        double[] encoderSpeeds = new double[NUM_LOOPS];
        double[] encoderFilteredSpeeds = new double[NUM_LOOPS];
        double sampleTime = 10;     // This number is in milliseconds
        double newEncVal = 0;
        double oldEncVal = 0;

        // Initialize drive motor, which powers the front wheel
        DcMotor motor;
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();


        motor.setPower(1.0);

        // Go through the specified number of loops while sampling distance traveled from encoders
        for (int i = 0; i < NUM_LOOPS; i++)
        {
                //** Use this code block for angular velocity vs. time
            loopTime.reset();

            oldEncVal = newEncVal;
            newEncVal = motor.getCurrentPosition();

            // Calculate time rate of change of encoder values; sampleTime is converted to seconds
            encoderSpeeds[i] = (-newEncVal + oldEncVal) / (0.001 * sampleTime);
            //encoderSpeeds[i] = ((-newEncValBL + newEncValBR - newEncValFL + newEncValFR) / 4
            //        -(-oldEncValBL + oldEncValBR - oldEncValFL + oldEncValFR) / 4) / (0.001 * sampleTime);

            // Filter data using a triangular filter.  We must modify our calculations for the first
            // 2 values of i due to our method of filtering
            if (i == 6)
            {
                encoderFilteredSpeeds[0] = encoderSpeeds[0];
                encoderFilteredSpeeds[1] = (encoderSpeeds[0] + 2 * encoderSpeeds[1] + encoderSpeeds[2]) / 4;
                encoderFilteredSpeeds[2] = (encoderSpeeds[0] + 2 * encoderSpeeds[1] + 3 * encoderSpeeds[2] +
                        2 * encoderSpeeds[3] + encoderSpeeds[4]) / 9;
                encoderFilteredSpeeds[3] = (encoderSpeeds[0] + 2 * encoderSpeeds[1] + 3 * encoderSpeeds[2] +
                        4 * encoderSpeeds[3] + 3 * encoderSpeeds[4] + 2 * encoderSpeeds[5] + encoderSpeeds[6]) / 16;
            }
            else if (i >= 8)
            {
                encoderFilteredSpeeds[i - 4] = (encoderSpeeds[i - 8] + 2 * encoderSpeeds[i - 7] +
                        3 * encoderSpeeds[i - 6] + 4 * encoderSpeeds[i - 5] + 5 * encoderSpeeds[i - 4] +
                        4 * encoderSpeeds[i - 3] + 3 * encoderSpeeds[i - 2] + 2 * encoderSpeeds[i - 1] +
                        encoderSpeeds[i]);
            }

            while (loopTime.milliseconds() < sampleTime && opModeIsActive())
                idle();

                //** Use this code block for distance vs. time
            /*
            loopTime.reset();

            encValues[i] = -motor.getCurrentPosition();

            while (loopTime.milliseconds() < sampleTime)
                idle();
            */
        }

        motor.setPower(0.0);

        // Display the data
        System.out.println(Arrays.toString(encValues));
    }

}
