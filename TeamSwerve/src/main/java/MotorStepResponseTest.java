import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;


// The purpose of this class is to empirically determine the step response of a motor.  This is
// accomplished by setting 4 motors to full power and sampling the power value of one after
// preset time increments.  The resulting data is placed in an array to be used for further analysis.

@Autonomous(name = "Motor Step Response Test", group = "Autonomous")
public class MotorStepResponseTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        final int NUM_LOOPS = 200;
        ElapsedTime loopTime = new ElapsedTime();
        // These two arrays correspond to the respective sets of x and y coordinates on a graph in Logger Pro
        double[] encoderSpeeds = new double[NUM_LOOPS];
        double[] encoderTimes = new double[NUM_LOOPS];
        double sampleTime = 0;
        // Used to store encoder differences each loop to find time rate of change of encoder counts
        double newEncValBL = 0;
        double newEncValBR = 0;
        double newEncValFL = 0;
        double newEncValFR = 0;
        double oldEncValBL = 0;
        double oldEncValBR = 0;
        double oldEncValFL = 0;
        double oldEncValFR = 0;

        // Set up motors in configuration
        DcMotor motorBackLeft;
        DcMotor motorBackRight;
        DcMotor motorFrontLeft;
        DcMotor motorFrontRight;

        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        loopTime.reset();


        motorBackLeft.setPower(-1.0);
        motorBackRight.setPower(1.0);
        motorFrontLeft.setPower(-1.0);
        motorFrontRight.setPower(1.0);


        // Go through the specified number of loops while storing motor power and time signature for each loop
        for (int i = 0; i < NUM_LOOPS; i++)
        {
            // Store old encoder value(s)
            oldEncValBL = newEncValBL;
            //oldEncValBR = newEncValBR;
            //oldEncValFL = newEncValFL;
            //oldEncValFR = newEncValFR;

            // Store encoder value(s) from current loop
            newEncValBL = motorBackLeft.getCurrentPosition();
            //newEncValBR = motorBackRight.getCurrentPosition();
            //newEncValFL = motorFrontLeft.getCurrentPosition();
            //newEncValFR = motorFrontRight.getCurrentPosition();

            // Measure the time at which the measurement occurs and how much time has passed since last loop
            encoderTimes[i] = loopTime.seconds();

            if (i >= 1)
            {
                sampleTime = encoderTimes[i] - encoderTimes[i - 1];

                // Calculate rotational velocity of motor in ticks / s
                encoderSpeeds[i] = (-newEncValBL + oldEncValBL) / sampleTime;
                //encoderSpeeds[i] = ((-newEncValBL + newEncValBR - newEncValFL + newEncValFR) / 4 -
                //        (-oldEncValBL + oldEncValBR - newEncValFL + newEncValFR) / 4) / (sampleTime);
            }
        }

        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);


        // Display the data
        System.out.println(Arrays.toString(encoderTimes));
        System.out.println(Arrays.toString(encoderSpeeds));
    }
}
