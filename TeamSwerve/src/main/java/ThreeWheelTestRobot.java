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
        final int NUM_LOOPS = 250;
        ElapsedTime loopTime = new ElapsedTime();
        double[] encValues = new double[NUM_LOOPS];
        double sampleTime = 8;     // This number is in milliseconds

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
            loopTime.reset();

            encValues[i] = -motor.getCurrentPosition();

            while (loopTime.milliseconds() < sampleTime)
                idle();
        }

        motor.setPower(0.0);

        // Display the data
        System.out.println(Arrays.toString(encValues));
    }

}
