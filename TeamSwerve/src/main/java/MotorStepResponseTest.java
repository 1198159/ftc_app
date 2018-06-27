import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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
        double[] motorPowers = new double[200];
        double sampleTime = 10;     // This number is in milliseconds

        // Set up motors in configuration
        DcMotor motorBackLeft;
        DcMotor motorBackRight;
        DcMotor motorFrontLeft;
        DcMotor motorFrontRight;
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        motorBackLeft.setPower(-1.0);
        motorBackRight.setPower(1.0);
        motorFrontLeft.setPower(-1.0);
        motorFrontRight.setPower(1.0);

        loopTime.reset();

        // Go through the specified number of loops while sampling motor power at set increments
        for (int i = 0; i < NUM_LOOPS; i++)
        {
            loopTime.reset();
            motorPowers[i] = motorBackLeft.getPower();

            while (loopTime.milliseconds() < sampleTime && opModeIsActive())
            {
                idle();
            }
        }

        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);

        // Display the data
        System.out.println(motorPowers);

        // Let the program keep running until we stop it
        while (opModeIsActive())
            idle();
    }
}
