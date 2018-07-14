import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

// This opmode is used to test whether a motor attached to a pulley can both lift and lower a weight
// effectively using the default PID parameters.  We are convinced that the same parameters cannot
// be used to do both tasks properly, so this test should be primarily a confirmation of what we
// already know.

@TeleOp(name = "Pulley Test", group = "TeleOp")
public class PulleyTest extends LinearOpMode
{
    DcMotorEx motor;

    // Allows us to change which mode the motor is running in each loop
    boolean isUsingRunToPosition = true;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize motor
        motor = (DcMotorEx)hardwareMap.get(DcMotor.class, "motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Allows us to display the PID coefficients for reference
        PIDCoefficients coefficients = motor.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();


        // Tell the operator which buttons move the weight up and down and also what the PID coefficients are
        telemetry.addData("Pull up", "y");
        telemetry.addData("Pull down", "a");
        telemetry.addData("P,I,D", "%f, %f, %f", coefficients.p, coefficients.i, coefficients.d);  // These read about 10, 0.05, 0
        telemetry.update();

        // Main loop
        while (opModeIsActive())
        {
            // Switch between running with and without encoders
            if (gamepad1.right_bumper)
            {
                if (isUsingRunToPosition)
                {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    isUsingRunToPosition = false;
                }
                else
                {
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    isUsingRunToPosition = true;
                }
            }

            // Run the motor in the mode it has been set to
            if (isUsingRunToPosition)
            {
                // Pull up
                if (gamepad1.y)
                {
                    motor.setTargetPosition(4800);
                    motor.setPower(1.0);
                }
                // Pull down
                else if (gamepad1.a)
                {
                    motor.setTargetPosition(0);
                    motor.setPower(1.0);
                }
            }
            else
            {
                // Run with joystick
                motor.setPower(gamepad1.right_stick_y);
            }

            idle();
        }
    }
}
