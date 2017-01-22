package org.firstinspires.ftc.team417;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by user on 1/8/2017.
 */
@TeleOp(name="TeleOpTests", group = "Swerve")
public class TeleOpTests extends LinearOpMode
{
    DcMotor motor;
    double startPos;
    double endPos;
    double difPos;


    public void runOpMode()
    {
        // Connect to motor (Assume standard left wheel)
        motor = hardwareMap.dcMotor.get("motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMaxSpeed(1230); // set to ticks per second
        motor.setMaxSpeed(1157); // set to ticks per second
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motor" );
        telemetry.update();

        waitForStart();

        startPos = motor.getCurrentPosition();
        motor.setPower(0.75);
        sleep(10000);
        endPos = motor.getCurrentPosition();
        motor.setPower(0);
        difPos = endPos - startPos;
        telemetry.addData("startPos", startPos);
        telemetry.addData("endPos", endPos);
        telemetry.addData("difPos", difPos);

        while (opModeIsActive())
        {
            telemetry.update();
        }
        // Display the current value
        telemetry.addData(">", "Press Stop to end test." );
    }
}
