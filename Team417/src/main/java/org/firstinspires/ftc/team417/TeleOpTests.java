package org.firstinspires.ftc.team417;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOpTests", group = "Swerve")
public class TeleOpTests extends LinearOpMode
{
    DcMotor motorLauncher;
    double startPos;
    double endPos;
    double difPos;

    // TODO: change config of the one motor file from "motor" to "motorLauncher"

    public void runOpMode()
    {
        // Connect to motor
        motorLauncher = hardwareMap.dcMotor.get("motorLauncher");

        //motorLauncher.setDirection(DcMotor.Direction.REVERSE);
        motorLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //motor.setMaxSpeed(1230); // set to ticks per second
        motorLauncher.setMaxSpeed(1157); // set to ticks per second
        //motorLauncher.setMaxSpeed(800);
        motorLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motor");
        telemetry.update();

        waitForStart();

        startPos = motorLauncher.getCurrentPosition();
        motorLauncher.setPower(1.0);
        sleep(10000);
        endPos = motorLauncher.getCurrentPosition();
        motorLauncher.setPower(0);
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
