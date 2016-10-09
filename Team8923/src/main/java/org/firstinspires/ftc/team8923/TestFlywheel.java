package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Flywheel Test", group = "Tests")
public class TestFlywheel extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private double powerLeft = 0.5;
    private double powerRight = 0.5;

    private static final double DELTA_POWER = 0.05;

    private boolean isRunning = false;
    private boolean isUsingEncoders = false;
    private boolean leftWasPressed = false;
    private boolean rightWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        waitForStart();

        while(opModeIsActive())
        {
            // Toggle run state and encoder state
            if(gamepad1.start)
                isRunning = !isRunning;
            if(gamepad1.back)
                isUsingEncoders = !isUsingEncoders;

            // Adjust motor powers. Dpad up and down for left motor, and a and y buttons for right
            if(gamepad1.dpad_up && !leftWasPressed)
            {
                powerLeft += DELTA_POWER;
                leftWasPressed = true;
            }
            else if(gamepad1.dpad_down && !leftWasPressed)
            {
                powerLeft -= DELTA_POWER;
                leftWasPressed = true;
            }
            else if(!gamepad1.dpad_down && !gamepad1.dpad_up)
                leftWasPressed = false;

            if(gamepad1.y && !leftWasPressed)
            {
                powerRight += DELTA_POWER;
                rightWasPressed= true;
            }
            else if(gamepad1.a && !leftWasPressed)
            {
                powerRight -= DELTA_POWER;
                rightWasPressed = true;
            }
            else if(!gamepad1.y && !gamepad1.a)
                rightWasPressed = false;

            // Toggle encoders if necessary
            if(isUsingEncoders && motorLeft.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
            {
                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(!isUsingEncoders && motorLeft.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            {
                motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Set power if running or stop
            if(isRunning)
            {
                motorLeft.setPower(powerLeft);
                motorRight.setPower(powerRight);
            }
            else
            {
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
            }

            // Inform drivers
            telemetry.addData("Left", powerLeft);
            telemetry.addData("Right", powerRight);
            telemetry.addData("Using Encoders", isUsingEncoders);

            telemetry.update();
            idle();
        }
    }
}
