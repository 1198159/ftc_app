package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Cap Ball Grabber Test", group = "Tests")
public class TestCapBallGrabber extends LinearOpMode
{
    private Servo servoLeft;
    private Servo servoRight;

    private double servoLeftPosition = 0.0;
    private double servoRightPosition = 1.0;

    private static final double DELTA_POSITION = 0.05;

    private boolean wasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        servoRight = hardwareMap.servo.get("servoGrabberRight");
        servoLeft = hardwareMap.servo.get("servoGrabberLeft");

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.dpad_up && !wasPressed)
            {
                servoRightPosition += DELTA_POSITION;
                servoLeftPosition -= DELTA_POSITION;
                wasPressed = true;
            }
            else if(gamepad1.dpad_down && !wasPressed)
            {
                servoRightPosition -= DELTA_POSITION;
                servoLeftPosition += DELTA_POSITION;
                wasPressed = true;
            }
            else if(!gamepad1.dpad_down && !gamepad1.dpad_up)
                wasPressed = false;

            servoRight.setPosition(servoRightPosition);
            servoLeft.setPosition(servoLeftPosition);

            telemetry.addData("Left", servoLeftPosition);
            telemetry.addData("Right", servoRightPosition);

            telemetry.update();
            idle();
        }
    }
}
